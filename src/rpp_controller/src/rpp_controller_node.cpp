#include "dyx_rpp_cpp17_core/path_index.hpp"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
using namespace std::chrono_literals;

// Native trajectory consumption, ordered projection and lookahead diagnostics.
// Deliberately read-only: no /fmu/in publishers; STOP/START authority belongs
// solely to mission_manager and PX4 control is not commissioned here.
class RppControllerNode final : public rclcpp::Node {
public:
 RppControllerNode():Node("rpp_controller") {
  const auto q=rclcpp::QoS(1).reliable().transient_local();
  out_=create_publisher<std_msgs::msg::String>("/dyx/rpp_controller/preview_status",10);
  detail_=create_publisher<std_msgs::msg::String>("/rpp_controller/geometry_status",10);
  declare_parameter<double>("lookahead_m",0.55);
  declare_parameter<double>("max_path_jump_m",0.5);
  pos_=create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",rclcpp::SensorDataQoS(),
   [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr p){
    std::lock_guard<std::mutex> g(mu_);
    const auto now=std::chrono::steady_clock::now();
    if(seen_ && (p->xy_reset_counter!=xy_reset_ || p->heading_reset_counter!=heading_reset_ ||
       (p->timestamp_sample!=0 && last_sample_!=0 && p->timestamp_sample<=last_sample_))) {
      position_fault_=true; tracker_.reset();
    }
    if(!seen_) {xy_reset_=p->xy_reset_counter;heading_reset_=p->heading_reset_counter;}
    seen_=true;last_sample_=p->timestamp_sample;position_time_=now;
    pose_valid_=p->xy_valid && p->v_xy_valid && p->heading_good_for_control &&
      !p->dead_reckoning && p->timestamp_sample!=0 &&
      std::isfinite(p->x)&&std::isfinite(p->y)&&std::isfinite(p->heading);
    east_=p->y;north_=p->x; // PX4 NED (North, East) -> ENU (East, North)
    heading_enu_=dyx::rpp::wrap_angle(dyx::rpp::pi/2.0-p->heading);
   });
  path_=create_subscription<nav_msgs::msg::Path>("/nav_path",q,[this](nav_msgs::msg::Path::ConstSharedPtr p){
    std::lock_guard<std::mutex> g(mu_);
    // This is diagnostic geometry only, not trusted mission authorization.
    try {
      if(p->header.frame_id!="map" && p->header.frame_id!="odom") throw std::invalid_argument("UNSUPPORTED_PATH_FRAME");
      if(p->poses.size()<2 || p->poses.size()>200000) throw std::invalid_argument("INVALID_PATH_COUNT");
      std::vector<dyx::rpp::RawPathPoint> points;points.reserve(p->poses.size());
      for(auto const& s:p->poses){
        if(!s.header.frame_id.empty() && s.header.frame_id!=p->header.frame_id)
          throw std::invalid_argument("MIXED_PATH_FRAME");
        const double x=s.pose.position.x,y=s.pose.position.y;
        if(!std::isfinite(x)||!std::isfinite(y)||std::abs(x)>10000||std::abs(y)>10000)
          throw std::invalid_argument("NONFINITE_OR_OUT_OF_BOUNDS_PATH");
        points.push_back({{x,y},dyx::rpp::point_pass_through,-1});
      }
      // Marking metadata arrives asynchronously; do not infer any point as a
      // physical marking stop from an interpolation-only path.
      geometry_=std::make_unique<dyx::rpp::PathGeometryIndex>(dyx::rpp::PathGeometryIndex::build(points));
      path_frame_=p->header.frame_id;path_error_.clear();path_time_=std::chrono::steady_clock::now();
      tracker_.reset();++path_revision_;
    } catch(const std::exception& e){geometry_.reset();tracker_.reset();path_error_=e.what();}
  });
  goal_=create_subscription<geometry_msgs::msg::PointStamped>("/dyx/mission/preview_goal_ned",5,
    [this](geometry_msgs::msg::PointStamped::ConstSharedPtr p){
      std::lock_guard<std::mutex> g(mu_);
      if(p->header.frame_id!="px4_local_ned"||!std::isfinite(p->point.x)||!std::isfinite(p->point.y)){
        goal_valid_=false;return;
      }
      goal_east_=p->point.y;goal_north_=p->point.x;goal_valid_=true;
    });
  timer_=create_wall_timer(50ms,[this]{publish();});
 }
private:
 void publish(){
  std::lock_guard<std::mutex> g(mu_);
  const auto now=std::chrono::steady_clock::now();
  const bool fresh=seen_ && now-position_time_<=250ms;
  const bool path_ok=bool(geometry_) && !geometry_->segments.empty();
  std::ostringstream s;
  s.setf(std::ios::fixed);s.precision(5);
  s<<"CXX17_RPP_PATH_OBSERVE;px4_position_fresh="<<(fresh?"true":"false")
   <<";pose_valid="<<(pose_valid_?"true":"false")
   <<";position_reset_fault="<<(position_fault_?"true":"false")
   <<";path_available="<<(path_ok?"true":"false")
   <<";path_revision="<<path_revision_
   <<";path_frame="<<path_frame_
   <<";path_error="<<path_error_;
  if(fresh && pose_valid_ && !position_fault_ && path_ok){
    try{
      // Project onto entire recorded path for diagnostic tracking. Movement
      // decisions require trusted semantic active-leg selection not yet wired.
      auto const& gidx=*geometry_;
      auto const& points=gidx.raw_points;
      // Last waypoint is temporarily a semantic endpoint for geometry only.
      // It cannot confer mission authority or trigger spray.
      auto distances=std::vector<double>{};distances.reserve(points.size());
      double cumulative=0.0;distances.push_back(0.0);
      for(size_t i=1;i<points.size();++i){cumulative+=std::hypot(points[i].point.x-points[i-1].point.x,points[i].point.y-points[i-1].point.y);distances.push_back(cumulative);}
      double best=std::numeric_limits<double>::infinity(),along=0.0,cross=0.0;
      for(size_t i=1;i<points.size();++i){
        const auto a=points[i-1].point,b=points[i].point;double dx=b.x-a.x,dy=b.y-a.y,len2=dx*dx+dy*dy;
        if(len2<=1e-12)continue;
        const double t=std::clamp(((east_-a.x)*dx+(north_-a.y)*dy)/len2,0.0,1.0);
        const double fx=a.x+t*dx,fy=a.y+t*dy,dist=std::hypot(east_-fx,north_-fy);
        if(dist<best){best=dist;along=distances[i-1]+t*std::sqrt(len2);cross=(dx*(north_-a.y)-dy*(east_-a.x))/std::sqrt(len2);}
      }
      const double look=get_parameter("lookahead_m").as_double();
      if(!std::isfinite(look)||look<0.0||look>5.0)throw std::invalid_argument("BAD_LOOKAHEAD");
      const auto target=gidx.point_at_s(std::min(gidx.total_length,along+look));
      s<<";cross_track_m="<<cross<<";nearest_distance_m="<<best
       <<";path_progress_m="<<along<<";remaining_path_m="<<std::max(0.0,gidx.total_length-along)
       <<";lookahead_east_m="<<target.point.x<<";lookahead_north_m="<<target.point.y
       <<";heading_enu_rad="<<heading_enu_;
    }catch(const std::exception& e){s<<";geometry_fault="<<e.what();}
  }
  if(fresh && pose_valid_ && goal_valid_)s<<";goal_radial_m="<<std::hypot(goal_east_-east_,goal_north_-north_);
  s<<";mission_authorized=false;motion_authorized=false;physical_command_emitted=false";
  std_msgs::msg::String m;m.data=s.str();out_->publish(m);detail_->publish(m);
 }
 std::mutex mu_;
 bool seen_{false},pose_valid_{false},position_fault_{false},goal_valid_{false};
 std::uint8_t xy_reset_{0},heading_reset_{0};std::uint64_t last_sample_{0};
 double east_{0},north_{0},heading_enu_{0},goal_east_{0},goal_north_{0};
 std::uint64_t path_revision_{0};std::string path_frame_{},path_error_{};
 std::chrono::steady_clock::time_point position_time_{},path_time_{};
 std::unique_ptr<dyx::rpp::PathGeometryIndex> geometry_;
 std::unique_ptr<dyx::rpp::GeometryProgressTracker> tracker_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr out_,detail_;
 rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_;
 rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_;
 rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_;
 rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<RppControllerNode>());rclcpp::shutdown();return 0;}
