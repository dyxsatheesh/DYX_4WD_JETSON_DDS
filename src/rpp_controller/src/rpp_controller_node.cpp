#include "dyx_rpp_cpp17_core/path_index.hpp"
#include "guidance_preview.hpp"
#include "semantic_metadata.hpp"
#include "active_leg_diagnostic.hpp"
#include "active_leg_execution.hpp"
#include "monotonic_leg_tracking.hpp"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rpp_interfaces/msg/rpp_preview.hpp>
#include <rpp_interfaces/msg/rpp_command.hpp>
#include <rpp_interfaces/msg/mission_safety_preview.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <algorithm>
#include <functional>
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
  // Distinct interface and topic from /rpp/command. No physical control path.
  shadow_pub_=create_publisher<rpp_interfaces::msg::RppPreview>(
    "/dyx/rpp/preview",rclcpp::QoS(5));
  // Deliberately NOT /rpp/command: preview has no actuation authority.
  shadow_command_pub_=create_publisher<rpp_interfaces::msg::RppCommand>(
    "/dyx/rpp/shadow_command",rclcpp::QoS(5));
  declare_parameter<double>("lookahead_m",0.55);
  declare_parameter<double>("max_path_jump_m",0.5);
  declare_parameter<double>("preview_cruise_mps",0.6);
  declare_parameter<double>("accel_distance_m",0.8);
  declare_parameter<double>("decel_distance_m",0.8);
  declare_parameter<double>("rear_master_to_pivot_m",0.42);
  pos_=create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",rclcpp::SensorDataQoS(),
   [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr p){
    std::lock_guard<std::mutex> g(mu_);
    const auto now=std::chrono::steady_clock::now();
    if(seen_ && (p->xy_reset_counter!=xy_reset_ || p->heading_reset_counter!=heading_reset_ ||
       (p->timestamp_sample!=0 && last_sample_!=0 && p->timestamp_sample<=last_sample_))) {
      position_fault_=true; leg_tracker_.reset();
    }
    if(!seen_) {xy_reset_=p->xy_reset_counter;heading_reset_=p->heading_reset_counter;}
    seen_=true;last_sample_=p->timestamp_sample;position_time_=now;
    pose_valid_=p->xy_valid && p->v_xy_valid && p->heading_good_for_control &&
      !p->dead_reckoning && p->timestamp_sample!=0 &&
      std::isfinite(p->x)&&std::isfinite(p->y)&&std::isfinite(p->heading);
    if(std::isfinite(p->vx) && std::isfinite(p->vy)) speed_mps_=std::hypot(p->vx,p->vy);
    else pose_valid_=false;
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
      // A latched path may be republished every second. Identical data MUST
      // NOT reset path progress or increment mission/path revision.
      if (path_frame_==p->header.frame_id && points.size()==last_path_points_.size()) {
        bool same=true;
        for(std::size_t i=0;i<points.size();++i) {
          if(points[i].point.x!=last_path_points_[i].point.x ||
             points[i].point.y!=last_path_points_[i].point.y) {same=false;break;}
        }
        if(same && geometry_) {path_time_=std::chrono::steady_clock::now();return;}
      }
      // Path replacement invalidates mission-selected active-leg preview.
      auto replacement=std::make_unique<dyx::rpp::PathGeometryIndex>(dyx::rpp::PathGeometryIndex::build(points));
      geometry_=std::move(replacement);
      last_path_points_=std::move(points);
      semantic_valid_=false;
      types_seen_=false; marks_seen_=false;
      types_.clear(); marks_.clear();
      // Reattempt only when both metadata topics are available. This remains
      // diagnostics; no START or actuation authority is derived from metadata.
      rebuild_semantics_locked();
      path_frame_=p->header.frame_id;path_error_.clear();path_time_=std::chrono::steady_clock::now();
      leg_tracker_.reset();++path_revision_;
    } catch(const std::exception& e){geometry_.reset();leg_tracker_.reset();last_path_points_.clear();path_error_=e.what();}
  });
  types_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
    "/trajectory_generator/path_types", q,
    [this](std_msgs::msg::UInt8MultiArray::ConstSharedPtr m){
      std::lock_guard<std::mutex> g(mu_);
      types_=m->data; types_seen_=true; rebuild_semantics_locked();
    });
  marks_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
    "/trajectory_generator/marking_indices", q,
    [this](std_msgs::msg::Int32MultiArray::ConstSharedPtr m){
      std::lock_guard<std::mutex> g(mu_);
      marks_=m->data; marks_seen_=true; rebuild_semantics_locked();
    });
  active_sub_ = create_subscription<std_msgs::msg::Int32>(
    "/dyx/mission/active_marking_index",q,
    [this](std_msgs::msg::Int32::ConstSharedPtr m){
      std::lock_guard<std::mutex> g(mu_);
      if(m->data != active_index_){active_index_=m->data;leg_tracker_.reset();}
    });
  goal_=create_subscription<geometry_msgs::msg::PointStamped>("/dyx/mission/preview_goal_ned",5,
    [this](geometry_msgs::msg::PointStamped::ConstSharedPtr p){
      std::lock_guard<std::mutex> g(mu_);
      if(p->header.frame_id!="px4_local_ned"||!std::isfinite(p->point.x)||!std::isfinite(p->point.y)){
        goal_valid_=false;return;
      }
      goal_east_=p->point.y;goal_north_=p->point.x;goal_valid_=true;
    });
  mission_safety_sub_ = create_subscription<rpp_interfaces::msg::MissionSafetyPreview>(
    "/dyx/mission/safety_preview", rclcpp::QoS(5),
    [this](rpp_interfaces::msg::MissionSafetyPreview::ConstSharedPtr m) {
      std::lock_guard<std::mutex> g(mu_);
      mission_safety_seen_ = true;
      mission_safety_received_ = std::chrono::steady_clock::now();
      mission_stop_requested_ = m->stop_requested;
      mission_rtk_fixed_ = m->rtk_fixed;
      mission_float_over_5s_ = m->float_over_five_seconds;
      mission_rtk_pause_ = m->rtk_pause_requested;
      mission_correction_age_verified_ = m->correction_age_verified;
    });
  timer_=create_wall_timer(50ms,[this]{publish();});
 }
 private:
 void rebuild_semantics_locked(){
   semantic_valid_=false; semantic_marking_count_=0;
   if(!geometry_ || !types_seen_ || !marks_seen_) return;
   try {
     auto tagged=dyx::rpp::attach_semantics(last_path_points_,types_,marks_);
     auto semantic=std::make_unique<dyx::rpp::PathGeometryIndex>(
       dyx::rpp::PathGeometryIndex::build(tagged));
     leg_tracker_.reset(); geometry_=std::move(semantic);
     semantic_valid_=true;
     semantic_marking_count_=marks_.size();
     semantic_error_.clear();
   }catch(const std::exception& e){
     // Never retain possibly stale semantic identities after a bad update.
     semantic_error_=e.what();
     leg_tracker_.reset(); geometry_=std::make_unique<dyx::rpp::PathGeometryIndex>(
       dyx::rpp::PathGeometryIndex::build(last_path_points_));
   }
 }
 void publish(){
  std::lock_guard<std::mutex> g(mu_);
  const auto now=std::chrono::steady_clock::now();
  const bool fresh=seen_ && now-position_time_<=250ms;
  const bool path_ok=bool(geometry_) && !geometry_->segments.empty();
  rpp_interfaces::msg::RppPreview preview{};
  preview.header.stamp = get_clock()->now();
  preview.header.frame_id = "enu_preview_not_authorized";
  preview.path_revision = path_revision_;
  preview.active_marking_index = active_index_;
  preview.pose_fresh = fresh && pose_valid_ && !position_fault_;
  preview.semantic_valid = semantic_valid_;
  preview.motion_authorized = false; // Immutable pre-commissioning contract.
  preview.reason = "HOLD_NO_VALIDATED_GUIDANCE";
  const bool safety_fresh = mission_safety_seen_ &&
      now - mission_safety_received_ <= 250ms;
  const bool safety_hold = !safety_fresh || mission_stop_requested_ ||
      !mission_rtk_fixed_ || mission_rtk_pause_ ||
      !mission_correction_age_verified_;
  // Diagnostics only: safety evidence never authorizes motion.
  // Keep guidance_valid about geometry only; separate safety information follows.
  preview.reason = safety_hold ? "MISSION_OR_RTK_SAFETY_HOLD" :
      "NON_ACTUATING_RPP_GEOMETRY_ONLY";
  std::ostringstream s;
  s.setf(std::ios::fixed);s.precision(5);
  s<<"CXX17_RPP_PATH_OBSERVE;px4_position_fresh="<<(fresh?"true":"false")
   <<";pose_valid="<<(pose_valid_?"true":"false")
   <<";position_reset_fault="<<(position_fault_?"true":"false")
   <<";path_available="<<(path_ok?"true":"false")
   <<";path_revision="<<path_revision_
   <<";path_frame="<<path_frame_
   <<";path_error="<<path_error_
   <<";semantic_valid="<<(semantic_valid_?"true":"false")
   <<";semantic_marking_count="<<semantic_marking_count_
   <<";semantic_error="<<semantic_error_
   <<";active_marking_index="<<active_index_;
  if(fresh && pose_valid_ && !position_fault_ && path_ok){
    if(semantic_valid_ && active_index_>=0) {
      try {
        const auto leg=dyx::rpp::inspect_active_leg(
            *geometry_,marks_,active_index_,{east_,north_},
            get_parameter("lookahead_m").as_double());
        s<<";active_leg_valid=true;active_goal="<<leg.goal_identity
         <<";active_goal_radial_m="<<leg.radial_m
         <<";active_leg_cross_track_m="<<leg.cross_track_m
         <<";active_leg_remaining_m="<<leg.remaining_m
         <<";active_leg_lookahead_east_m="<<leg.lookahead.x
         <<";active_leg_lookahead_north_m="<<leg.lookahead.y;
      } catch(const std::exception& e) {
        s<<";active_leg_valid=false;active_leg_fault="<<e.what();
      }
    } else s<<";active_leg_valid=false;active_leg_fault=NO_MISSION_SELECTED_SEMANTIC_LEG";
    try{
      // A8 strictly limits RPP preview to the mission-selected semantic span.
      // No fallback to a global nearest path point: crossing legs must not
      // silently replace the active marking destination.
      if(semantic_valid_ && active_index_>=1) {
        dyx::rpp::ActiveLegControlConfig cfg;
        cfg.cruise_mps=get_parameter("preview_cruise_mps").as_double();
        cfg.lookahead_m=get_parameter("lookahead_m").as_double();
        cfg.accel_m=get_parameter("accel_distance_m").as_double();
        cfg.decel_m=get_parameter("decel_distance_m").as_double();
        cfg.master_to_pivot_m=get_parameter("rear_master_to_pivot_m").as_double();
        const auto plan=dyx::rpp::plan_active_leg_preview(
          *geometry_,marks_,active_index_,{east_,north_},heading_enu_,speed_mps_,cfg);
        // Stateful, bounded, monotonically non-decreasing active-leg progress.
        // A different active goal or geometry revision requires a new tracker.
        // This remains read-only; NO terminal certificate is issued here.
        const auto tracked=leg_tracker_.step(*geometry_, marks_,active_index_,
          path_revision_,{east_,north_},
          get_parameter("lookahead_m").as_double(),
          get_parameter("max_path_jump_m").as_double());
        s<<";tracked_progress_m="<<tracked.progress_m
         <<";tracked_remaining_m="<<tracked.remaining_m
         <<";tracked_cross_track_m="<<tracked.cross_track_m
         <<";tracked_lookahead_east_m="<<tracked.lookahead.x
         <<";tracked_lookahead_north_m="<<tracked.lookahead.y
         <<";tracked_clamped="<<(tracked.monotonic_clamped?"true":"false");
        preview.guidance_valid = true;
        preview.preview_forward_mps = plan.forward_preview_mps;
        preview.preview_yaw_rate_enu_radps = plan.yaw_rate_preview_radps;
        preview.heading_enu_rad = heading_enu_;
        preview.remaining_m = plan.remaining_m;
        // Preserve safety HOLD even when geometric guidance is valid.
        if (!safety_hold) preview.reason = "NON_ACTUATING_RPP_GEOMETRY_ONLY";
        s<<";execution_preview_valid=true;execution_phase="<<plan.phase
         <<";execution_goal="<<plan.goal_identity
         <<";execution_cross_track_m="<<plan.cross_track_m
         <<";execution_radial_m="<<plan.radial_m
         <<";execution_remaining_m="<<plan.remaining_m
         <<";execution_heading_error_rad="<<plan.heading_error_rad
         <<";execution_speed_preview_mps="<<plan.forward_preview_mps
         <<";execution_yaw_preview_radps="<<plan.yaw_rate_preview_radps
         <<";execution_pivot_required="<<(plan.reverse_pivot_required?"true":"false")
         <<";execution_reverse_distance_preview_m="<<plan.reverse_distance_preview_m
         <<";execution_stationary_at_mark="<<(plan.stationary_at_mark?"true":"false")
         <<";execution_lookahead_east_m="<<plan.lookahead_east_m
         <<";execution_lookahead_north_m="<<plan.lookahead_north_m;
      } else {
        s<<";execution_preview_valid=false;execution_phase=HOLD_NO_TRUSTED_ACTIVE_LEG";
      }
    }catch(const std::exception& e){s<<";geometry_fault="<<e.what();}
  }
  if(fresh && pose_valid_ && goal_valid_)s<<";goal_radial_m="<<std::hypot(goal_east_-east_,goal_north_-north_);
  s<<";mission_safety_fresh="<<(safety_fresh?"true":"false")
   <<";mission_stop_requested="<<(mission_stop_requested_?"true":"false")
   <<";rtk_fixed_diagnostic="<<(mission_rtk_fixed_?"true":"false")
   <<";rtk_float_over_5s="<<(mission_float_over_5s_?"true":"false")
   <<";rtk_pause_intent="<<(mission_rtk_pause_?"true":"false")
   <<";correction_age_verified="<<(mission_correction_age_verified_?"true":"false")
   <<";mission_authorized=false;motion_authorized=false;physical_command_emitted=false";
  // A valid preview never confers START, motion, spray, or PX4 authority.
  // Represent advisory guidance in the existing typed RPP command schema.
  // This separate topic is NEVER consumed by the PX4 motor bridge.
  rpp_interfaces::msg::RppCommand shadow_command{};
  shadow_command.header = preview.header;
  shadow_command.header.frame_id = "enu_shadow_not_authorized";
  shadow_command.yaw_valid = false; // Hard stop if guidance/safety unavailable.
  const bool preview_safe_to_calculate =
      preview.guidance_valid && preview.pose_fresh && preview.semantic_valid &&
      !safety_hold && std::isfinite(preview.preview_forward_mps) &&
      std::isfinite(preview.preview_yaw_rate_enu_radps) &&
      std::isfinite(preview.heading_enu_rad) &&
      preview.preview_forward_mps >= 0.0 && preview.preview_forward_mps <= 1.0 &&
      std::abs(preview.preview_yaw_rate_enu_radps) <= 0.18;
  if (preview_safe_to_calculate) {
    shadow_command.velocity_east_mps = preview.preview_forward_mps *
        std::cos(preview.heading_enu_rad);
    shadow_command.velocity_north_mps = preview.preview_forward_mps *
        std::sin(preview.heading_enu_rad);
    shadow_command.yaw_enu_rad = preview.heading_enu_rad;
    shadow_command.yaw_rate_enu_radps = preview.preview_yaw_rate_enu_radps;
    shadow_command.yaw_valid = true; // Numeric preview validity only; not authority.
  }
  shadow_command_pub_->publish(shadow_command);
  shadow_pub_->publish(preview);
  std_msgs::msg::String m;m.data=s.str();out_->publish(m);detail_->publish(m);
 }
 bool mission_safety_seen_{false};
 bool mission_stop_requested_{true}, mission_rtk_fixed_{false};
 bool mission_float_over_5s_{false}, mission_rtk_pause_{true};
 bool mission_correction_age_verified_{false};
 std::chrono::steady_clock::time_point mission_safety_received_{};
 std::mutex mu_;
 bool seen_{false},pose_valid_{false},position_fault_{false},goal_valid_{false};
 std::uint8_t xy_reset_{0},heading_reset_{0};std::uint64_t last_sample_{0};
 double east_{0},north_{0},heading_enu_{0},goal_east_{0},goal_north_{0},speed_mps_{0};
 std::uint64_t path_revision_{0};std::string path_frame_{},path_error_{};
 std::chrono::steady_clock::time_point position_time_{},path_time_{};
 std::int32_t active_index_{-1};
 bool types_seen_{false}, marks_seen_{false}, semantic_valid_{false};
 std::size_t semantic_marking_count_{0};
 std::string semantic_error_{};
 std::vector<std::uint8_t> types_{};
 std::vector<std::int32_t> marks_{};
 std::vector<dyx::rpp::RawPathPoint> last_path_points_;
 std::unique_ptr<dyx::rpp::PathGeometryIndex> geometry_;
 dyx::rpp::MonotonicLegTracking leg_tracker_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr out_,detail_;
 rclcpp::Publisher<rpp_interfaces::msg::RppPreview>::SharedPtr shadow_pub_;
 rclcpp::Publisher<rpp_interfaces::msg::RppCommand>::SharedPtr shadow_command_pub_;
 rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_;
 rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_;
 rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr types_sub_;
 rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr marks_sub_;
 rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr active_sub_;
 rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_;
 rclcpp::Subscription<rpp_interfaces::msg::MissionSafetyPreview>::SharedPtr mission_safety_sub_;
 rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<RppControllerNode>());rclcpp::shutdown();return 0;}
