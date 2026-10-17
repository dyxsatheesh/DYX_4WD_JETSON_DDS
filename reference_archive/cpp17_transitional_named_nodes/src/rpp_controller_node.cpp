#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <sstream>
using namespace std::chrono_literals;
class RppControllerNode final: public rclcpp::Node{
public: RppControllerNode():Node("rpp_controller_node"){
  pub_=create_publisher<std_msgs::msg::String>("/dyx/rpp_controller/preview_status",5);
  pos_=create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",rclcpp::SensorDataQoS(),[this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr p){if(seen_&&(p->xy_reset_counter!=xy_reset_||p->heading_reset_counter!=heading_reset_))fault_=true;if(!seen_){xy_reset_=p->xy_reset_counter;heading_reset_=p->heading_reset_counter;}seen_=true;fresh_=std::chrono::steady_clock::now();valid_=p->xy_valid&&p->v_xy_valid&&std::isfinite(p->x)&&std::isfinite(p->y)&&std::isfinite(p->heading);x_=p->x;y_=p->y;});
  goal_=create_subscription<geometry_msgs::msg::PointStamped>("/dyx/mission/preview_goal_ned",5,[this](geometry_msgs::msg::PointStamped::ConstSharedPtr p){if(p->header.frame_id!="px4_local_ned"||!std::isfinite(p->point.x)||!std::isfinite(p->point.y)){fault_=true;return;}gx_=p->point.x;gy_=p->point.y;goal_seen_=true;});
  timer_=create_wall_timer(50ms,[this]{std_msgs::msg::String m;const bool fresh=seen_&&std::chrono::duration<double>(std::chrono::steady_clock::now()-fresh_).count()<=0.25;std::ostringstream s;s<<"CXX17_RPP_OBSERVE;px4_position_fresh="<<(fresh?"true":"false")<<";pose_valid="<<(valid_?"true":"false")<<";fault="<<(fault_?"true":"false")<<";goal_seen="<<(goal_seen_?"true":"false");if(fresh&&valid_&&goal_seen_&&!fault_)s<<";radial_preview_m="<<std::hypot(gx_-x_,gy_-y_);s<<";motion_authorized=false;physical_command_emitted=false";m.data=s.str();pub_->publish(m);});
 }
private:bool seen_{false},valid_{false},fault_{false},goal_seen_{false};std::uint8_t xy_reset_{0},heading_reset_{0};double x_{0},y_{0},gx_{0},gy_{0};std::chrono::steady_clock::time_point fresh_{};rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_;rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;rclcpp::TimerBase::SharedPtr timer_;
};int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<RppControllerNode>());rclcpp::shutdown();return 0;}
