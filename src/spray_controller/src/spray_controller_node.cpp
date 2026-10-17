#include "dyx_rpp_cpp17_core/spray_transaction.hpp"
#include "spray_observation_guard.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>
using namespace std::chrono_literals;
class SprayControllerNode final:public rclcpp::Node {
public:
  SprayControllerNode():Node("spray_controller_node") {
    if(!declare_parameter<bool>("observe_only",true))
      throw std::invalid_argument("SPRAY_OUTPUT_UNCOMMISSIONED_OBSERVE_ONLY_REQUIRED");
    settings_.off_pwm_us=static_cast<int>(declare_parameter<int>("off_pwm_us",1200));
    settings_.on_pwm_us=static_cast<int>(declare_parameter<int>("on_pwm_us",1400));
    settings_.duration_sec=declare_parameter<double>("duration_sec",3.0);
    dyx::spray::validate(settings_);
    pub_=create_publisher<std_msgs::msg::String>("/dyx/spray_controller/observation",5);
    mission_sub_=create_subscription<std_msgs::msg::String>("/mission_manager/status",10,
      [this](std_msgs::msg::String::ConstSharedPtr m){
        last_mission_=std::chrono::steady_clock::now();
        observed_mission_stop_=dyx::spray::status_is_stop(m->data);
        seen_mission_=true;
      });
    timer_=create_wall_timer(50ms,[this]{
      const auto now=std::chrono::steady_clock::now();
      const double sec=std::chrono::duration<double>(now.time_since_epoch()).count();
      auto state=transaction_.step(dyx::spray::blocked_input(sec));
      const bool fresh=seen_mission_ && now-last_mission_<=250ms;
      std::ostringstream ss;
      ss<<"CXX17_SPRAY_OBSERVE;state="<<dyx::rpp::spray_state_name(state.state)
        <<";mission_status_fresh="<<(fresh?"true":"false")
        <<";mission_hold_observed="<<((fresh&&observed_mission_stop_)?"true":"false")
        <<";off_pwm_us="<<settings_.off_pwm_us
        <<";on_pwm_us="<<settings_.on_pwm_us
        <<";duration_sec="<<settings_.duration_sec
        <<";off_position_verified=false"
        <<";mission_spray_request_authorized=false"
        <<";spray_output_enabled=false;physical_command_emitted=false";
      std_msgs::msg::String m;m.data=ss.str();pub_->publish(m);
    });
    RCLCPP_WARN(get_logger(),"AUX5 transaction observer active; no actuator publisher, OFF state not verified");
  }
private:
  dyx::spray::Observation settings_{};
  dyx::rpp::NativeSprayTransaction transaction_{};
  std::chrono::steady_clock::time_point last_mission_{};
  bool seen_mission_{false},observed_mission_stop_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<SprayControllerNode>());}catch(const std::exception&e){RCLCPP_ERROR(rclcpp::get_logger("spray_controller_node"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
