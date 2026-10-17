#include "dyx_rpp_cpp17_core/spray_transaction.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <stdexcept>
using namespace std::chrono_literals;
class SprayControllerNode final : public rclcpp::Node {
public:
  SprayControllerNode():Node("dyx_spray_controller") {
    if(!declare_parameter<bool>("observe_only",true))
      throw std::invalid_argument("Physical spray adapter not implemented");
    publisher_=create_publisher<std_msgs::msg::String>("/dyx/spray_controller/observation",rclcpp::QoS(5));
    timer_=create_wall_timer(1s,[this]{
      dyx::rpp::SprayInput in;
      in.now_sec=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
      in.request_spray=false;
      in.controller_enabled=false;
      const auto out=transaction_.step(in);
      std_msgs::msg::String msg;
      msg.data=std::string("OBSERVE_ONLY;state=")+dyx::rpp::spray_state_name(out.state)+";PHYSICAL_COMMAND_PERMITTED=false";
      publisher_->publish(msg);
    });
  }
private:
  dyx::rpp::NativeSprayTransaction transaction_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<SprayControllerNode>());}
  catch(const std::exception &e){RCLCPP_ERROR(rclcpp::get_logger("spray_controller"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}