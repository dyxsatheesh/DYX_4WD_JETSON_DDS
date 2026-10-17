#include "dyx_rpp_cpp17_core/spray_transaction.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <stdexcept>
using namespace std::chrono_literals;
class SprayControllerNode final:public rclcpp::Node{
public: SprayControllerNode():Node("spray_controller_node"){
 if(!declare_parameter<bool>("observe_only",true))throw std::invalid_argument("Physical spray adapter unavailable; observe_only required");
 pub_=create_publisher<std_msgs::msg::String>("/dyx/spray_controller/observation",5);
 timer_=create_wall_timer(500ms,[this]{dyx::rpp::SprayInput in{};in.now_sec=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();in.request_spray=false;in.controller_enabled=false;auto o=transaction_.step(in);std_msgs::msg::String m;m.data=std::string("CXX17_SPRAY_HOLD;state=")+dyx::rpp::spray_state_name(o.state)+";spray_output_enabled=false";pub_->publish(m);});}
private:dyx::rpp::NativeSprayTransaction transaction_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;rclcpp::TimerBase::SharedPtr timer_;
};int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<SprayControllerNode>());}catch(const std::exception&e){RCLCPP_ERROR(rclcpp::get_logger("spray_controller_node"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
