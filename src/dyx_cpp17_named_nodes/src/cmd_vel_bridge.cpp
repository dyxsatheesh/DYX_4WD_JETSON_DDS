// Phase 4 C++17 named boundary: zero hardware command publication.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
using namespace std::chrono_literals;
class CmdVelBridge final : public rclcpp::Node {
public:
  CmdVelBridge() : Node("cmd_vel_bridge") {
    status_ = create_publisher<std_msgs::msg::String>("/cmd_vel_bridge/migration_status", 10);
    timer_ = create_wall_timer(200ms, [this] {
      std_msgs::msg::String m;
      m.data = "CXX17_MIGRATION_HOLD;dds_setpoint_bridge_ready=false;"
               "motor_commands_enabled=false;physical_stop_confirmed=false;"
               "physical_command_emitted=false";
      status_->publish(m);
    });
    RCLCPP_WARN(get_logger(), "C++17 cmd_vel_bridge: no PX4 publisher, no actuator authority");
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<CmdVelBridge>());
  rclcpp::shutdown(); return 0;
}
