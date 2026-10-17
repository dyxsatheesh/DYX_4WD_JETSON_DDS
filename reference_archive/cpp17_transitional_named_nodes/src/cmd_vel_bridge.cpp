// C++17 cmd_vel_bridge: fail-closed mission-status watchdog only.
// No VehicleCommand, TrajectorySetpoint, OffboardControlMode or actuator publishers.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <string>
using namespace std::chrono_literals;
class CmdVelBridge final : public rclcpp::Node {
public:
  CmdVelBridge() : Node("cmd_vel_bridge") {
    mission_sub_ = create_subscription<std_msgs::msg::String>(
      "/mission_manager/status", rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        last_mission_sec_ = monotonic_sec();
        hold_observed_ = m->data.find("emergency_stop=true") != std::string::npos &&
                         m->data.find("hold_zero=true") != std::string::npos;
      });
    status_ = create_publisher<std_msgs::msg::String>("/cmd_vel_bridge/migration_status", 10);
    timer_ = create_wall_timer(50ms, [this] {
      const double now = monotonic_sec();
      const bool fresh = last_mission_sec_ >= 0.0 && now >= last_mission_sec_ &&
                         now - last_mission_sec_ <= 0.25;
      std_msgs::msg::String m;
      m.data = std::string("CXX17_DDS_BRIDGE;mission_watchdog_fresh=") +
               (fresh ? "true" : "false") +
               ";mission_hold_observed=" + ((fresh && hold_observed_) ? "true" : "false") +
               ";dds_setpoint_bridge_ready=false;motor_commands_enabled=false;" +
               "physical_stop_confirmed=false;physical_command_emitted=false";
      status_->publish(m);
    });
    RCLCPP_WARN(get_logger(), "DDS bridge mission watchdog active; PX4 output DISABLED");
  }
private:
  static double monotonic_sec() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
  }
  double last_mission_sec_{-1.0};
  bool hold_observed_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<CmdVelBridge>());
  rclcpp::shutdown(); return 0;
}
