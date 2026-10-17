// C++17 mission_manager_node: sole owner of this node's non-actuating sequencer.
// No unauthenticated ROS command input, no PX4/spray publisher.
#include "dyx_rpp_cpp17_core/mission_sequence.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <string>
using namespace std::chrono_literals;
class MissionManagerNode final : public rclcpp::Node {
public:
  MissionManagerNode() : Node("mission_manager_node") {
    status_ = create_publisher<std_msgs::msg::String>("/mission_manager/status", 10);
    timer_ = create_wall_timer(50ms, [this] {
      dyx::rpp::MissionSequenceInput input{};
      input.monotonic_time_sec = monotonic_sec();
      input.emergency_stop = true;
      input.motion_health_authorized = false;
      const auto decision = sequencer_.step(input);
      std_msgs::msg::String m;
      m.data = "CXX17_MISSION;mission_stage=" +
        std::string(dyx::rpp::mission_stage_name(decision.stage)) +
        ";mission_enable=false;emergency_stop=true;" +
        "hold_zero=true;start_enabled=false;resume_enabled=false;" +
        "spray_request=false;physical_command_emitted=false";
      status_->publish(m);
    });
    RCLCPP_WARN(get_logger(), "C++17 mission sequencer active in safety HOLD; NO mission load/start or actuation");
  }
private:
  static double monotonic_sec() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
  }
  dyx::rpp::MissionSequencer sequencer_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<MissionManagerNode>());
  rclcpp::shutdown(); return 0;
}
