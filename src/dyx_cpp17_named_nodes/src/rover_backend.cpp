// C++17 backend migration: read-only ROS mission status freshness observer.
// No HTTP listener is introduced: legacy authenticated gateway remains isolated.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <string>
using namespace std::chrono_literals;
class RoverBackend final : public rclcpp::Node {
public:
  RoverBackend() : Node("rover_backend") {
    mission_sub_ = create_subscription<std_msgs::msg::String>(
      "/mission_manager/status", rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        last_mission_sec_ = monotonic_sec();
        // Observation only. No client-supplied status is permission evidence.
        mission_held_ = m->data.find("emergency_stop=true") != std::string::npos &&
                        m->data.find("hold_zero=true") != std::string::npos &&
                        m->data.find("physical_command_emitted=false") != std::string::npos;
      });
    status_ = create_publisher<std_msgs::msg::String>("/rover_backend/migration_status", 10);
    timer_ = create_wall_timer(100ms, [this] {
      const double now = monotonic_sec();
      const bool fresh = last_mission_sec_ >= 0.0 && now >= last_mission_sec_ &&
                         now - last_mission_sec_ <= 0.5;
      std_msgs::msg::String m;
      m.data = std::string("CXX17_BACKEND;mission_status_fresh=") + (fresh ? "true" : "false") +
               ";mission_hold_observed=" + ((fresh && mission_held_) ? "true" : "false") +
               ";backend_api_ready=false;authentication_ready=false;" +
               "mission_commands_enabled=false;physical_command_emitted=false";
      status_->publish(m);
    });
    RCLCPP_WARN(get_logger(), "Backend observes mission status; authenticated HTTP transfer PENDING");
  }
private:
  static double monotonic_sec() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
  }
  double last_mission_sec_{-1.0};
  bool mission_held_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<RoverBackend>());
  rclcpp::shutdown(); return 0;
}
