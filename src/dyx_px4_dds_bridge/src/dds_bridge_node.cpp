#include "dyx_px4_dds_bridge/geometry.hpp"
#include "dyx_px4_dds_bridge/safety_freshness.hpp"
#include "dyx_px4_dds_bridge/bridge_commissioning_guard.hpp"
#include "dyx_px4_dds_bridge/native_rover_dds_contract.hpp"
#include "dyx_px4_dds_bridge/native_writer_arbitration.hpp"

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/rover_speed_setpoint.hpp>
#include <px4_msgs/msg/rover_rate_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rpp_interfaces/msg/rpp_command.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <iostream>
#include <stdexcept>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class DyxPx4DdsBridge final : public rclcpp::Node {
 public:
  DyxPx4DdsBridge() : Node("dyx_px4_dds_bridge") {
    // Deliberately DISABLED by default and not added to rover.launch.py.
    enable_motion_ = declare_parameter<bool>("enable_motion", false);
    // Hardware commissioning not completed. Never let a ROS parameter
    // enable the experimental velocity/trajectory PX4 writer.
    if (enable_motion_) throw std::runtime_error(
        "B3_LOCKOUT: enable_motion=true prohibited before hardware certification");
    maximum_speed_mps_ = declare_parameter<double>("maximum_speed_mps", 1.0);
    maximum_yaw_rate_radps_ =
        declare_parameter<double>("maximum_yaw_rate_radps", 0.18);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.15);
    heartbeat_timeout_s_ =
        declare_parameter<double>("backend_heartbeat_timeout_s", 1.5);
    safety_input_timeout_s_ =
        declare_parameter<double>("safety_input_timeout_s", 0.30);

    if (!(maximum_speed_mps_ > 0.0 && maximum_speed_mps_ <= 1.0 &&
          maximum_yaw_rate_radps_ > 0.0 && maximum_yaw_rate_radps_ <= 0.18 &&
          command_timeout_s_ > 0.0 && command_timeout_s_ <= 0.5 &&
          heartbeat_timeout_s_ > 0.0 &&
          std::isfinite(safety_input_timeout_s_) &&
          safety_input_timeout_s_ > 0.0 &&
          safety_input_timeout_s_ <= 0.5)) {
      throw std::runtime_error("Invalid bridge safety parameters");
    }

    // PX4 DDS writers need a compatible publisher; subscribers to PX4 use
    // BEST_EFFORT (PX4 publishes sensor/status streams with best-effort QoS).
    const auto px4_pub_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    px4_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", px4_pub_qos);
    px4_rover_speed_pub_ = create_publisher<px4_msgs::msg::RoverSpeedSetpoint>(
        "/fmu/in/rover_speed_setpoint", px4_pub_qos);
    px4_rover_rate_pub_ = create_publisher<px4_msgs::msg::RoverRateSetpoint>(
        "/fmu/in/rover_rate_setpoint", px4_pub_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/dyx_px4_dds_bridge/status", rclcpp::QoS(10));

    rpp_sub_ = create_subscription<rpp_interfaces::msg::RppCommand>(
        "/rpp/command", input_qos,
        [this](const rpp_interfaces::msg::RppCommand::SharedPtr msg) {
          latest_rpp_ = dyx::convert_rpp(
              msg->velocity_north_mps, msg->velocity_east_mps,
              msg->yaw_rate_enu_radps, msg->yaw_valid,
              maximum_speed_mps_, maximum_yaw_rate_radps_);
          rpp_received_ = std::chrono::steady_clock::now();
          have_rpp_ = latest_rpp_.valid;
        });

    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop", input_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          emergency_stop_ = msg->data;
          have_estop_ = true;
          estop_received_ = Steady::now();
        });
    mission_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/mission_enable", input_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          mission_enabled_ = msg->data;
          have_mission_ = true;
          mission_received_ = Steady::now();
        });
    heartbeat_sub_ = create_subscription<std_msgs::msg::UInt64>(
        "/rover_backend/heartbeat", input_qos,
        [this](const std_msgs::msg::UInt64::SharedPtr) {
          have_heartbeat_ = true;
          heartbeat_received_ = std::chrono::steady_clock::now();
        });
    vehicle_status_sub_ =
        create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
              armed_ = (msg->arming_state ==
                        px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
              in_offboard_ = (msg->nav_state ==
                             px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
              have_px4_status_ = true;
              px4_status_received_ = std::chrono::steady_clock::now();
            });

    mission_status_sub_ = create_subscription<std_msgs::msg::String>(
        "/mission_manager/status", rclcpp::QoS(10),
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          audit_.observe(m->data, Steady::now());
        });
    // This is diagnostic only. No trust is granted from a textual status.
    tick_ = create_wall_timer(20ms, [this]() { publish_setpoint(); });
    RCLCPP_WARN(get_logger(),
                "DDS bridge installed but NOT production enabled; "
                "enable_motion=%s. Do not launch old MAVROS command bridge "
                "and this DDS bridge together.",
                enable_motion_ ? "true" : "false");
  }

 private:
  using Steady = std::chrono::steady_clock;
  static bool fresh(bool seen, Steady::time_point last, double max_age) {
    return seen &&
           std::chrono::duration<double>(Steady::now() - last).count() <= max_age;
  }

  void publish_setpoint() {
    // PRE-COMMISSIONING: diagnostics only. No PX4 command is published.
    // The constructor rejects enable_motion=true, even with ROS overrides.
    have_rpp_ = false;
    if (++tick_count_ % 50 != 0) return;

    const auto mode = count_publishers("/fmu/in/offboard_control_mode");
    const auto speed = count_publishers("/fmu/in/rover_speed_setpoint");
    const auto rate = count_publishers("/fmu/in/rover_rate_setpoint");
    dyx::bridge::WriterCounts counts{};
    counts.offboard_mode = mode > 0 ? mode - 1 : 0;
    counts.rover_speed = speed > 0 ? speed - 1 : 0;
    counts.rover_rate = rate > 0 ? rate - 1 : 0;
    counts.legacy_trajectory = count_publishers("/fmu/in/trajectory_setpoint");
    counts.rover_attitude = count_publishers("/fmu/in/rover_attitude_setpoint");
    const auto graph = dyx::bridge::inspect_writer_graph(counts, true, false);
    std_msgs::msg::String status;
    status.data = std::string("INHIBITED:NATIVE_ROVER_DDS_NON_ACTUATING") +
        ";native_rover_publishers_created=true" +
        ";setpoint_published=false;offboard_heartbeat_published=false" +
        ";writer_graph_reason=" + graph.reason +
        ";other_writers=" + std::to_string(graph.observed_writers) +
        ";publisher_exclusivity_proven=false" +
        ";motor_commands_enabled=false;physical_command_emitted=false";
    status_pub_->publish(status);
  }

  dyx::bridge::BridgeAudit audit_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_status_sub_;
  bool enable_motion_{false};
  bool emergency_stop_{true};
  bool mission_enabled_{false};
  bool armed_{false};
  bool in_offboard_{false};
  bool have_heartbeat_{false};
  bool have_estop_{false};
  bool have_mission_{false};
  bool have_px4_status_{false};
  bool have_rpp_{false};
  double maximum_speed_mps_{1.0};
  double maximum_yaw_rate_radps_{0.18};
  double command_timeout_s_{0.15};
  double heartbeat_timeout_s_{1.5};
  double safety_input_timeout_s_{0.30};
  size_t tick_count_{0};
  dyx::NedCommand latest_rpp_{};
  Steady::time_point heartbeat_received_{};
  Steady::time_point estop_received_{};
  Steady::time_point mission_received_{};
  Steady::time_point px4_status_received_{};
  Steady::time_point rpp_received_{};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr px4_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::RoverSpeedSetpoint>::SharedPtr px4_rover_speed_pub_;
  rclcpp::Publisher<px4_msgs::msg::RoverRateSetpoint>::SharedPtr px4_rover_rate_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<rpp_interfaces::msg::RppCommand>::SharedPtr rpp_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::TimerBase::SharedPtr tick_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DyxPx4DdsBridge>());
  } catch (const std::exception &e) {
    std::cerr << "DDS bridge startup failure: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
