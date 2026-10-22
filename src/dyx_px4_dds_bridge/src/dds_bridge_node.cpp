#include "dyx_px4_dds_bridge/geometry.hpp"
#include "dyx_px4_dds_bridge/body_speed_projection.hpp"
#include "dyx_px4_dds_bridge/safety_freshness.hpp"
#include "dyx_px4_dds_bridge/bridge_commissioning_guard.hpp"
#include "dyx_px4_dds_bridge/native_rover_dds_contract.hpp"
#include "dyx_px4_dds_bridge/rover_setpoint_builder.hpp"
#include "dyx_px4_dds_bridge/native_writer_arbitration.hpp"

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/rover_speed_setpoint.hpp>
#include <px4_msgs/msg/rover_rate_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rpp_interfaces/msg/rpp_command.hpp>
#include <rpp_interfaces/msg/mission_safety_preview.hpp>
#include <rpp_interfaces/msg/mission_safety_preview.hpp>
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
        "/dyx/mission/owned_rpp_command", input_qos,
        [this](const rpp_interfaces::msg::RppCommand::SharedPtr msg) {
          latest_rpp_ = dyx::convert_rpp(
              msg->velocity_north_mps, msg->velocity_east_mps,
              msg->yaw_enu_rad, msg->yaw_rate_enu_radps,
              msg->yaw_valid,
              maximum_speed_mps_, maximum_yaw_rate_radps_);
          const bool estimator_fresh =
              heading_valid_ &&
              fresh(heading_seen_, heading_received_, 0.25);

          latest_body_ = dyx::bridge::project_body_speed(
              msg->velocity_north_mps,
              msg->velocity_east_mps,
              measured_heading_enu_rad_,
              msg->yaw_valid && estimator_fresh,
              maximum_speed_mps_);
          latest_wire_ = dyx::bridge::rover_speed_rate_wire(
              latest_body_.forward_mps,
              msg->yaw_rate_enu_radps,
              latest_rpp_.valid && latest_body_.valid,
              maximum_speed_mps_,
              maximum_yaw_rate_radps_);

          rpp_received_ = std::chrono::steady_clock::now();
          have_rpp_ = latest_wire_.valid;
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
        "/dyx/mission/heartbeat", input_qos,
        [this](const std_msgs::msg::UInt64::SharedPtr msg) {
          if (msg->data == 0 ||
              (have_heartbeat_ &&
               msg->data <= last_heartbeat_sequence_)) {
            have_heartbeat_ = false;
            heartbeat_fault_latched_ = true;
            return;
          }

          if (heartbeat_fault_latched_) return;

          last_heartbeat_sequence_ = msg->data;
          have_heartbeat_ = true;
          heartbeat_received_ = Steady::now();
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

    heading_sub_ =
        create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1",
            rclcpp::SensorDataQoS(),
            [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr p) {
              if (heading_seen_ &&
                  (p->timestamp_sample <= last_heading_sample_us_ ||
                   p->xy_reset_counter != initial_xy_reset_ ||
                   p->heading_reset_counter != initial_heading_reset_)) {
                heading_fault_latched_ = true;
              }

              if (!heading_seen_) {
                initial_xy_reset_ = p->xy_reset_counter;
                initial_heading_reset_ = p->heading_reset_counter;
              }

              heading_seen_ = true;
              last_heading_sample_us_ = p->timestamp_sample;
              heading_received_ = Steady::now();

              heading_valid_ =
                  !heading_fault_latched_ &&
                  p->timestamp_sample != 0 &&
                  p->xy_valid &&
                  p->v_xy_valid &&
                  p->heading_good_for_control &&
                  !p->dead_reckoning &&
                  std::isfinite(p->heading);

              if (heading_valid_) {
                measured_heading_enu_rad_ =
                    1.5707963267948966 - p->heading;
              } else {
                have_rpp_ = false;
                latest_body_ = {};
                latest_wire_ = {};
              }
            });

    mission_safety_sub_ =
        create_subscription<rpp_interfaces::msg::MissionSafetyPreview>(
            "/dyx/mission/safety_preview",
            rclcpp::QoS(5),
            [this](
                rpp_interfaces::msg::MissionSafetyPreview::ConstSharedPtr m) {
              mission_safety_seen_ = true;
              mission_safety_received_ = Steady::now();

              // Diagnostic state only. ROS topic publication alone
              // cannot grant mission or actuator authority.
              mission_preview_stop_ = m->stop_requested;
              mission_preview_authorized_ = m->motion_authorized;
              mission_preview_rtk_fixed_ = m->rtk_fixed;
              mission_preview_correction_age_verified_ =
                  m->correction_age_verified;
            });

    mission_safety_sub_ =
        create_subscription<rpp_interfaces::msg::MissionSafetyPreview>(
            "/dyx/mission/safety_preview",
            rclcpp::QoS(5),
            [this](
                rpp_interfaces::msg::MissionSafetyPreview::ConstSharedPtr m) {
              mission_safety_seen_ = true;
              mission_safety_received_ = Steady::now();

              // Diagnostic state only. ROS topic publication alone
              // cannot grant mission or actuator authority.
              mission_preview_stop_ = m->stop_requested;
              mission_preview_authorized_ = m->motion_authorized;
              mission_preview_rtk_fixed_ = m->rtk_fixed;
              mission_preview_correction_age_verified_ =
                  m->correction_age_verified;
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
    if (!rclcpp::ok(get_node_base_interface()->get_context()))
      return;

    // PRE-COMMISSIONING: diagnostics only. No PX4 command is published.
    // The constructor rejects enable_motion=true, even with ROS overrides.
    const bool command_fresh =
        have_rpp_ && latest_rpp_.valid &&
        fresh(true, rpp_received_, command_timeout_s_) &&
        heading_valid_ &&
        fresh(heading_seen_, heading_received_, 0.25);
    const auto timestamp_ns = get_clock()->now().nanoseconds();
    const auto prepared = dyx::bridge::prepare_rover_setpoints(
        latest_wire_,
        timestamp_ns > 0
            ? static_cast<std::uint64_t>(timestamp_ns / 1000)
            : 0,
        command_fresh);

    if (++tick_count_ % 50 != 0) return;

    try {
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
        ";rpp_command_valid=" + std::string(have_rpp_ ? "true" : "false") +
        ";rpp_command_fresh=" + std::string(command_fresh ? "true" : "false") +
        ";converted_north_mps=" + std::to_string(command_fresh ? latest_rpp_.north_mps : 0.0f) +
        ";converted_east_mps=" + std::to_string(command_fresh ? latest_rpp_.east_mps : 0.0f) +
        ";converted_yaw_rate_ned_radps=" + std::to_string(
            command_fresh ? latest_rpp_.yaw_rate_ned_radps : 0.0f) +
        ";body_speed_valid=" +
            std::string(latest_body_.valid ? "true" : "false") +
        ";body_forward_mps=" +
            std::to_string(command_fresh ? latest_body_.forward_mps : 0.0f) +
        ";measured_heading_valid=" +
            std::string(
                heading_valid_ &&
                fresh(heading_seen_, heading_received_, 0.25)
                ? "true" : "false") +
        ";heading_fault_latched=" +
            std::string(heading_fault_latched_ ? "true" : "false") +
        ";mission_safety_preview_fresh=" +
            std::string(
                fresh(mission_safety_seen_,
                      mission_safety_received_, 0.30)
                ? "true" : "false") +
        ";mission_preview_stop_requested=" +
            std::string(mission_preview_stop_ ? "true" : "false") +
        ";mission_preview_motion_authorized=" +
            std::string(mission_preview_authorized_ ? "true" : "false") +
        ";mission_preview_rtk_fixed=" +
            std::string(mission_preview_rtk_fixed_ ? "true" : "false") +
        ";mission_preview_correction_age_verified=" +
            std::string(
                mission_preview_correction_age_verified_
                ? "true" : "false") +
        ";trusted_mission_actuation_authorized=false" +
        ";mission_safety_preview_fresh=" +
            std::string(
                fresh(mission_safety_seen_,
                      mission_safety_received_, 0.30)
                ? "true" : "false") +
        ";mission_preview_stop_requested=" +
            std::string(mission_preview_stop_ ? "true" : "false") +
        ";mission_preview_motion_authorized=" +
            std::string(mission_preview_authorized_ ? "true" : "false") +
        ";mission_preview_rtk_fixed=" +
            std::string(mission_preview_rtk_fixed_ ? "true" : "false") +
        ";mission_preview_correction_age_verified=" +
            std::string(
                mission_preview_correction_age_verified_
                ? "true" : "false") +
        ";trusted_mission_actuation_authorized=false" +
        ";native_mission_heartbeat_fresh=" +
            std::string(
                !heartbeat_fault_latched_ &&
                fresh(have_heartbeat_,
                      heartbeat_received_, heartbeat_timeout_s_)
                ? "true" : "false") +
        ";native_mission_heartbeat_fault=" +
            std::string(heartbeat_fault_latched_ ? "true" : "false") +
        ";native_mission_heartbeat_authorizes_motion=false" +
        ";native_wire_valid=" +
            std::string(latest_wire_.valid ? "true" : "false") +
        ";native_wire_fresh=" +
            std::string(command_fresh ? "true" : "false") +
        ";native_speed_body_x_mps=" +
            std::to_string(command_fresh ? latest_wire_.speed_body_x : 0.0f) +
        ";native_yaw_rate_ned_radps=" +
            std::to_string(command_fresh ? latest_wire_.yaw_rate_setpoint : 0.0f) +
        ";px4_setpoints_prepared=" +
            std::string(prepared.valid ? "true" : "false") +
        ";px4_offboard_velocity_flag=" +
            std::string(
                prepared.valid && prepared.mode.velocity
                ? "true" : "false") +
        ";px4_setpoint_publish_authorized=false" +
        ";native_wire_publish_authorized=false" +
        ";motor_commands_enabled=false;physical_command_emitted=false";
    status_pub_->publish(status);
    } catch (const std::exception&) {
      // Graph operations can race with Ctrl+C context shutdown.
      // Suppress exceptions ONLY once ROS shutdown has begun.
      if (!rclcpp::ok(get_node_base_interface()->get_context()))
        return;
      throw;
    }
  }

  dyx::bridge::BridgeAudit audit_{};

  rclcpp::Subscription<
      rpp_interfaces::msg::MissionSafetyPreview>::SharedPtr
      mission_safety_sub_;

  bool mission_safety_seen_{false};
  bool mission_preview_stop_{true};
  bool mission_preview_authorized_{false};
  bool mission_preview_rtk_fixed_{false};
  bool mission_preview_correction_age_verified_{false};

  std::chrono::steady_clock::time_point
      mission_safety_received_{};


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_status_sub_;
  bool enable_motion_{false};
  bool emergency_stop_{true};
  bool mission_enabled_{false};
  bool armed_{false};
  bool in_offboard_{false};
  bool have_heartbeat_{false};
  bool heartbeat_fault_latched_{false};
  std::uint64_t last_heartbeat_sequence_{0};
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
  dyx::bridge::BodySpeedProjection latest_body_{};
  dyx::bridge::RoverSpeedRateWire latest_wire_{};

  rclcpp::Subscription<
      px4_msgs::msg::VehicleLocalPosition>::SharedPtr heading_sub_;

  bool heading_seen_{false};
  bool heading_valid_{false};
  bool heading_fault_latched_{false};

  std::uint64_t last_heading_sample_us_{0};
  std::uint8_t initial_xy_reset_{0};
  std::uint8_t initial_heading_reset_{0};

  double measured_heading_enu_rad_{0.0};

  Steady::time_point heading_received_{};
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
