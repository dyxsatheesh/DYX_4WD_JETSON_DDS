#include "dyx_px4_dds_bridge/geometry.hpp"
#include "dyx_px4_dds_bridge/body_speed_projection.hpp"
#include "dyx_px4_dds_bridge/safety_freshness.hpp"
#include "dyx_px4_dds_bridge/bridge_commissioning_guard.hpp"
#include "dyx_px4_dds_bridge/wheel_lift_gate.hpp"
#include "dyx_px4_dds_bridge/bench_command_pipeline.hpp"
#include "dyx_px4_dds_bridge/bench_transport_interlock.hpp"
#include "dyx_px4_dds_bridge/commissioned_output_gate.hpp"
#include "dyx_px4_dds_bridge/native_rover_dds_contract.hpp"
#include "dyx_px4_dds_bridge/rover_setpoint_builder.hpp"
#include "dyx_px4_dds_bridge/rover_speed_heading_builder.hpp"
#include "dyx_px4_dds_bridge/native_writer_arbitration.hpp"

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/rover_speed_setpoint.hpp>
#include <px4_msgs/msg/rover_attitude_setpoint.hpp>
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
    px4_rover_attitude_pub_ = create_publisher<px4_msgs::msg::RoverAttitudeSetpoint>(
        "/fmu/in/rover_attitude_setpoint", px4_pub_qos);
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

          // Previous Python bridge: RPP supplies desired heading; PX4 closes
          // the heading and yaw-rate loops. Ignore Jetson-generated yaw-rate
          // at the motor transport layer.
          latest_heading_enu_rad_ = msg->yaw_enu_rad;
          latest_heading_valid_ = msg->yaw_valid && std::isfinite(msg->yaw_enu_rad);
          rpp_received_ = std::chrono::steady_clock::now();
          have_rpp_ = latest_wire_.valid && latest_heading_valid_;
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
    // Mission health and motion authorization are distinct.
    // This ROS preview is only a conservative rejection input;
    // it is not proof of operator authorization or hardware safety.
    const bool mission_heartbeat_fresh =
        !heartbeat_fault_latched_ &&
        fresh(have_heartbeat_, heartbeat_received_,
              heartbeat_timeout_s_);

    const bool mission_safety_fresh =
        fresh(mission_safety_seen_,
              mission_safety_received_,
              safety_input_timeout_s_);

    const bool mission_preview_allows_staging =
        mission_safety_fresh &&
        !mission_preview_stop_ &&
        mission_preview_authorized_ &&
        mission_preview_rtk_fixed_ &&
        mission_preview_correction_age_verified_;

    const bool rpp_input_fresh =
        have_rpp_ && latest_rpp_.valid &&
        fresh(true, rpp_received_, command_timeout_s_);

    const bool heading_fresh =
        heading_valid_ &&
        fresh(heading_seen_, heading_received_, 0.25);

    const bool command_fresh =
        rpp_input_fresh &&
        heading_fresh &&
        mission_heartbeat_fresh &&
        mission_preview_allows_staging;
    const auto timestamp_ns = get_clock()->now().nanoseconds();
    const auto prepared_heading = dyx::bridge::prepare_rover_speed_heading(
        latest_wire_.speed_body_x, latest_heading_enu_rad_,
        timestamp_ns > 0 ? static_cast<std::uint64_t>(timestamp_ns / 1000) : 0,
        command_fresh && latest_heading_valid_);

    // Output policy is evaluated at every 20 ms tick; diagnostics are throttled below.

    try {
    const auto mode = count_publishers("/fmu/in/offboard_control_mode");
    const auto speed = count_publishers("/fmu/in/rover_speed_setpoint");
    const auto rate = count_publishers("/fmu/in/rover_rate_setpoint");
    dyx::bridge::WriterCounts counts{};
    counts.offboard_mode = mode > 0 ? mode - 1 : 0;
    counts.rover_speed = speed > 0 ? speed - 1 : 0;
    counts.rover_rate = rate;
    counts.legacy_trajectory = count_publishers("/fmu/in/trajectory_setpoint");
    const auto attitude = count_publishers("/fmu/in/rover_attitude_setpoint");
    counts.rover_attitude = attitude > 0 ? attitude - 1 : 0;
    const auto graph = dyx::bridge::inspect_writer_graph(counts, true, false);
    // Real PX4 output calls are implemented, but fail closed. None of the
    // ROS Bool topics or operator-written parameters proves a *physical*
    // interlock or deadman. Those inputs must arrive via a separately
    // certified hardware adapter before this writer may operate.
    dyx::bridge::CommissionedEvidence output_evidence{};
    output_evidence.requested_enable = enable_motion_;
    output_evidence.px4_status_fresh = fresh(have_px4_status_, px4_status_received_, 0.25);
    output_evidence.px4_armed = armed_;
    output_evidence.px4_offboard = in_offboard_;
    output_evidence.mission_stop = mission_preview_stop_;
    output_evidence.mission_authorized = mission_preview_authorized_ && mission_safety_fresh;
    output_evidence.command_fresh = command_fresh;
    output_evidence.heartbeat_fresh = mission_heartbeat_fresh;
    output_evidence.estimator_valid = heading_fresh;
    output_evidence.writer_quiet = graph.graph_quiet;
    output_evidence.estop_fresh = fresh(have_estop_, estop_received_, 0.25);
    output_evidence.estop_released_verified = false;  // ROS is not physical proof.
    output_evidence.independent_hardware_interlock_ok = false;
    output_evidence.operator_deadman_verified = false;
    output_evidence.rc_takeover_verified = false;
    output_evidence.forward_mps = latest_wire_.speed_body_x;
    output_evidence.yaw_rate_radps = latest_wire_.yaw_rate_setpoint;
    const auto output_decision = dyx::bridge::evaluate_commissioned_output(output_evidence);
    if (output_decision.allow_publish && prepared_heading.valid && prepared_heading.publish_authorized) {
      // Only executed after hardware adapter + controller authorization
      // are independently implemented, reviewed and bench-tested.
      px4_mode_pub_->publish(prepared_heading.mode);
      px4_rover_speed_pub_->publish(prepared_heading.speed);
      px4_rover_attitude_pub_->publish(prepared_heading.attitude);
    }

    if (++tick_count_ % 50 != 0) return;

    // Native bench pipeline validates candidate conversion but NEVER sends PX4 outputs.
    dyx::bridge::BenchCommandInput bench_input{};
    bench_input.motion_request_seen = have_rpp_;
    bench_input.mission_preview_fresh = mission_safety_fresh;
    bench_input.mission_stop_requested = mission_preview_stop_;
    bench_input.mission_motion_authorized = mission_preview_authorized_;
    bench_input.heading_fresh = heading_fresh;
    bench_input.yaw_valid = latest_rpp_.valid;
    bench_input.command_fresh = rpp_input_fresh;
    bench_input.heartbeat_fresh = mission_heartbeat_fresh;
    bench_input.output_writer_quiet = graph.graph_quiet;
    bench_input.forward_mps = latest_wire_.speed_body_x;
    bench_input.yaw_rate_radps = latest_wire_.yaw_rate_setpoint;
    const auto bench_candidate = dyx::bridge::evaluate_bench_command(bench_input);
    // Live diagnostic integration deliberately never invokes reset_simulation:
    // incomplete physical safety evidence must remain latched after startup.
    const auto simulated_receipt = bench_transport_.evaluate(
        bench_candidate, mission_preview_stop_, emergency_stop_,
        !graph.graph_quiet, mission_heartbeat_fresh, heading_fresh);
    // Precommissioning diagnostics only. Do not infer mechanical safety from ROS topics.
    dyx::bridge::WheelLiftEvidence commissioning{};
    commissioning.px4_armed = armed_;
    commissioning.offboard_active = in_offboard_;
    commissioning.px4_status_fresh = fresh(have_px4_status_, px4_status_received_, 0.25);
    commissioning.command_fresh = command_fresh;
    commissioning.heartbeat_fresh = mission_heartbeat_fresh;
    commissioning.estop_signal_fresh = fresh(have_estop_, estop_received_, 0.25);
    commissioning.estop_asserted = emergency_stop_;
    commissioning.writer_graph_quiet = graph.graph_quiet;
    commissioning.speed_mps = latest_wire_.speed_body_x;
    commissioning.yaw_rate_radps = latest_wire_.yaw_rate_setpoint;
    // Physical confirmation / takeover / certification are NOT attested on this bench.
    const auto wheel_lift = dyx::bridge::inspect_wheel_lift(commissioning);
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
        ";rpp_input_fresh=" +
            std::string(rpp_input_fresh ? "true" : "false") +
        ";mission_heartbeat_fresh=" +
            std::string(mission_heartbeat_fresh ? "true" : "false") +
        ";mission_safety_gate_ready=" +
            std::string(mission_preview_allows_staging ? "true" : "false") +
        ";heading_fresh=" +
            std::string(heading_fresh ? "true" : "false") +
        ";native_wire_valid=" +
            std::string(latest_wire_.valid ? "true" : "false") +
        ";native_wire_fresh=" +
            std::string(command_fresh ? "true" : "false") +
        ";native_speed_body_x_mps=" +
            std::to_string(command_fresh ? latest_wire_.speed_body_x : 0.0f) +
        ";native_yaw_rate_ned_radps=" +
            std::to_string(command_fresh ? latest_wire_.yaw_rate_setpoint : 0.0f) +
        ";px4_setpoints_prepared=" +
            std::string(prepared_heading.valid ? "true" : "false") +
        ";px4_offboard_velocity_flag=" +
            std::string(
                prepared_heading.valid && prepared_heading.mode.velocity
                ? "true" : "false") +
        ";px4_setpoint_publish_authorized=false" +
        ";native_wire_publish_authorized=false;autonomous_control_contract=PX4_SPEED_HEADING_PID" +
        ";motor_commands_enabled=false;physical_command_emitted=false" +
        ";wheel_lift_gate_reason=" + wheel_lift.reason +
        ";wheel_lift_publish_authorized=false" +
        ";bench_candidate_valid=" +
            std::string(bench_candidate.candidate_valid ? "true" : "false") +
        ";bench_candidate_reason=" + bench_candidate.reason +
        ";bench_candidate_forward_mps=" + std::to_string(bench_candidate.staged_speed_mps) +
        ";bench_candidate_yaw_rate_radps=" + std::to_string(bench_candidate.staged_yaw_rate_radps) +
        ";bench_candidate_publish_authorized=false" +
        ";sim_transport_latched=" +
           std::string(simulated_receipt.latched_stop ? "true" : "false") +
        ";sim_transport_reason=" + simulated_receipt.reason +
        ";sim_transport_physical_authorized=false" +
        ";commissioned_output_policy_reason=" + output_decision.reason +
        ";commissioned_output_policy_authorized=false";
    status_pub_->publish(status);
    } catch (const std::exception&) {
      // Graph operations can race with Ctrl+C context shutdown.
      // Suppress exceptions ONLY once ROS shutdown has begun.
      if (!rclcpp::ok(get_node_base_interface()->get_context()))
        return;
      throw;
    }
  }

  dyx::bridge::BenchTransportInterlock bench_transport_{};
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
  double latest_heading_enu_rad_{0.0};
  bool latest_heading_valid_{false};

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
  rclcpp::Publisher<px4_msgs::msg::RoverAttitudeSetpoint>::SharedPtr px4_rover_attitude_pub_;
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
