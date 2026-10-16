#include "dyx_rpp_cpp17_core/native_control_owner.hpp"
#include "dyx_rpp_cpp17_core/rear_marking_reference.hpp"

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <memory>
#include <sstream>
#include <string>

using namespace std::chrono_literals;
using dyx::rpp::NativeControlOwner;
using dyx::rpp::DdsTelemetryDecision;

class DyxMissionDdsRuntime final : public rclcpp::Node {
 public:
  DyxMissionDdsRuntime()
      : Node("dyx_mission_dds_runtime") {
    // Configurable lever arm, disabled until actually measured.
    // Configuration alone does NOT certify a marking reference.
    lever_.forward_m =
        declare_parameter<double>("imu_to_rear_master_forward_m", 0.0);
    lever_.right_m =
        declare_parameter<double>("imu_to_rear_master_right_m", 0.0);

    if (!std::isfinite(lever_.forward_m) ||
        !std::isfinite(lever_.right_m) ||
        std::hypot(lever_.forward_m, lever_.right_m) > 2.0)
      throw std::invalid_argument("Invalid rear master lever arm");

    const auto qos = rclcpp::SensorDataQoS();

    battery_sub_ =
        create_subscription<px4_msgs::msg::BatteryStatus>(
            "/fmu/out/battery_status", qos,
            [this](px4_msgs::msg::BatteryStatus::ConstSharedPtr m) {
              dyx::rpp::DdsBatteryFrame f{};
              f.timestamp_us = m->timestamp;
              f.received_sec = now_sec();
              f.voltage_v = m->voltage_v;
              f.connected = m->connected;
              f.powering_off = m->is_powering_off;
              f.cell_count = m->cell_count;
              f.warning = m->warning;
              inspect(owner_.observe_battery(f));
            });

    gps_sub_ =
        create_subscription<px4_msgs::msg::SensorGps>(
            "/fmu/out/vehicle_gps_position", qos,
            [this](px4_msgs::msg::SensorGps::ConstSharedPtr m) {
              dyx::rpp::DdsGpsFrame f{};
              f.gps_timestamp_sample_us = m->timestamp_sample;
              f.received_sec = now_sec();
              f.device_id = m->device_id;
              f.latitude_deg = m->latitude_deg;
              f.longitude_deg = m->longitude_deg;
              f.fix_type = m->fix_type;
              f.rtcm_crc_failed = m->rtcm_crc_failed;
              f.system_error = m->system_error;
              inspect(owner_.observe_gps(f));
            });

    vehicle_sub_ =
        create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr m) {
              dyx::rpp::DdsVehicleFrame f{};
              f.timestamp_us = m->timestamp;
              f.received_sec = now_sec();
              f.arming_state = m->arming_state;
              f.nav_state = m->nav_state;
              f.vehicle_type = m->vehicle_type;
              f.failsafe = m->failsafe;
              f.failure_detector_status =
                  m->failure_detector_status;
              inspect(owner_.observe_vehicle(f));
            });

    position_sub_ =
        create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            [this](
                px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr m) {
              dyx::rpp::DdsEstimatorFrame f{};
              f.timestamp_sample_us = m->timestamp_sample;
              f.received_sec = now_sec();
              f.xy_valid = m->xy_valid;
              f.v_xy_valid = m->v_xy_valid;
              f.heading_good_for_control =
                  m->heading_good_for_control;
              f.dead_reckoning = m->dead_reckoning;
              f.x_m = m->x;
              f.y_m = m->y;
              f.heading_rad = m->heading;
              inspect(owner_.observe_estimator(f));

              if (position_seen_ &&
                  (m->timestamp_sample <= last_position_sample_us_ ||
                   m->xy_reset_counter != initial_xy_reset_ ||
                   m->heading_reset_counter != initial_heading_reset_)) {
                position_fault_ = true;
                owner_.force_stop();
              }

              if (!position_seen_) {
                initial_xy_reset_ = m->xy_reset_counter;
                initial_heading_reset_ = m->heading_reset_counter;
              }

              if (m->timestamp_sample == 0)
                position_fault_ = true;

              position_seen_ = true;
              last_position_sample_us_ = m->timestamp_sample;
              last_position_received_sec_ = f.received_sec;
              last_position_ = f;

              // PX4 local velocity is North/East in the NED frame.
              // A speed magnitude alone is not proof of forward motion.
              last_velocity_north_mps_ = m->vx;
              last_velocity_east_mps_ = m->vy;
              last_velocity_valid_ =
                  m->v_xy_valid &&
                  std::isfinite(m->vx) &&
                  std::isfinite(m->vy);

              if (!last_velocity_valid_) {
                position_fault_ = true;
                owner_.force_stop();
              }
            });

    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/dyx/native/mission_dds_runtime/status",
        rclcpp::QoS(5));

    timer_ = create_wall_timer(50ms, [this]() { tick(); });

    RCLCPP_WARN(
        get_logger(),
        "Native DDS mission gateway active; "
        "motion/spray command publishing DISABLED; "
        "RTCM/obstacle/operator adapters pending");
  }

 private:
  static double now_sec() {
    using Clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(
        Clock::now().time_since_epoch()).count();
  }

  void inspect(DdsTelemetryDecision decision) {
    if (decision != DdsTelemetryDecision::accepted &&
        decision != DdsTelemetryDecision::duplicate) {
      owner_.force_stop();
      telemetry_fault_ = true;
    }
  }

  void tick() {
    const double now = now_sec();

    dyx::rpp::MissionSequenceInput evidence{};
    evidence.monotonic_time_sec = now;
    evidence.emergency_stop = true;

    // No trusted operator or physical E-stop adapter is connected.
    // Never permit motion or mission execution in this build.
    dyx::rpp::NativeMissionRppInput guidance{};
    guidance.sequence = ++control_sequence_;
    guidance.guidance.now_sec = now;
    guidance.guidance.dt_sec = 0.05;

    // No trusted mission upload, control lease, GNSS calibration,
    // obstacle clearance or physical E-stop adapter yet.
    // These values must not be inferred from DDS status alone.
    guidance.guidance.motion_enabled = false;
    guidance.guidance.telemetry_fresh = false;
    guidance.guidance.stop_requested = true;

    const auto result = owner_.tick(
        now,
        false,
        false,
        true,
        evidence,
        guidance);

    std_msgs::msg::String status{};
    std::ostringstream out;

    const bool position_fresh =
        position_seen_ && !position_fault_ &&
        std::isfinite(last_position_received_sec_) &&
        now >= last_position_received_sec_ &&
        now - last_position_received_sec_ <= 0.25 &&
        last_position_.xy_valid &&
        last_position_.heading_good_for_control;

    bool reference_available = false;
    dyx::rpp::RearMarkingPosition rear{};

    if (position_fresh) {
      try {
        rear = dyx::rpp::compute_rear_marking_reference(
            last_position_.x_m,
            last_position_.y_m,
            last_position_.heading_rad,
            lever_);
        reference_available = true;
      } catch (const std::invalid_argument&) {
        position_fault_ = true;
        owner_.force_stop();
      }
    }

    dyx::rpp::MissionLegPreviewInput leg_input{};

    leg_input.now_sec = now;
    leg_input.dt_sec = 0.05;

    leg_input.position_sample_time_sec =
        last_position_received_sec_;

    // Path coordinates are local ENU: X=East, Y=North.
    leg_input.marking_reference_enu = {
        rear.east_m,
        rear.north_m
    };

    // PX4 NED heading: zero=North, clockwise positive.
    // RPP ENU yaw: zero=East, counterclockwise positive.
    if (position_fresh) {
      leg_input.rover_yaw_enu_rad =
          dyx::rpp::wrap_angle(
              dyx::rpp::pi / 2.0 -
              last_position_.heading_rad);
    }

    if (last_velocity_valid_) {
      leg_input.position_derived_speed_mps =
          std::hypot(
              last_velocity_north_mps_,
              last_velocity_east_mps_);

      // Diagnostic magnitude only. No positive actuator
      // decision is allowed from this estimate.
      leg_input.measured_forward_speed_mps =
          leg_input.position_derived_speed_mps;
    }

    leg_input.telemetry_fresh =
        position_fresh &&
        reference_available &&
        last_velocity_valid_;

    // Deliberate fail-closed placeholders until real trusted
    // production adapters are integrated.
    leg_input.marking_reference_calibrated = false;
    leg_input.rtk_fixed = false;
    leg_input.mission_enabled = false;
    leg_input.battery_ok = false;
    leg_input.obstacle_clear = false;
    leg_input.emergency_stop = true;
    leg_input.stationary_pivot_permitted = false;

    const auto leg = owner_.preview_active_leg(leg_input);

    out << "NATIVE_DDS_MISSION_RUNTIME"
        << ";stage="
        << dyx::rpp::mission_stage_name(
               owner_.mission().stage())
        << ";px4_vehicle_fresh="
        << result.mission.telemetry.px4_vehicle_fresh
        << ";gps_fresh="
        << result.mission.telemetry.gps_sample_fresh
        << ";battery_fresh="
        << result.mission.telemetry.battery_sample_fresh
        << ";telemetry_fault="
        << (telemetry_fault_ ||
            result.mission.telemetry_fault_latched)
        << ";rpp_motion_permitted="
        << result.motion.permitted
        << ";rear_reference_available=" << reference_available
        << ";rear_reference_north_m=" << rear.north_m
        << ";rear_reference_east_m=" << rear.east_m
        << ";rear_reference_calibrated=false"
        << ";position_fault=" << position_fault_
        << ";rpp_preview_valid=" << leg.valid
        << ";rpp_preview_hold_zero=" << leg.hold_zero
        << ";rpp_preview_reason=" << leg.reason
        << ";rpp_preview_forward_mps="
        << leg.forward_preview_mps
        << ";rpp_preview_yaw_rate_radps="
        << leg.yaw_rate_enu_preview_radps
        << ";motion_enabled=false"
        << ";spray_enabled=false"
        << ";physical_command_emitted=false";

    status.data = out.str();
    status_pub_->publish(status);
  }

  NativeControlOwner owner_{};
  bool telemetry_fault_{false};
  std::uint64_t control_sequence_{0};

  dyx::rpp::RearMarkingLeverArm lever_{};
  dyx::rpp::DdsEstimatorFrame last_position_{};

  double last_velocity_north_mps_{0.0};
  double last_velocity_east_mps_{0.0};
  bool last_velocity_valid_{false};

  bool position_seen_{false};
  bool position_fault_{false};
  std::uint64_t last_position_sample_us_{0};
  double last_position_received_sec_{0.0};

  std::uint8_t initial_xy_reset_{0};
  std::uint8_t initial_heading_reset_{0};

  rclcpp::Subscription<
      px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<
      px4_msgs::msg::SensorGps>::SharedPtr gps_sub_;
  rclcpp::Subscription<
      px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_sub_;
  rclcpp::Subscription<
      px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;

  rclcpp::Publisher<
      std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
        std::make_shared<DyxMissionDdsRuntime>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(
        rclcpp::get_logger("dyx_mission_dds_runtime"),
        "Startup failure: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
