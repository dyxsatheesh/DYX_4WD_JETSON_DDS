#include "dyx_rpp_cpp17_core/native_control_owner.hpp"

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
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
        << ";motion_enabled=false"
        << ";spray_enabled=false"
        << ";physical_command_emitted=false";

    status.data = out.str();
    status_pub_->publish(status);
  }

  NativeControlOwner owner_{};
  bool telemetry_fault_{false};
  std::uint64_t control_sequence_{0};

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
