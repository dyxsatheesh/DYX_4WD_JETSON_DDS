// Independent, read-only PX4 DDS telemetry node.
// Migration boundary: this node does NOT yet replace legacy subscriptions in
// dyx_mission_dds_runtime_node. No command, arming, motor or spray publishers.
#include "dyx_mission_manager/native_telemetry_safety_component.hpp"
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
using namespace std::chrono_literals;
class DyxPx4TelemetryNode final : public rclcpp::Node {
 public:
  DyxPx4TelemetryNode() : rclcpp::Node("dyx_px4_telemetry") {
    const auto qos = rclcpp::SensorDataQoS();
    battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
      "/fmu/out/battery_status", qos,
      [this](px4_msgs::msg::BatteryStatus::ConstSharedPtr m) {
        const auto now = monotonic_sec();
        monitor_.observe_battery(now, m->timestamp, m->voltage_v,
          m->connected, m->is_powering_off, m->cell_count, m->warning);
      });
    gps_sub_ = create_subscription<px4_msgs::msg::SensorGps>(
      "/fmu/out/vehicle_gps_position", qos,
      [this](px4_msgs::msg::SensorGps::ConstSharedPtr m) {
        monitor_.observe_gps(monotonic_sec(), m->timestamp_sample,
          m->fix_type, m->rtcm_crc_failed, m->system_error);
      });
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/dyx/native/px4_telemetry/diagnostics", rclcpp::QoS(5));
    timer_ = create_wall_timer(100ms, [this] { publish_status(); });
    RCLCPP_WARN(get_logger(),
      "Read-only DDS telemetry node; RTCM receiver age UNVERIFIED; all actuation disabled");
  }
 private:
  static double monotonic_sec() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
  }
  void publish_status() {
    const double now = monotonic_sec();
    const auto& power = monitor_.power();
    const auto& rtk = monitor_.rtk();
    std::ostringstream out;
    out << "DYX_PX4_TELEMETRY"
        << ";battery_fresh=" << power.battery_fresh(now)
        << ";battery_critical_latched=" << monitor_.critical_battery_latched()
        << ";battery_healthy_diagnostic=" << power.battery_healthy(now)
        << ";gps_fresh=" << power.gps_fresh(now)
        << ";gps_fix_type=" << power.fix_type()
        << ";rtk_fixed=" << rtk.fixed(now)
        << ";rtk_float_duration_sec=" << rtk.float_duration_sec(now)
        << ";rtk_pause_intent=" << rtk.pause_requested(now)
        << ";rtk_sample_fault=" << rtk.fault()
        << ";rtcm_receiver_age_connected=false"
        << ";corrections_fresh=false"
        << ";ready_for_motion=false"
        << ";physical_command_emitted=false";
    std_msgs::msg::String msg;
    msg.data = out.str();
    status_pub_->publish(msg);
  }
  dyx::http_backend::NativeTelemetrySafetyComponent monitor_{};
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr gps_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<DyxPx4TelemetryNode>()); }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("dyx_px4_telemetry"), "Startup: %s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
