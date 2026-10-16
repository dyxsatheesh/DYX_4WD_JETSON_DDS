// DYX H6D: C++17 PX4 DDS battery/GNSS/vehicle/estimator OBSERVER ONLY.
// Does not produce command topics, safety authorizations, motor or spray output.
// RTCM correction age and obstacle evidence have NO trusted adapters yet.
#include "dyx_safety_cpp17_observer/read_only_contract.hpp"
#include "dyx_rpp_cpp17_core/dds_safety_telemetry.hpp"
#include "dyx_rpp_cpp17_core/safety_arbiter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
using namespace std::chrono_literals;

class DyxSafetyObserver final : public rclcpp::Node {
 public:
  DyxSafetyObserver():Node("dyx_h6d_safety_observer") {
    enabled_ = declare_parameter<bool>("observe_enabled", false);
    // Firmware-exported stream names must be verified before deployment.
    const auto gps_topic = declare_parameter<std::string>(
      "gps_topic", "/fmu/out/vehicle_gps_position");
    if (gps_topic != "/fmu/out/vehicle_gps_position" &&
        gps_topic != "/fmu/out/sensor_gps")
      throw std::invalid_argument("H6D unsupported PX4 GPS topic");
    const auto qos = rclcpp::SensorDataQoS();
    battery_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
      "/fmu/out/battery_status", qos,
      [this](px4_msgs::msg::BatteryStatus::ConstSharedPtr m) {
        dyx::rpp::DdsBatteryFrame f;
        f.timestamp_us=m->timestamp;
        f.received_sec=now_s();
        f.voltage_v=m->voltage_v;
        f.connected=m->connected;
        f.powering_off=m->is_powering_off;
        f.cell_count=m->cell_count;
        f.warning=m->warning;
        observe_result(telemetry_.observe_battery(f));
        battery_last_v_=f.voltage_v;
      });
    gps_sub_ = create_subscription<px4_msgs::msg::SensorGps>(
      gps_topic, qos, [this](px4_msgs::msg::SensorGps::ConstSharedPtr m) {
        dyx::rpp::DdsGpsFrame f;
        f.gps_timestamp_sample_us=m->timestamp_sample;
        f.received_sec=now_s();
        f.device_id=m->device_id;
        f.latitude_deg=m->latitude_deg;
        f.longitude_deg=m->longitude_deg;
        f.fix_type=m->fix_type;
        f.rtcm_crc_failed=m->rtcm_crc_failed;
        f.system_error=m->system_error;
        observe_result(telemetry_.observe_gps(f));
        gps_last_fix_=f.fix_type;
      });
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", qos,
      [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr m) {
        dyx::rpp::DdsVehicleFrame f;
        f.timestamp_us=m->timestamp;
        f.received_sec=now_s();
        f.arming_state=m->arming_state;
        f.nav_state=m->nav_state;
        f.vehicle_type=m->vehicle_type;
        f.failsafe=m->failsafe;
        f.failure_detector_status=m->failure_detector_status;
        observe_result(telemetry_.observe_vehicle(f));
      });
    estimator_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr m) {
        if (estimator_seen_ &&
            (m->xy_reset_counter!=initial_xy_reset_ ||
             m->heading_reset_counter!=initial_heading_reset_)) {
          estimator_reset_latched_=true;
        }
        if (!estimator_seen_) {
          initial_xy_reset_=m->xy_reset_counter;
          initial_heading_reset_=m->heading_reset_counter;
        }
        estimator_seen_=true;
        dyx::rpp::DdsEstimatorFrame f;
        f.timestamp_sample_us=m->timestamp_sample;
        f.received_sec=now_s();
        f.xy_valid=m->xy_valid;
        f.v_xy_valid=m->v_xy_valid;
        f.heading_good_for_control=m->heading_good_for_control;
        f.dead_reckoning=m->dead_reckoning;
        f.x_m=m->x;
        f.y_m=m->y;
        f.heading_rad=m->heading;
        observe_result(telemetry_.observe_estimator(f));
      });
    output_ = create_publisher<std_msgs::msg::String>(
      "/dyx/h6d/safety_observer/status", rclcpp::QoS(5));
    tick_timer_ = create_wall_timer(200ms, [this] { tick(); });
    RCLCPP_WARN(get_logger(),
      "H6D OBSERVATION ONLY; enabled=%s; RTCM/obstacle adapters UNCONNECTED; "
      "no motion or mission authorization ever emitted",
      enabled_ ? "true" : "false");
  }
 private:
  static double now_s() {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void observe_result(dyx::rpp::DdsTelemetryDecision d) {
    if (d!=dyx::rpp::DdsTelemetryDecision::accepted &&
        d!=dyx::rpp::DdsTelemetryDecision::duplicate)
      ingress_fault_latched_=true;
  }
  void tick() {
    std_msgs::msg::String out;
    if (!enabled_) {
      out.data="H6D_DISABLED;PHYSICAL_COMMAND_PERMITTED=false";
      output_->publish(out);
      return;
    }
    // Hypothetical health arbitration for DIAGNOSTICS only, not permission.
    // Correction and obstacle inputs remain absent, guaranteeing blocked health.
    const auto snap = telemetry_.snapshot(now_s(), true, true, false);
    auto safety = arbiter_.step(snap.safety);
    const bool fault = ingress_fault_latched_ || estimator_reset_latched_ ||
      snap.telemetry_integrity_fault;
    const auto o = dyx::h6d::observe_only(
      fault, safety.battery_critical_latched,
      safety.rtk_float_confirming, safety.battery_debounce_pending,
      safety.reason);
    std::ostringstream s;
    s << "H6D_OBSERVATION_ONLY;reason=" << o.reason
      << ";px4_model_hold=" << safety.reason
      << ";battery_v=" << battery_last_v_
      << ";gps_fix_type=" << gps_last_fix_
      << ";battery_fresh=" << snap.battery_sample_fresh
      << ";gps_fresh=" << snap.gps_sample_fresh
      << ";vehicle_fresh=" << snap.px4_vehicle_fresh
      << ";battery_lockout=" << o.battery_critical_latched
      << ";float_confirming=" << o.rtk_float_confirming
      << ";battery_debounce=" << o.battery_debounce_pending
      << ";estimator_reset_latched=" << estimator_reset_latched_
      << ";corrections_verified=false;obstacles_verified=false"
      << ";PHYSICAL_COMMAND_PERMITTED=false"
      << ";MISSION_TRANSITION_PERMITTED=false;SPRAY_PERMITTED=false";
    out.data=s.str();
    output_->publish(out);
  }
  bool enabled_{false};
  bool ingress_fault_latched_{false}, estimator_seen_{false};
  bool estimator_reset_latched_{false};
  std::uint8_t initial_xy_reset_{0}, initial_heading_reset_{0};
  double battery_last_v_{0.0};
  int gps_last_fix_{0};
  dyx::rpp::NativeDdsSafetyTelemetry telemetry_{};
  dyx::rpp::NativeSafetyArbiter arbiter_{};
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr gps_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr estimator_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr output_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<DyxSafetyObserver>()); }
  catch (const std::exception &e) {
    std::cerr << "H6D startup failure: " << e.what() << '\n';
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
