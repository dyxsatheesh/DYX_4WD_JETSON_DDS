// H6B: 50 Hz C++17 PX4 DDS telemetry + dynamic mission-goal PREVIEW only.
// Safety invariant: NO PX4 input topics, motor outputs, or RppCommand publisher.
// A String preview on a namespaced diagnostic topic is NOT a motion command.
#include "dyx_rpp_controller/admission.hpp"
#include "dyx_rpp_cpp17_core/shadow_geometry.hpp"
#include "dyx_rpp_cpp17_core/forward_yaw_rate.hpp"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
using namespace std::chrono_literals;
class DyxRppNativePreview final : public rclcpp::Node {
 public:
  DyxRppNativePreview() : rclcpp::Node("dyx_rpp_controller") {
    preview_enabled_ = declare_parameter<bool>("preview_enabled", false);
    speed_mps_ = declare_parameter<double>("preview_speed_mps", 0.6);
    if (!std::isfinite(speed_mps_) || speed_mps_ < 0.0 || speed_mps_ > 1.0)
      throw std::invalid_argument("preview_speed_mps out of range");
    // PX4 status/position is best effort; this node has no PX4 publications.
    const auto sensor_qos = rclcpp::SensorDataQoS();
    pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", sensor_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg) {
        if (position_seen_ &&
            (msg->xy_reset_counter != initial_xy_reset_ ||
             msg->heading_reset_counter != initial_heading_reset_))
          reset_seen_ = true;
        if (!position_seen_) {
          initial_xy_reset_ = msg->xy_reset_counter;
          initial_heading_reset_ = msg->heading_reset_counter;
        }
        position_seen_ = true;
        position_ = *msg;
        position_received_ = now_s();
      });
    rate_sub_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
      "/fmu/out/vehicle_angular_velocity", sensor_qos,
      [this](px4_msgs::msg::VehicleAngularVelocity::ConstSharedPtr msg) {
        rate_seen_ = true;
        rate_ = *msg;
        rate_received_ = now_s();
      });
    // Preview goals use PX4 LOCAL NED (PointStamped x=N, y=E, z=D).
    // Deliberately separate from production mission authority.
    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/dyx/mission/preview_goal_ned", rclcpp::QoS(5).reliable(),
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        goal_seen_ = true;
        goal_received_ = now_s();
        goal_invalid_ = goal_invalid_ ||
          msg->header.frame_id != "px4_local_ned" ||
          !std::isfinite(msg->point.x) || !std::isfinite(msg->point.y) ||
          !std::isfinite(msg->point.z) ||
          std::abs(msg->point.x) > 100000.0 ||
          std::abs(msg->point.y) > 100000.0;
        goal_north_m_ = msg->point.x;
        goal_east_m_ = msg->point.y;
      });
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/dyx/rpp_controller/preview_status", rclcpp::QoS(5));
    tick_ = create_wall_timer(20ms, [this] { step(); });
    RCLCPP_WARN(get_logger(),
      "H6B PREVIEW ONLY; preview_enabled=%s; no /fmu/in or /rpp/command publishers",
      preview_enabled_ ? "true" : "false");
  }
 private:
  static double now_s() {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void status(const std::string& payload) {
    std_msgs::msg::String out;
    out.data = payload + "; PHYSICAL_COMMAND_PERMITTED=false";
    status_pub_->publish(out);
  }
  void step() {
    const double now = now_s();
    dyx::preview::Evidence e;
    e.preview_enabled = preview_enabled_;
    e.now_s = now;
    e.position_seen = position_seen_;
    e.rate_seen = rate_seen_;
    e.goal_seen = goal_seen_;
    e.reset_detected = reset_seen_;
    e.invalid_goal = goal_invalid_;
    e.estimator_valid = position_seen_ && position_.xy_valid &&
      position_.v_xy_valid && position_.heading_good_for_control;
    e.telemetry_finite = position_seen_ && rate_seen_ &&
      std::isfinite(position_.x) && std::isfinite(position_.y) &&
      std::isfinite(position_.vx) && std::isfinite(position_.vy) &&
      std::isfinite(position_.heading) && std::isfinite(rate_.xyz[2]);
    e.position_age_s = position_seen_ ? now - position_received_ : 1e6;
    e.rate_age_s = rate_seen_ ? now - rate_received_ : 1e6;
    e.goal_age_s = goal_seen_ ? now - goal_received_ : 1e6;
    e.position_sample_us = position_.timestamp_sample;
    e.rate_sample_us = rate_.timestamp_sample;
    e.goal_north_m = goal_north_m_;
    e.goal_east_m = goal_east_m_;
    const auto allowed = admission_.inspect(e);
    if (!allowed.preview_permitted) {
      // Nothing is emitted to command transports, including a zero setpoint.
      status(std::string("INHIBITED:") + allowed.reason);
      return;
    }
    dyx::rpp::ShadowTelemetry telemetry;
    telemetry.have_position = position_seen_;
    telemetry.have_rate = rate_seen_;
    telemetry.xy_valid = position_.xy_valid;
    telemetry.v_xy_valid = position_.v_xy_valid;
    telemetry.heading_good_for_control = position_.heading_good_for_control;
    telemetry.position_age_sec = e.position_age_s;
    telemetry.rate_age_sec = e.rate_age_s;
    telemetry.position_timestamp_sample_us = position_.timestamp_sample;
    telemetry.rate_timestamp_sample_us = rate_.timestamp_sample;
    telemetry.position_north_m = position_.x;
    telemetry.position_east_m = position_.y;
    telemetry.velocity_north_mps = position_.vx;
    telemetry.velocity_east_mps = position_.vy;
    telemetry.heading_ned_rad = position_.heading;
    telemetry.body_z_rate_frd_radps = rate_.xyz[2];
    const auto observed = dyx::rpp::inspect_shadow_telemetry(telemetry);
    if (!observed.valid) {
      status(std::string("INHIBITED:") + observed.reason);
      return;
    }
    const auto de = goal_east_m_ - observed.east_m;
    const auto dn = goal_north_m_ - observed.north_m;
    const double radial_m = std::hypot(de, dn);
    const double target_yaw_enu = radial_m > 1e-8 ?
      std::atan2(dn, de) : observed.yaw_enu_rad;
    dyx::rpp::ForwardYawInput command;
    command.now_sec = now;
    command.dt_sec = previous_tick_ >= 0.0 ? now - previous_tick_ : 0.0;
    previous_tick_ = now;
    command.rover_yaw_enu_rad = observed.yaw_enu_rad;
    command.target_yaw_enu_rad = target_yaw_enu;
    command.measured_yaw_rate_enu_radps = observed.approximate_yaw_rate_enu_radps;
    command.requested_forward_speed_mps = speed_mps_;
    command.telemetry_fresh = true;
    command.motion_enabled = true; // Internal PREVIEW calculation, not permission.
    command.pivot_permission = false; // No externally signed pivot authorization.
    command.stop_requested = radial_m <= 0.02;
    const auto result = planner_.step(command);
    std::ostringstream line;
    line << std::fixed << std::setprecision(5)
         << "PREVIEW_ONLY;phase=" << dyx::rpp::phase_name(result.phase)
         << ";reason=" << result.reason << ";radial_m=" << radial_m
         << ";preview_north_mps=" << result.velocity_north_mps
         << ";preview_east_mps=" << result.velocity_east_mps
         << ";preview_yaw_rate_enu_radps=" << result.yaw_rate_enu_radps
         << ";PREVIEW_NEVER_SENT_TO_PX4";
    status(line.str());
  }
  bool preview_enabled_{false};
  double speed_mps_{0.6}, previous_tick_{-1.0};
  bool position_seen_{false}, rate_seen_{false}, goal_seen_{false};
  bool reset_seen_{false}, goal_invalid_{false};
  std::uint8_t initial_xy_reset_{0}, initial_heading_reset_{0};
  double position_received_{0.0}, rate_received_{0.0}, goal_received_{0.0};
  double goal_north_m_{0.0}, goal_east_m_{0.0};
  px4_msgs::msg::VehicleLocalPosition position_{};
  px4_msgs::msg::VehicleAngularVelocity rate_{};
  dyx::preview::Admission admission_{};
  dyx::rpp::ForwardYawRatePlanner planner_{};
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr rate_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr tick_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<DyxRppNativePreview>());}
  catch (const std::exception& e) {
    std::cerr << "H6B startup failed: " << e.what() << '\n';
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
