// Phase 2F1: READ-ONLY PX4 DDS telemetry -> C++17 RPP shadow diagnostics.
// This node has no /fmu/in, /rpp/command, motor, arm, or mode publishers.
// Shadow preview is not a mission controller and is never an actuator source.
#include "dyx_rpp_cpp17_core/shadow_geometry.hpp"
#include "dyx_rpp_cpp17_core/forward_yaw_rate.hpp"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

class DyxRppCpp17Shadow final : public rclcpp::Node {
 public:
  DyxRppCpp17Shadow() : rclcpp::Node("dyx_rpp_cpp17_shadow") {
    // Deliberately no motion-command publisher, even when enabled.
    enabled_ = declare_parameter<bool>("shadow_enabled", false);
    preview_goal_east_m_ = declare_parameter<double>("preview_goal_east_m", 0.0);
    preview_goal_north_m_ = declare_parameter<double>("preview_goal_north_m", 0.0);
    preview_speed_mps_ = declare_parameter<double>("preview_speed_mps", 0.0);
    max_age_sec_ = declare_parameter<double>("max_telemetry_age_sec", 0.25);
    max_skew_sec_ = declare_parameter<double>("max_sample_skew_sec", 0.10);
    if (!std::isfinite(preview_goal_east_m_) || !std::isfinite(preview_goal_north_m_) ||
        !std::isfinite(preview_speed_mps_) || preview_speed_mps_ < 0.0 ||
        preview_speed_mps_ > 1.0 || !std::isfinite(max_age_sec_) ||
        max_age_sec_ <= 0.0 || max_age_sec_ > 0.5 ||
        !std::isfinite(max_skew_sec_) || max_skew_sec_ < 0.0 ||
        max_skew_sec_ > 0.25) {
      throw std::invalid_argument("invalid shadow-only configuration");
    }
    auto qos = rclcpp::SensorDataQoS();
    pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr m) {
        if (pos_seen_ &&
            (m->xy_reset_counter != initial_xy_reset_counter_ ||
             m->heading_reset_counter != initial_heading_reset_counter_)) {
          reset_detected_ = true; // latch; operator restart required
        }
        if (!pos_seen_) {
          initial_xy_reset_counter_ = m->xy_reset_counter;
          initial_heading_reset_counter_ = m->heading_reset_counter;
        }
        pos_seen_ = true;
        xy_valid_ = m->xy_valid;
        v_xy_valid_ = m->v_xy_valid;
        heading_valid_ = m->heading_good_for_control;
        px4_pos_sample_us_ = m->timestamp_sample;
        north_ = m->x;
        east_ = m->y;
        vx_ = m->vx;
        vy_ = m->vy;
        heading_ned_ = m->heading;
        pos_received_ = std::chrono::steady_clock::now();
      });
    rate_sub_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
      "/fmu/out/vehicle_angular_velocity", qos,
      [this](px4_msgs::msg::VehicleAngularVelocity::SharedPtr m) {
        rate_seen_ = true;
        px4_rate_sample_us_ = m->timestamp_sample;
        body_z_rate_ = m->xyz[2];
        rate_received_ = std::chrono::steady_clock::now();
      });
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/dyx/rpp_cpp17_shadow/status", rclcpp::QoS(10));
    tick_timer_ = create_wall_timer(100ms, [this] { tick(); });
    RCLCPP_WARN(get_logger(),
      "SHADOW ONLY: enabled=%s. Subscribes to PX4 telemetry and publishes "
      "diagnostic strings ONLY; no /fmu/in or /rpp/command output.",
      enabled_ ? "true" : "false");
  }

 private:
  using SteadyClock = std::chrono::steady_clock;
  static double age_sec(bool seen, SteadyClock::time_point since,
                        SteadyClock::time_point now) {
    return seen ? std::chrono::duration<double>(now-since).count() : 1e9;
  }
  void publish_status(const std::string &message) {
    std_msgs::msg::String s;
    s.data = message;
    status_pub_->publish(s);
  }
  void tick() {
    if (!enabled_) {
      publish_status("DISABLED; non-actuating C++17 shadow node; shadow_enabled=false");
      return;
    }
    if (reset_detected_) {
      inhibit("EKF_OR_HEADING_RESET_RESTART_REQUIRED");
      return;
    }
    const auto now = SteadyClock::now();
    dyx::rpp::ShadowTelemetry tele;
    tele.have_position = pos_seen_;
    tele.have_rate = rate_seen_;
    tele.xy_valid = xy_valid_;
    tele.v_xy_valid = v_xy_valid_;
    tele.heading_good_for_control = heading_valid_;
    tele.position_age_sec = age_sec(pos_seen_, pos_received_, now);
    tele.rate_age_sec = age_sec(rate_seen_, rate_received_, now);
    tele.position_timestamp_sample_us = px4_pos_sample_us_;
    tele.rate_timestamp_sample_us = px4_rate_sample_us_;
    tele.position_north_m = north_;
    tele.position_east_m = east_;
    tele.velocity_north_mps = vx_;
    tele.velocity_east_mps = vy_;
    tele.heading_ned_rad = heading_ned_;
    tele.body_z_rate_frd_radps = body_z_rate_;
    const auto state = dyx::rpp::inspect_shadow_telemetry(
        tele, max_age_sec_, max_skew_sec_);
    if (!state.valid) {
      inhibit(state.reason);
      return;
    }

    const double dx = preview_goal_east_m_ - state.east_m;
    const double dy = preview_goal_north_m_ - state.north_m;
    const double radial = std::hypot(dx, dy);
    const double target_yaw = radial > 1e-8
        ? std::atan2(dy, dx) : state.yaw_enu_rad;
    // Planner output is a preview; NO ROS actuator topic consumes it.
    dyx::rpp::ForwardYawInput in;
    in.now_sec = std::chrono::duration<double>(now.time_since_epoch()).count();
    in.dt_sec = 0.10;
    in.rover_yaw_enu_rad = state.yaw_enu_rad;
    in.target_yaw_enu_rad = target_yaw;
    in.measured_yaw_rate_enu_radps = state.approximate_yaw_rate_enu_radps;
    in.requested_forward_speed_mps = preview_speed_mps_;
    in.motion_enabled = true; // INTERNAL preview calculation only, never motor permission
    in.telemetry_fresh = true;
    in.pivot_permission = false; // external pivot certificate not integrated
    in.stop_requested = radial <= 0.020;
    const auto out = planner_.step(in);
    started_ = true;
    std::ostringstream line;
    line << std::fixed << std::setprecision(5)
         << "SHADOW_ONLY; phase=" << dyx::rpp::phase_name(out.phase)
         << "; reason=" << out.reason
         << "; east_m=" << state.east_m
         << "; north_m=" << state.north_m
         << "; yaw_enu_rad=" << state.yaw_enu_rad
         << "; measured_speed_mps=" << state.horizontal_speed_mps
         << "; preview_goal_error_m=" << radial
         << "; preview_forward_mps=" << out.forward_speed_mps
         << "; preview_yaw_rate_enu_radps=" << out.yaw_rate_enu_radps
         << "; PREVIEW_NOT_SENT_TO_PX4";
    publish_status(line.str());
  }
  void inhibit(const std::string &reason) {
    if (started_) {
      // Once begun, a telemetry/safety failure must latch planner SAFE_HOLD.
      dyx::rpp::ForwardYawInput failure;
      failure.now_sec = std::chrono::duration<double>(
          SteadyClock::now().time_since_epoch()).count();
      failure.telemetry_fresh = false;
      planner_.step(failure);
    }
    publish_status("INHIBITED; " + reason + "; PREVIEW_NOT_SENT_TO_PX4");
  }

  bool enabled_{false}, pos_seen_{false}, rate_seen_{false},
       xy_valid_{false}, v_xy_valid_{false}, heading_valid_{false},
       reset_detected_{false}, started_{false};
  std::uint8_t initial_xy_reset_counter_{0};
  std::uint8_t initial_heading_reset_counter_{0};
  std::uint64_t px4_pos_sample_us_{0}, px4_rate_sample_us_{0};
  double preview_goal_east_m_{0.0}, preview_goal_north_m_{0.0};
  double preview_speed_mps_{0.0}, max_age_sec_{0.25}, max_skew_sec_{0.10};
  double north_{0.0}, east_{0.0}, vx_{0.0}, vy_{0.0};
  double heading_ned_{0.0}, body_z_rate_{0.0};
  SteadyClock::time_point pos_received_{}, rate_received_{};
  dyx::rpp::ForwardYawRatePlanner planner_{};
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr rate_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DyxRppCpp17Shadow>());
  } catch (const std::exception &e) {
    std::cerr << "SHADOW STARTUP FAILED: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
