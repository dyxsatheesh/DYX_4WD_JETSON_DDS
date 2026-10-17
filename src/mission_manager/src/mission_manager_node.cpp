#include "dyx_rpp_cpp17_core/native_control_owner.hpp"
#include "dyx_mission_manager/native_http_server.hpp"
#include "dyx_mission_manager/native_command_mailbox.hpp"
#include "dyx_mission_manager/native_mission_command_executor.hpp"
#include "dyx_mission_manager/native_telemetry_safety_component.hpp"
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

class MissionManagerNode final : public rclcpp::Node {
 public:
  MissionManagerNode()
      : Node("mission_manager_node") {
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
              telemetry_safety_.observe_battery(f.received_sec, m->timestamp,
                   m->voltage_v, m->connected, m->is_powering_off,
                   m->cell_count, m->warning);
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
              telemetry_safety_.observe_gps(f.received_sec, m->timestamp_sample,
                   m->fix_type, m->rtcm_crc_failed, m->system_error);
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
              // Trusted disarm evidence comes from PX4 DDS, never JSON.
              last_vehicle_disarm_received_sec_ = f.received_sec;
              last_vehicle_disarmed_ =
                  m->timestamp != 0 &&
                  m->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED &&
                  !m->failsafe && m->failure_detector_status == 0;
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

    trajectory_status_sub_ = create_subscription<std_msgs::msg::String>(
        "/trajectory_generator/status", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          // Only diagnostic status; never interpreted as authorization.
          trajectory_seen_ = true;
          trajectory_status_ = m->data.substr(0, 160);
        });
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/dyx/native/mission_dds_runtime/status",
        rclcpp::QoS(5));
    legacy_compat_status_pub_ = create_publisher<std_msgs::msg::String>(
        "/mission_manager/status", rclcpp::QoS(10));

    // Target scheduling rate: 50 Hz. This does not guarantee
    // real-time scheduling or PX4 command delivery.
    // Start the native HTTP transport within this ROS 2 process.
    // Initially localhost-only, non-actuating, and separate from
    // Android's existing production endpoint.
    native_http_ =
        std::make_unique<dyx::http_backend::NativeHttpServer>(
            18766, native_status_, command_mailbox_, upload_ingress_, load_ingress_, command_decisions_);

    timer_ = create_wall_timer(20ms, [this]() { tick(); });

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
    // The 30-second critical-voltage debounce latches an internal mission STOP.
    // This is not a PX4 hardware stop acknowledgment.
    if (telemetry_safety_.critical_battery_latched()) owner_.force_stop();

    // Single ROS-owner command executor, extracted from the gateway.
    // STOP remains decision-side only; positive commands remain denied.
    const auto command_result = mission_command_executor_.drain(
        command_mailbox_, command_decisions_,
        [this]() { owner_.force_stop(); },
        upload_ingress_, load_ingress_,
        dyx::rpp::kBackendMissionStopPath);
    processed_commands_ += command_result.processed;

    // Exactly one bounded upload decision per tick. The owner_ is touched
    // only here on the ROS executor; HTTP never invokes the mission gateway.
    if (auto entry = upload_ingress_.pop_for_ros(
            command_mailbox_.stopped_latched())) {
      const auto preview = dyx::http_backend::NativeMissionPreview::validate(
          entry->payload);
      if (preview.http_status != 200 || command_mailbox_.stopped_latched()) {
        upload_ingress_.publish_result(entry->session_id, 422,
            "INVALID_OR_STOPPED_UPLOAD", "", "", 0);
      } else {
        dyx::rpp::BackendMissionUploadRequest request{};
        request.method = "POST";
        request.path = dyx::rpp::kBackendMissionUploadPath;
        request.principal.authenticated = true;
        request.principal.can_read_mission = true;
        request.principal.owns_control_lease = true;
        request.principal.operator_session_id = entry->session_id;
        request.mission_run_id =
            entry->payload.at("mission_run_id").get<std::string>();
        request.sequence = entry->sequence;
        request.host_received_sec = now;
        request.mission_mode =
            entry->payload.value("mission_mode", std::string("automatic")) == "manual"
            ? dyx::rpp::MissionMode::manual : dyx::rpp::MissionMode::automatic;
        request.extension_enabled = entry->payload.value("extension_enabled", false);
        request.extension_distance_m = entry->payload.value("extension_distance_m", 4.0);
        for (const auto& p : entry->payload.at("marking_points")) {
          request.marking_points.push_back({
              p.at("x_m").get<double>(), p.at("y_m").get<double>()});
        }
        // A stale/rejected PX4 DDS VehicleStatus cannot attest disarm.
        request.trusted_px4_disarmed = !telemetry_fault_ &&
            last_vehicle_disarmed_ &&
            std::isfinite(last_vehicle_disarm_received_sec_) &&
            now >= last_vehicle_disarm_received_sec_ &&
            now - last_vehicle_disarm_received_sec_ <= 0.25;
        const auto upload = owner_.handle_backend_upload(request);
        upload_ingress_.publish_result(entry->session_id, upload.http_status,
            upload.code, request.mission_run_id,
            upload.signature, upload.revision);
      }
    }

    // The upload and load paths share the single ROS owner and PX4 evidence.
    // LOAD never accepts a JSON-supplied disarm attestation.
    if (auto entry = load_ingress_.pop_for_ros(command_mailbox_.stopped_latched())) {
      const bool px4_disarmed = !telemetry_fault_ &&
          last_vehicle_disarmed_ &&
          std::isfinite(last_vehicle_disarm_received_sec_) &&
          now >= last_vehicle_disarm_received_sec_ &&
          now - last_vehicle_disarm_received_sec_ <= 0.25;
      if (!px4_disarmed) {
        load_ingress_.publish_result(entry->session_id, 412, "PX4_DISARM_REQUIRED");
      } else if (!upload_ingress_.staged_for(entry->session_id,
                    entry->load.run_id, entry->load.signature,
                    entry->load.revision)) {
        load_ingress_.publish_result(entry->session_id, 409, "DRAFT_IDENTITY_CONFLICT");
      } else {
        dyx::rpp::BackendMissionUploadRequest req{};
        req.path = dyx::rpp::kBackendMissionLoadPath;
        req.method = "POST";
        req.principal.authenticated = true;
        req.principal.owns_control_lease = true;
        req.principal.can_read_mission = true;
        req.principal.operator_session_id = entry->session_id;
        req.mission_run_id = entry->load.run_id;
        const auto staged_sequence =
            upload_ingress_.staged_sequence_for(entry->session_id);
        if (staged_sequence == 0 ||
            staged_sequence >= std::numeric_limits<std::uint64_t>::max() - entry->sequence) {
          load_ingress_.publish_result(entry->session_id, 409,
                                       "INVALID_STAGED_SEQUENCE");
        } else {
        req.sequence = staged_sequence + entry->sequence;
        req.host_received_sec = now;
        req.expected_revision = entry->load.revision;
        req.expected_signature = entry->load.signature;
        req.trusted_px4_disarmed = px4_disarmed;
        const auto result = owner_.handle_backend_upload(req);
        load_ingress_.publish_result(entry->session_id, result.http_status,
                                     result.code);
        }
      }
    }

    double cycle_dt_sec = 0.02;
    if (last_tick_sec_ >= 0.0) {
      cycle_dt_sec = now - last_tick_sec_;
      if (!std::isfinite(cycle_dt_sec) ||
          cycle_dt_sec <= 0.0 ||
          cycle_dt_sec > 0.10) {
        timing_fault_ = true;
        owner_.force_stop();
      } else if (cycle_dt_sec > 0.04) {
        ++missed_cycle_count_;
      }
    }
    last_tick_sec_ = now;

    dyx::rpp::MissionSequenceInput evidence{};
    evidence.monotonic_time_sec = now;
    evidence.emergency_stop = true;

    // No trusted operator or physical E-stop adapter is connected.
    // Never permit motion or mission execution in this build.
    dyx::rpp::NativeMissionRppInput guidance{};
    guidance.sequence = ++control_sequence_;
    guidance.guidance.now_sec = now;
    guidance.guidance.dt_sec = cycle_dt_sec;

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
    leg_input.dt_sec = cycle_dt_sec;

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

      // PX4 heading is clockwise from North.
      // Project NED velocity onto the rover's forward axis:
      // forward = North*cos(heading) + East*sin(heading).
      const double forward =
          last_velocity_north_mps_ *
              std::cos(last_position_.heading_rad) +
          last_velocity_east_mps_ *
              std::sin(last_position_.heading_rad);

      signed_forward_speed_mps_ = forward;

      // Never represent reverse motion as positive forward speed.
      // The preview API currently requires a nonnegative input,
      // so reverse velocity invalidates its diagnostic health.
      if (!std::isfinite(forward) || forward < -0.01) {
        forward_speed_valid_ = false;
        leg_input.measured_forward_speed_mps = 0.0;
      } else {
        forward_speed_valid_ = true;
        leg_input.measured_forward_speed_mps =
            std::max(0.0, forward);
      }
    }

    leg_input.telemetry_fresh =
        position_fresh &&
        reference_available &&
        last_velocity_valid_ &&
        forward_speed_valid_ &&
        !timing_fault_;

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
        << ";mailbox_normal_queued=" << command_mailbox_.queued_normal()
        << ";mailbox_stop_latched=" << command_mailbox_.stopped_latched()
        << ";mailbox_commands_processed=" << processed_commands_
        << ";control_target_hz=50"
        << ";control_cycle_dt_sec=" << cycle_dt_sec
        << ";control_missed_cycles=" << missed_cycle_count_
        << ";control_timing_fault=" << timing_fault_
        << ";signed_forward_speed_mps="
        << signed_forward_speed_mps_
        << ";forward_speed_valid=" << forward_speed_valid_
        << ";rpp_preview_valid=" << leg.valid
        << ";rpp_preview_hold_zero=" << leg.hold_zero
        << ";rpp_preview_reason=" << leg.reason
        << ";rpp_preview_forward_mps="
        << leg.forward_preview_mps
        << ";rpp_preview_yaw_rate_radps="
        << leg.yaw_rate_enu_preview_radps
        << ";trajectory_status_seen=" << trajectory_seen_
        << ";trajectory_status=" << trajectory_status_
        << ";motion_enabled=false"
        << ";spray_enabled=false"
        << ";physical_command_emitted=false";

    status.data = out.str();
    status_pub_->publish(status);
    // Original node-name status compatibility, always fail-closed.
    std_msgs::msg::String legacy_status{};
    legacy_status.data = status.data +
        ";mission_enable=false;emergency_stop=true;hold_zero=true"
        ";start_enabled=false;resume_enabled=false;physical_command_emitted=false";
    legacy_compat_status_pub_->publish(legacy_status);

    // Single-writer, read-only telemetry snapshot for HTTP.
    // Neither HTTP status nor this snapshot can authorize movement.
    native_status_->publish({
        {"service", "dyx_cpp17_http"},
        {"mission_stage",
         dyx::rpp::mission_stage_name(owner_.mission().stage())},
        {"mission_loaded", owner_.has_active_mission()},
        {"safe_stop_requested", owner_.safe_stop_requested()},
        {"px4_vehicle_fresh", result.mission.telemetry.px4_vehicle_fresh},
        {"gps_fresh", result.mission.telemetry.gps_sample_fresh},
        {"battery_fresh", result.mission.telemetry.battery_sample_fresh && telemetry_safety_.power().battery_fresh(now)},
        {"battery_voltage_v", std::isfinite(telemetry_safety_.power().voltage()) ?
            nlohmann::json(telemetry_safety_.power().voltage()) : nlohmann::json(nullptr)},
        {"battery_critical_latched", telemetry_safety_.critical_battery_latched()},
        {"battery_sample_fault", telemetry_safety_.power().battery_fault()},
        {"gps_sample_fault", telemetry_safety_.power().gps_fault()},
        {"gps_fix_type", telemetry_safety_.power().fix_type()},
        {"rtk_float_duration_sec", telemetry_safety_.rtk().float_duration_sec(now)},
        {"rtk_float_over_five_seconds", telemetry_safety_.rtk().float_over_five_seconds(now)},
        {"rtk_pause_intent", telemetry_safety_.rtk().pause_requested(now)},
        {"rtk_sample_fault", telemetry_safety_.rtk().fault()},
        {"rtk_auto_resume_authorized", false},
        {"telemetry_fault",
         telemetry_fault_ ||
         result.mission.telemetry_fault_latched},
        {"position_fault", position_fault_},
        {"rear_reference_available", reference_available},
        {"rear_reference_calibrated", false},
        // These sources are not integrated and must default fail-closed.
        {"rtk_fixed", !telemetry_fault_ && !telemetry_safety_.rtk().fault() &&
             result.mission.telemetry.gps_sample_fresh &&
             telemetry_safety_.power().rtk_fixed(now) && telemetry_safety_.rtk().fixed(now)},
        // No trusted receiver-age adapter exists yet. Never claim verified
        // correction freshness from NTRIP traffic or GPS FIX alone.
        {"corrections_fresh", false},
        {"rtcm_receiver_age_adapter_connected", false},
        {"rtcm_receiver_age_report_fresh", telemetry_safety_.rtcm().report_fresh(now)},
        {"rtcm_receiver_age_diagnostic_sec", std::isfinite(telemetry_safety_.rtcm().effective_age_sec(now)) ?
            nlohmann::json(telemetry_safety_.rtcm().effective_age_sec(now)) : nlohmann::json(nullptr)},
        {"rtcm_receiver_age_threshold_sec", dyx::http_backend::NativeRtcmAgeGuard::kMaxRtkCorrectionAgeSec},
        {"rtcm_receiver_age_source_fault", telemetry_safety_.rtcm().source_fault()},
        {"obstacle_clear_verified", false},
        {"physical_estop_clear_verified", false},
        {"operator_motion_authorized", false},
        // Critical threshold alone cannot certify reserve energy to return.
        {"battery_reserve_ok", false},
        {"battery_voltage_healthy_diagnostic", telemetry_safety_.power().battery_healthy(now)},
        {"px4_control_interface_commissioned", false},
        {"control_target_hz", 50},
        {"mailbox_normal_queued", command_mailbox_.queued_normal()},
        {"mailbox_stop_latched", command_mailbox_.stopped_latched()},
        {"mailbox_commands_processed", processed_commands_},
        {"http_command_ingress_ready", true},
        {"http_stop_only", true},
        {"load_requires_px4_disarmed", true},
        {"mission_start_enabled", false},
        {"http_upload_staging_only", true},
        {"px4_disarmed_attestation_fresh", !telemetry_fault_ &&
             last_vehicle_disarmed_ &&
             now >= last_vehicle_disarm_received_sec_ &&
             now - last_vehicle_disarm_received_sec_ <= 0.25},
        {"physical_stop_confirmed", false},
        {"control_timing_fault", timing_fault_},
        {"control_missed_cycles", missed_cycle_count_},
        {"authentication_ready", false},
        {"mission_commands_enabled", false},
        {"motor_commands_enabled", false},
        {"spray_commands_enabled", false},
        {"physical_command_emitted", false}
    });
  }

  bool trajectory_seen_{false};
  std::string trajectory_status_{"UNAVAILABLE"};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_status_sub_;
  NativeControlOwner owner_{};
  dyx::http_backend::NativeTelemetrySafetyComponent telemetry_safety_{};
  dyx::http_backend::NativeMissionCommandExecutor mission_command_executor_{};
  dyx::http_backend::NativeCommandMailbox command_mailbox_{};
  dyx::http_backend::NativeUploadIngress upload_ingress_{};
  dyx::http_backend::NativeLoadIngress load_ingress_{};
  dyx::http_backend::NativeCommandDecisionAudit command_decisions_{};
  bool last_vehicle_disarmed_{false};
  double last_vehicle_disarm_received_sec_{-1.0};
  std::uint64_t processed_commands_{0};

  // Construct the shared snapshot before the HTTP server so
  // it remains available throughout the server's lifetime.
  std::shared_ptr<dyx::http_backend::NativeStatusSnapshot>
      native_status_{
          std::make_shared<dyx::http_backend::NativeStatusSnapshot>()};

  std::unique_ptr<dyx::http_backend::NativeHttpServer> native_http_{};
  bool telemetry_fault_{false};
  std::uint64_t control_sequence_{0};
  double last_tick_sec_{-1.0};
  std::uint64_t missed_cycle_count_{0};
  bool timing_fault_{false};

  dyx::rpp::RearMarkingLeverArm lever_{};
  dyx::rpp::DdsEstimatorFrame last_position_{};

  double last_velocity_north_mps_{0.0};
  double last_velocity_east_mps_{0.0};
  bool last_velocity_valid_{false};
  double signed_forward_speed_mps_{0.0};
  bool forward_speed_valid_{false};

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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr legacy_compat_status_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
        std::make_shared<MissionManagerNode>());
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
