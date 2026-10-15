#pragma once
// Phase 2H4D: transport-neutral, NON-ACTUATING, C++17 backend API contract.
// Not an HTTP server, JSON deserializer, TLS listener, authentication system,
// or actuator bridge. The ingress adapter MUST authenticate every principal,
// validate its scopes/lease, enforce request size/timeouts and trust-boundary
// provenance, then supply only VERIFIED context and HOST monotonic timestamps.
// Read-only status deliberately never authorizes motion or acknowledges a tool.

#include "dyx_rpp_cpp17_core/mission_command_gateway.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace dyx::rpp {

// These are fixed API routes, not user-supplied path fragments or regexes.
constexpr const char* kBackendMissionStatusPath = "/api/v1/mission/status";
constexpr const char* kBackendMissionStartPath  = "/api/v1/mission/start";
constexpr const char* kBackendMissionPausePath  = "/api/v1/mission/pause";
constexpr const char* kBackendMissionResumePath = "/api/v1/mission/resume";
constexpr const char* kBackendMissionNextPath   = "/api/v1/mission/next";
constexpr const char* kBackendMissionSkipPath   = "/api/v1/mission/skip";
constexpr const char* kBackendMissionStopPath   = "/api/v1/mission/stop";

// VERIFIED facts only; the untrusted client must never select their value.
struct BackendVerifiedPrincipal {
  bool authenticated{false};
  bool can_read_mission{false};
  bool owns_control_lease{false};
  std::string operator_session_id{};
};
struct BackendMissionApiRequest {
  std::string method{"GET"};
  std::string path{};
  std::string mission_run_id{};
  std::uint64_t sequence{0};
  double host_received_sec{std::numeric_limits<double>::quiet_NaN()};
  BackendVerifiedPrincipal principal{};
};
struct BackendMissionApiResponse {
  int http_status{500};
  std::string code{"INTERNAL_ERROR"};
  // Observed mission state is ADVISORY. No permission-to-drive bit is exposed.
  MissionStage stage{MissionStage::unloaded};
  std::string stage_name{"unloaded"};
  std::string reason{};
  std::string active_marking_label{}; // display label only, no certificate/secret
  std::size_t total{0}, completed{0}, failed{0}, skipped{0}, pending{0};
  bool mission_loaded{false};
  bool safe_stop_intent{false}; // NOT an executed PX4/physical emergency stop
  bool decision_only{true};
  bool hardware_command_emitted{false}; // must always be false
  bool accepted() const { return http_status == 200; }
};

class NativeBackendMissionApi final {
 public:
  explicit NativeBackendMissionApi(NativeMissionCommandGateway& gateway)
      : gateway_(gateway) {}

  BackendMissionApiResponse handle(const BackendMissionApiRequest& req) {
    const auto action = route_action(req.path);
    const bool status_route = req.path == kBackendMissionStatusPath;
    if (!action && !status_route) return reply(404, "UNKNOWN_API_ROUTE");
    if ((status_route && req.method != "GET") ||
        (!status_route && req.method != "POST"))
      return reply(405, "METHOD_NOT_ALLOWED");
    if (!req.principal.authenticated) return reply(401, "AUTHENTICATION_REQUIRED");
    if (status_route) {
      if (!req.principal.can_read_mission) return reply(403, "READ_SCOPE_REQUIRED");
      // Never mutate the gateway on a status request. Ignore ALL command data.
      return reply(200, "STATUS_SNAPSHOT_READ_ONLY");
    }
    const bool is_stop = *action == ClientMissionAction::stop;
    if (!is_stop && !req.principal.owns_control_lease)
      return reply(403, "CONTROL_LEASE_REQUIRED");
    if (!valid_native_mission_run_id(req.mission_run_id) ||
        !valid_native_mission_run_id(req.principal.operator_session_id) ||
        req.sequence == 0)
      return reply(400, "INVALID_COMMAND_ENVELOPE");
    // For STOP, even an invalid/stale clock must never suppress safe-stop.
    if (!is_stop && (!std::isfinite(req.host_received_sec) ||
                     req.host_received_sec < 0.0))
      return reply(400, "INVALID_HOST_MONOTONIC_TIME");

    ClientCommandRequest command;
    command.action = *action;
    command.mission_run_id = req.mission_run_id;
    command.operator_session_id = req.principal.operator_session_id;
    command.session_sequence = req.sequence;
    command.received_at_host_sec = req.host_received_sec;
    command.authenticated_operator = true; // trusted principal only
    command.authorized_control_lease = req.principal.owns_control_lease;
    const auto decision = gateway_.dispatch(command);
    switch (decision.status) {
      case ClientCommandStatus::accepted: return reply(200, "MISSION_DECISION_ACCEPTED");
      case ClientCommandStatus::unauthorized: return reply(403, "CONTROL_NOT_AUTHORIZED");
      case ClientCommandStatus::invalid_identity: return reply(400, "INVALID_COMMAND_ENVELOPE");
      case ClientCommandStatus::wrong_run: return reply(409, "MISSION_IDENTITY_CONFLICT");
      case ClientCommandStatus::no_mission: return reply(409, "NO_ACTIVE_MISSION");
      case ClientCommandStatus::duplicate_or_reordered: return reply(409, "COMMAND_REPLAY_REJECTED");
      case ClientCommandStatus::invalid_clock: return reply(400, "INVALID_HOST_MONOTONIC_TIME");
      case ClientCommandStatus::stale_or_consumed_safety: return reply(412, "FRESH_SAFETY_TICK_REQUIRED");
      case ClientCommandStatus::invalid_stage: return reply(409, "INVALID_MISSION_STAGE");
      case ClientCommandStatus::runtime_rejected: return reply(412, "RUNTIME_SAFETY_REJECTED");
      case ClientCommandStatus::run_id_reused: return reply(409, "MISSION_RUN_ID_REUSED");
      case ClientCommandStatus::recovery_quarantined: return reply(423, "MISSION_RECOVERY_QUARANTINED");
    }
    return reply(500, "UNRECOGNIZED_GATEWAY_RESULT");
  }

 private:
  static const ClientMissionAction* route_action(const std::string& path) {
    static const ClientMissionAction start = ClientMissionAction::start;
    static const ClientMissionAction pause = ClientMissionAction::pause;
    static const ClientMissionAction resume = ClientMissionAction::resume;
    static const ClientMissionAction next = ClientMissionAction::next;
    static const ClientMissionAction skip = ClientMissionAction::skip;
    static const ClientMissionAction stop = ClientMissionAction::stop;
    if (path == kBackendMissionStartPath) return &start;
    if (path == kBackendMissionPausePath) return &pause;
    if (path == kBackendMissionResumePath) return &resume;
    if (path == kBackendMissionNextPath) return &next;
    if (path == kBackendMissionSkipPath) return &skip;
    if (path == kBackendMissionStopPath) return &stop;
    return nullptr;
  }
  BackendMissionApiResponse reply(int status, const char* code) const {
    BackendMissionApiResponse out;
    out.http_status = status;
    out.code = code;
    const auto mission = gateway_.mission().status();
    out.stage = mission.stage;
    out.stage_name = mission_stage_name(mission.stage);
    out.reason = mission.reason;
    out.total = mission.total;
    out.completed = mission.completed;
    out.failed = mission.failed;
    out.skipped = mission.skipped;
    const std::size_t resolved = mission.completed + mission.failed + mission.skipped;
    out.pending = mission.total >= resolved ? mission.total - resolved : 0;
    const auto& records = gateway_.mission().records();
    if (mission.current_marking < records.size())
      out.active_marking_label = records[mission.current_marking].goal.goal_label;
    out.mission_loaded = gateway_.has_active_mission();
    out.safe_stop_intent = gateway_.safe_stop_requested();
    return out;
  }
  NativeMissionCommandGateway& gateway_;
};

} // namespace dyx::rpp
