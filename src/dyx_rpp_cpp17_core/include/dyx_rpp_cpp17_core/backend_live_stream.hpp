#pragma once
// DYX Phase 2H4L: deterministic OFFLINE SSE event/reconnect policy.
// This component performs NO socket I/O, TLS, authentication, DDS, PX4,
// mission control or physical actuation. Only a separately authenticated,
// serialized stream owner may populate BackendTrustedHttpRequest. Client
// Last-Event-ID is merely an untrusted cursor hint, NEVER authorization.
// Single owner / subscriber; external synchronization is mandatory.

#include "dyx_rpp_cpp17_core/backend_http_wire.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace dyx::rpp {

struct BackendLiveStreamConfig {
  double heartbeat_sec{2.0};
  std::size_t max_sse_frame_bytes{32768};
  void validate() const {
    if (!std::isfinite(heartbeat_sec) || heartbeat_sec < 0.1 ||
        heartbeat_sec > 30.0 || max_sse_frame_bytes < 1024 ||
        max_sse_frame_bytes > 65536)
      throw std::invalid_argument("invalid offline live-stream limits");
  }
};

struct BackendStreamResult {
  std::string code{"STREAM_NOT_CONNECTED"};
  std::string frame{};
  std::string event_id{};
  bool delivered{false}; // built a frame, not proof any network client received it
  bool full_resync{false};
  bool heartbeat{false};
  bool network_io_performed{false};
  bool hardware_command_emitted{false};
};

class NativeOfflineLiveStream final {
 public:
  // A trusted owner MUST choose a unique, persistent, nonzero service epoch;
  // after restart/recovery choose a STRICTLY HIGHER epoch. The value must not
  // originate from the Android client. No persistence is implemented here.
  NativeOfflineLiveStream(NativeBackendHttpWire& wire,
                          std::uint64_t trusted_service_epoch,
                          BackendLiveStreamConfig cfg = {})
      : wire_(wire), epoch_(trusted_service_epoch), cfg_(cfg) {
    cfg_.validate();
    if (epoch_ == 0) throw std::invalid_argument("zero trusted stream epoch");
  }

  // Always re-snapshot on reconnect: no replay buffer or retained authority.
  // Even a valid Last-Event-ID does not permit replay or skipping revalidation.
  BackendStreamResult connect(const BackendTrustedHttpRequest& trusted,
                              const std::string& untrusted_last_event_id = {}) {
    const auto result = validated_read(trusted);
    if (result.code != "STREAM_READ_OK") return result;
    // Another verified reader cannot evict an existing subscriber. No
    // client-supplied reconnect cursor grants takeover authority.
    if (connected_ && trusted.principal.operator_session_id != session_)
      return {"STREAM_SUBSCRIBER_BUSY"};
    connected_ = false;
    session_.clear();
    last_json_.clear();
    const auto cursor = parse_cursor(untrusted_last_event_id);
    std::string reason = "STREAM_INITIAL_SNAPSHOT";
    if (!untrusted_last_event_id.empty()) {
      reason = cursor.valid && cursor.epoch == epoch_ &&
          cursor.sequence <= sequence_ && cursor.sequence > 0 ?
          "STREAM_RECONNECTED_NO_REPLAY" : "STREAM_CURSOR_RESYNC_REQUIRED";
    }
    session_ = trusted.principal.operator_session_id;
    connected_ = true;
    return produce(trusted, true, false, reason);
  }

  BackendStreamResult poll(const BackendTrustedHttpRequest& trusted) {
    if (!connected_) return {"STREAM_NOT_CONNECTED"};
    // A different principal must not evict a currently authorized stream.
    if (trusted.principal.operator_session_id != session_)
      return {"STREAM_SUBSCRIBER_MISMATCH"};
    if (!trusted.principal.authenticated || !trusted.principal.can_read_mission) {
      disconnect();
      return {"STREAM_SESSION_REVOKED"};
    }
    const auto validation = validated_read(trusted);
    if (validation.code != "STREAM_READ_OK") {
      disconnect();
      return validation;
    }
    const auto current = wire_.handle(trusted);
    if (current.status != 200) {
      disconnect();
      return {"STREAM_READ_REJECTED"};
    }
    const bool changed = current.body != last_json_;
    const bool heartbeat = !changed &&
        trusted.host_received_sec - last_emitted_sec_ >= cfg_.heartbeat_sec;
    if (!changed && !heartbeat) return {"STREAM_UNCHANGED"};
    return produce_body(trusted.host_received_sec, current.body,
                        false, heartbeat,
                        heartbeat ? "STREAM_HEARTBEAT_SNAPSHOT" : "STREAM_STATUS_CHANGED");
  }

  void disconnect() {
    connected_ = false;
    session_.clear();
    last_json_.clear();
  }

  // A stream clock fault cannot self-clear. A verified human recovery and
  // independent PX4 DISARMED state are prerequisites. This only resets the
  // offline SSE cursor; it does not reset mission, safety or DDS faults.
  bool reset_epoch_disarmed(bool verified_operator, bool px4_disarmed,
                            std::uint64_t trusted_new_epoch,
                            double trusted_host_now_sec) {
    if (!verified_operator || !px4_disarmed || trusted_new_epoch <= epoch_ ||
        !std::isfinite(trusted_host_now_sec) || trusted_host_now_sec < 0.0)
      return false;
    epoch_ = trusted_new_epoch;
    sequence_ = 0;
    last_clock_sec_ = trusted_host_now_sec;
    clock_fault_ = false;
    disconnect();
    return true;
  }

  bool clock_fault_latched() const { return clock_fault_; }
  bool connected() const { return connected_; }
  std::uint64_t epoch() const { return epoch_; }
  std::uint64_t last_event_sequence() const { return sequence_; }

 private:
  struct Cursor { bool valid{false}; std::uint64_t epoch{0}; std::uint64_t sequence{0}; };
  static bool decimal(const std::string& s, std::size_t first,
                      std::size_t last, std::uint64_t& value) {
    if (first == last || last > s.size() ||
        (last-first > 1 && s[first] == '0')) return false;
    value=0;
    for (auto i=first; i<last; ++i) {
      if (s[i] < '0' || s[i] > '9') return false;
      const auto digit=static_cast<std::uint64_t>(s[i]-'0');
      if (value > (std::numeric_limits<std::uint64_t>::max()-digit)/10)
        return false;
      value=value*10+digit;
    }
    return value>0;
  }
  static Cursor parse_cursor(const std::string& text) {
    if (text.empty() || text.size() > 41) return {};
    const auto dash=text.find('-');
    if (dash==std::string::npos || text.find('-',dash+1)!=std::string::npos)
      return {};
    Cursor c;
    c.valid=decimal(text,0,dash,c.epoch) &&
            decimal(text,dash+1,text.size(),c.sequence);
    return c;
  }
  BackendStreamResult validated_read(const BackendTrustedHttpRequest& request) {
    if (clock_fault_) return {"STREAM_CLOCK_FAULT_LATCHED"};
    if (!std::isfinite(request.host_received_sec) ||
        request.host_received_sec < 0.0 ||
        request.host_received_sec < last_clock_sec_) {
      clock_fault_=true;
      disconnect();
      return {"STREAM_HOST_CLOCK_FAULT"};
    }
    // This clock is supplied by trusted host ingress. The caller must never
    // derive it from a client's request timestamp.
    last_clock_sec_=request.host_received_sec;
    if (request.method!="GET" || request.path!=kBackendLiveStatusPath ||
        !request.body.empty() || !request.principal.authenticated ||
        !request.principal.can_read_mission ||
        !valid_native_mission_run_id(request.principal.operator_session_id))
      return {"STREAM_READ_NOT_AUTHORIZED"};
    return {"STREAM_READ_OK"};
  }
  BackendStreamResult produce(const BackendTrustedHttpRequest& trusted,
                              bool resync, bool heartbeat,
                              const std::string& reason) {
    const auto data=wire_.handle(trusted);
    if (data.status!=200) {
      disconnect();
      return {"STREAM_READ_REJECTED"};
    }
    return produce_body(trusted.host_received_sec,data.body,resync,heartbeat,reason);
  }
  BackendStreamResult produce_body(double now, const std::string& json,
                                    bool resync, bool heartbeat,
                                    const std::string& reason) {
    if (sequence_ == std::numeric_limits<std::uint64_t>::max()) {
      clock_fault_=true; // never wrap and reuse an event ID
      disconnect();
      return {"STREAM_EVENT_ID_EXHAUSTED"};
    }
    const auto id=std::to_string(epoch_)+"-"+std::to_string(sequence_+1);
    const auto frame="id: "+id+"\nevent: rover.status\ndata: "+json+"\n\n";
    if (frame.size() > cfg_.max_sse_frame_bytes) {
      disconnect();
      return {"STREAM_FRAME_TOO_LARGE"};
    }
    ++sequence_;
    last_json_=json;
    last_emitted_sec_=now;
    BackendStreamResult result;
    result.code=reason;
    result.frame=frame;
    result.event_id=id;
    result.delivered=true;
    result.full_resync=resync;
    result.heartbeat=heartbeat;
    return result;
  }
  NativeBackendHttpWire& wire_;
  std::uint64_t epoch_{0};
  std::uint64_t sequence_{0};
  BackendLiveStreamConfig cfg_{};
  bool connected_{false};
  bool clock_fault_{false};
  double last_clock_sec_{0.0};
  double last_emitted_sec_{0.0};
  std::string session_{};
  std::string last_json_{};
};

} // namespace dyx::rpp
