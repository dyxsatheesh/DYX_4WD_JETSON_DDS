#pragma once
// DYX Phase 2H4M: OFFLINE C++17 model of an Android read-only telemetry
// consumer. No Android SDK, HTTP client, sockets, cryptographic verification,
// credentials, ROS2, DDS, PX4 or physical actuation. This receives an already
// trusted *in-process* NativeOfflineLiveStream decision, not arbitrary Internet
// input. Real Android clients MUST independently parse JSON and authenticate
// transport. Last-Event-ID is a cursor hint, never authentication or authority.

#include "dyx_rpp_cpp17_core/backend_live_stream.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace dyx::rpp {

struct BackendAndroidReplicaConfig {
  double disconnected_after_sec{4.0};
  std::size_t max_frame_bytes{32768};
  void validate() const {
    if (!std::isfinite(disconnected_after_sec) || disconnected_after_sec < 0.1 ||
        disconnected_after_sec > 120.0 || max_frame_bytes < 1024 ||
        max_frame_bytes > 65536)
      throw std::invalid_argument("invalid offline Android replica limits");
  }
};

struct BackendAndroidReplicaView {
  std::string code{"ANDROID_RECONNECT_REQUIRED"};
  std::string last_event_id{}; // untrusted cursor hint only
  std::string status_json{}; // read-only backend-generated response
  bool connected{false};
  bool snapshot_fresh{false};
  bool rtk_fixed{false};
  bool requires_resync{true};
  bool mission_commands_enabled{false};
  bool network_io_performed{false};
  bool hardware_command_emitted{false};
};

class NativeOfflineAndroidTelemetryReplica final {
 public:
  explicit NativeOfflineAndroidTelemetryReplica(BackendAndroidReplicaConfig cfg = {})
      : cfg_(cfg) { cfg_.validate(); }

  // Consume the result of a *trusted in-process* stream producer. The
  // timestamp must come from a monotonic host clock, NOT an Android payload.
  BackendAndroidReplicaView accept(const BackendStreamResult& event,
                                   double trusted_host_sec) {
    if (!clock_ok(trusted_host_sec)) return view("ANDROID_HOST_CLOCK_FAULT");
    if (clock_fault_) return view("ANDROID_CLOCK_FAULT_LATCHED");
    if (!event.delivered || event.network_io_performed ||
        event.hardware_command_emitted || event.frame.empty() ||
        event.frame.size() > cfg_.max_frame_bytes ||
        !http_json::valid_utf8(event.frame))
      return fail("ANDROID_FRAME_REJECTED");

    std::uint64_t epoch = 0, seq = 0;
    std::string json;
    if (!decode_frame(event, epoch, seq, json))
      return fail("ANDROID_FRAME_REJECTED");
    if (epoch < minimum_epoch_ || (epoch_ != 0 && epoch < epoch_))
      return fail("ANDROID_OLD_EPOCH_REJECTED");

    const bool epoch_changed = epoch_ != 0 && epoch != epoch_;
    if (needs_resync_ || epoch_changed) {
      if (!event.full_resync || (epoch_changed && epoch <= epoch_) ||
          (epoch == epoch_ && seq <= seq_))
        return fail("ANDROID_FULL_RESYNC_REQUIRED");
    } else if (epoch != epoch_ || seq_ == std::numeric_limits<std::uint64_t>::max() ||
               seq != seq_ + 1 || event.full_resync) {
      // A lost, duplicated or out-of-order event invalidates the displayed
      // RTK state, and requires a fresh full snapshot before showing FIXED.
      return fail("ANDROID_EVENT_GAP_OR_REPLAY");
    }

    const std::string fresh_marker = "\"telemetry\":{\"fresh\":";
    const auto pos = json.find(fresh_marker);
    if (pos == std::string::npos ||
        json.find(fresh_marker, pos + fresh_marker.size()) != std::string::npos)
      return fail("ANDROID_SCHEMA_REJECTED");
    auto at = pos + fresh_marker.size();
    bool fresh = false, integrity = false;
    if (!boolean_at(json, at, fresh) ||
        !consume_literal(json, at, ",\"integrity_fault\":") ||
        !boolean_at(json, at, integrity) ||
        !consume_literal(json, at, ",\"rtk\":\""))
      return fail("ANDROID_SCHEMA_REJECTED");
    const auto rtk_end = json.find('"', at);
    if (rtk_end == std::string::npos || rtk_end == at || rtk_end-at > 24)
      return fail("ANDROID_SCHEMA_REJECTED");
    const auto rtk = json.substr(at, rtk_end-at);
    for (const unsigned char ch : rtk)
      if (!((ch >= 'A' && ch <= 'Z') || ch == '_'))
        return fail("ANDROID_SCHEMA_REJECTED");
    at=rtk_end+1;
    if (!consume_literal(json, at, ",\"satellites_used\":"))
      return fail("ANDROID_SCHEMA_REJECTED");
    if ((!fresh || integrity) && rtk != "UNAVAILABLE")
      return fail("ANDROID_SCHEMA_REJECTED");
    if (json.compare(0, 19, "{\"ok\":true,\"code\":\"") != 0 ||
        json.find("\",\"mission\":{") == std::string::npos ||
        !ends_with(json, ",\"read_only\":true,\"hardware_command_emitted\":false}"))
      return fail("ANDROID_SCHEMA_REJECTED");

    epoch_=epoch;
    seq_=seq;
    last_event_id_=event.event_id;
    json_=std::move(json);
    fresh_=fresh && !integrity;
    rtk_fixed_=fresh_ && rtk == "FIXED";
    connected_=true;
    needs_resync_=false;
    last_received_sec_=trusted_host_sec;
    return view(event.full_resync ? "ANDROID_FULL_SNAPSHOT" :
                (event.heartbeat ? "ANDROID_HEARTBEAT" : "ANDROID_STATUS_UPDATED"));
  }

  BackendAndroidReplicaView on_monotonic_tick(double trusted_host_sec) {
    if (!clock_ok(trusted_host_sec)) return view("ANDROID_HOST_CLOCK_FAULT");
    if (clock_fault_) return view("ANDROID_CLOCK_FAULT_LATCHED");
    if (connected_ && trusted_host_sec - last_received_sec_ > cfg_.disconnected_after_sec)
      return fail("ANDROID_STREAM_TIMEOUT");
    return view(connected_ ? "ANDROID_CURRENT" : "ANDROID_RECONNECT_REQUIRED");
  }

  BackendAndroidReplicaView on_connection_lost() {
    return fail("ANDROID_DISCONNECTED");
  }

  // Stream epoch recovery requires independent authenticated human approval
  // and confirmed DISARMED status. No motor, spray, or laser operation occurs.
  bool recover_disarmed(bool verified_operator, bool px4_disarmed,
                        std::uint64_t trusted_new_epoch, double trusted_host_sec) {
    if (!verified_operator || !px4_disarmed ||
        trusted_new_epoch <= epoch_ || trusted_new_epoch == 0 ||
        !std::isfinite(trusted_host_sec) || trusted_host_sec < 0.0)
      return false;
    clock_fault_=false;
    last_clock_sec_=trusted_host_sec;
    minimum_epoch_=trusted_new_epoch;
    clear_visible();
    return true;
  }

  const std::string& last_event_id_hint() const { return last_event_id_; }
  bool clock_fault_latched() const { return clock_fault_; }

 private:
  static bool consume_literal(const std::string& data, std::size_t& at,
                              const char* literal) {
    const std::string s(literal);
    if (data.compare(at, s.size(), s) != 0) return false;
    at+=s.size();
    return true;
  }
  static bool boolean_at(const std::string& json, std::size_t& at, bool& value) {
    if (json.compare(at, 4, "true") == 0) { at+=4; value=true; return true; }
    if (json.compare(at, 5, "false") == 0) { at+=5; value=false; return true; }
    return false;
  }
  static bool ends_with(const std::string& s, const std::string& tail) {
    return s.size() >= tail.size() &&
        s.compare(s.size()-tail.size(), tail.size(), tail) == 0;
  }
  static bool strict_positive(const std::string& s, std::size_t b, std::size_t e,
                              std::uint64_t& out) {
    if (b == e || e-b > 20 || (e-b > 1 && s[b] == '0')) return false;
    out=0;
    for (auto i=b; i<e; ++i) {
      if (s[i] < '0' || s[i] > '9') return false;
      const auto digit=static_cast<std::uint64_t>(s[i]-'0');
      if (out > (std::numeric_limits<std::uint64_t>::max()-digit)/10) return false;
      out=out*10+digit;
    }
    return out != 0;
  }
  static bool decode_frame(const BackendStreamResult& event,
                           std::uint64_t& epoch, std::uint64_t& seq,
                           std::string& json) {
    const auto& frame=event.frame;
    const std::string prefix="id: ";
    const std::string middle="\nevent: rover.status\ndata: ";
    if (frame.compare(0,prefix.size(),prefix)!=0 ||
        !ends_with(frame,"\n\n")) return false;
    const auto split=frame.find(middle,prefix.size());
    if (split==std::string::npos) return false;
    const auto id=frame.substr(prefix.size(),split-prefix.size());
    if (id != event.event_id || id.size()>41) return false;
    const auto dash=id.find('-');
    if (dash==std::string::npos || id.find('-',dash+1)!=std::string::npos ||
        !strict_positive(id,0,dash,epoch) ||
        !strict_positive(id,dash+1,id.size(),seq)) return false;
    const auto json_start=split+middle.size();
    if (json_start+2>frame.size()) return false;
    json=frame.substr(json_start,frame.size()-json_start-2);
    return !json.empty() && json.find('\r')==std::string::npos &&
        json.find('\n')==std::string::npos;
  }
  bool clock_ok(double t) {
    if (clock_fault_) return false;
    if (!std::isfinite(t) || t < 0.0 || t < last_clock_sec_) {
      clock_fault_=true;
      clear_visible();
      return false;
    }
    last_clock_sec_=t;
    return true;
  }
  void clear_visible() {
    connected_=false;
    fresh_=false;
    rtk_fixed_=false;
    needs_resync_=true;
    json_.clear();
    // Retain ONLY the last accepted cursor to request full resynchronization;
    // it is not an authorization credential and is never shown as fresh data.
  }
  BackendAndroidReplicaView fail(const std::string& why) {
    clear_visible();
    return view(why);
  }
  BackendAndroidReplicaView view(const std::string& why) const {
    BackendAndroidReplicaView s;
    s.code=why;
    s.last_event_id=last_event_id_;
    s.status_json=json_;
    s.connected=connected_;
    s.snapshot_fresh=fresh_;
    s.rtk_fixed=rtk_fixed_;
    s.requires_resync=needs_resync_;
    return s;
  }
  BackendAndroidReplicaConfig cfg_{};
  std::uint64_t epoch_{0};
  std::uint64_t seq_{0};
  std::uint64_t minimum_epoch_{1};
  double last_clock_sec_{0.0};
  double last_received_sec_{0.0};
  bool connected_{false};
  bool fresh_{false};
  bool rtk_fixed_{false};
  bool needs_resync_{true};
  bool clock_fault_{false};
  std::string last_event_id_{};
  std::string json_{};
};

} // namespace dyx::rpp
