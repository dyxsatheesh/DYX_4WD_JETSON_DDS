#pragma once
// DYX Autonomous Phase 2H4P: portable, offline-only mission draft HTTP/JSON
// boundary. This class does NOT listen, authenticate, attest PX4 disarm,
// publish DDS, or control actuators. Only a trusted adapter may set principal,
// monotonic receive time and independent PX4 disarm attestation.
// Android/client JSON NEVER carries trusted fields.

#include "dyx_rpp_cpp17_core/backend_http_wire.hpp"
#include "dyx_rpp_cpp17_core/backend_mission_upload.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace dyx::rpp {

struct BackendUploadHttpConfig {
  std::size_t max_request_bytes{262144};
  std::size_t max_marking_points{10000};
  std::size_t max_response_bytes{4096};
  void validate() const {
    if (max_request_bytes < 256 || max_request_bytes > 1048576 ||
        max_marking_points < 2 || max_marking_points > 10000 ||
        max_response_bytes < 512 || max_response_bytes > 65536)
      throw std::invalid_argument("invalid upload HTTP limits");
  }
};

struct BackendTrustedUploadHttpRequest {
  BackendTrustedHttpRequest http{};
  // Supplied solely by privileged adapter using fresh PX4 state, NOT JSON.
  bool trusted_px4_disarmed{false};
};

namespace upload_json_detail {
// Exact, narrowly specified JSON grammar. Rejects duplicate/unknown keys,
// escapes/alternate encodings, nonfinite numbers, trailing bytes, and excess
// points. Does not rely on locale or permissive third-party JSON defaults.
class StrictDraftParser final {
 public:
  StrictDraftParser(const std::string& body, std::size_t maximum_points)
      : s_(body), maximum_points_(maximum_points) {}

  bool parse(BackendMissionUploadRequest& out, bool upload, bool update, bool load) {
    ws();
    if (!eat('{')) return false;
    std::uint16_t seen = 0;
    unsigned count = 0;
    ws();
    if (eat('}')) return false;
    while (true) {
      if (++count > 8) return false;
      std::string key;
      if (!token(key)) return false;
      ws();
      if (!eat(':')) return false;
      ws();
      std::uint16_t bit = 0;
      if (key == "mission_run_id") { bit = 1; if (!token(out.mission_run_id)) return false; }
      else if (key == "sequence") { bit = 2; if (!unsigned_int(out.sequence)) return false; }
      else if (key == "expected_revision") {
        bit = 4; if (upload || !unsigned_int(out.expected_revision)) return false;
      }
      else if (key == "expected_signature") {
        bit = 8; if (upload || !token(out.expected_signature)) return false;
      }
      else if (key == "marking_points") {
        bit = 16; if (load || !points(out.marking_points)) return false;
      }
      else if (key == "extension_enabled") {
        bit = 32; if (load || !boolean(out.extension_enabled)) return false;
      }
      else if (key == "extension_distance_m") {
        bit = 64; if (load || !number(out.extension_distance_m)) return false;
      }
      else if (key == "mission_mode") {
        bit = 128;
        if (load) return false;
        std::string mode;
        if (!token(mode)) return false;
        if (mode == "automatic") out.mission_mode = MissionMode::automatic;
        else if (mode == "manual") out.mission_mode = MissionMode::manual;
        else return false;
      } else return false;
      if ((seen & bit) != 0) return false;
      seen |= bit;
      ws();
      if (eat('}')) break;
      if (!eat(',')) return false;
      ws();
    }
    ws();
    const std::uint16_t required = upload ? 243 : (update ? 255 : 15);
    // UPLOAD keys: 1+2+16+32+64+128=243; UPDATE adds 4+8.
    return pos_ == s_.size() && seen == required &&
        valid_native_mission_run_id(out.mission_run_id) && out.sequence > 0 &&
        (upload || (out.expected_revision > 0 &&
                    valid_path_signature(out.expected_signature)));
  }

 private:
  void ws() {
    while (pos_ < s_.size() && (s_[pos_] == ' ' || s_[pos_] == '\n' ||
           s_[pos_] == '\t' || s_[pos_] == '\r')) ++pos_;
  }
  bool eat(char c) {
    if (pos_ >= s_.size() || s_[pos_] != c) return false;
    ++pos_;
    return true;
  }
  bool token(std::string& value) {
    if (!eat('"')) return false;
    const auto begin = pos_;
    while (pos_ < s_.size() && s_[pos_] != '"') {
      const unsigned char c = static_cast<unsigned char>(s_[pos_]);
      if (c < 0x21 || c > 0x7E || c == '\\') return false;
      ++pos_;
      if (pos_ - begin > 128) return false;
    }
    if (!eat('"')) return false;
    value.assign(s_, begin, pos_ - begin - 1);
    return true;
  }
  bool unsigned_int(std::uint64_t& out) {
    const auto begin = pos_;
    if (begin >= s_.size() || s_[begin] < '0' || s_[begin] > '9') return false;
    if (s_[begin] == '0' && begin + 1 < s_.size() &&
        s_[begin + 1] >= '0' && s_[begin + 1] <= '9') return false;
    out = 0;
    for (; pos_ < s_.size() && s_[pos_] >= '0' && s_[pos_] <= '9'; ++pos_) {
      const auto digit = static_cast<std::uint64_t>(s_[pos_] - '0');
      if (out > (std::numeric_limits<std::uint64_t>::max() - digit) / 10)
        return false;
      out = out * 10 + digit;
    }
    return pos_ != begin;
  }
  bool boolean(bool& result) {
    if (s_.compare(pos_, 4, "true") == 0) { pos_ += 4; result = true; return true; }
    if (s_.compare(pos_, 5, "false") == 0) { pos_ += 5; result = false; return true; }
    return false;
  }
  bool number(double& value) {
    const auto start = pos_;
    eat('-');
    if (pos_ >= s_.size()) return false;
    if (eat('0')) {
      if (pos_ < s_.size() && s_[pos_] >= '0' && s_[pos_] <= '9') return false;
    } else {
      if (s_[pos_] < '1' || s_[pos_] > '9') return false;
      while (pos_ < s_.size() && s_[pos_] >= '0' && s_[pos_] <= '9') ++pos_;
    }
    if (eat('.')) {
      if (pos_ >= s_.size() || s_[pos_] < '0' || s_[pos_] > '9') return false;
      while (pos_ < s_.size() && s_[pos_] >= '0' && s_[pos_] <= '9') ++pos_;
    }
    if (eat('e') || eat('E')) {
      if (!eat('+')) eat('-');
      if (pos_ >= s_.size() || s_[pos_] < '0' || s_[pos_] > '9') return false;
      while (pos_ < s_.size() && s_[pos_] >= '0' && s_[pos_] <= '9') ++pos_;
    }
    if (pos_ - start > 64) return false;
    std::istringstream in(s_.substr(start, pos_ - start));
    in.imbue(std::locale::classic());
    in >> value;
    return !in.fail() && in.peek() == std::char_traits<char>::eof() &&
        std::isfinite(value);
  }
  bool points(std::vector<Point2D>& out) {
    if (!eat('[')) return false;
    ws();
    if (eat(']')) return true; // backend rejects empty geometry
    while (true) {
      if (out.size() >= maximum_points_ || !eat('[')) return false;
      ws();
      double x = 0.0, y = 0.0;
      if (!number(x)) return false;
      ws();
      if (!eat(',')) return false;
      ws();
      if (!number(y)) return false;
      ws();
      if (!eat(']')) return false;
      out.push_back({x, y});
      ws();
      if (eat(']')) return true;
      if (!eat(',')) return false;
      ws();
    }
  }
  const std::string& s_;
  std::size_t maximum_points_;
  std::size_t pos_{0};
};
} // namespace upload_json_detail

class NativeBackendUploadHttpWire final {
 public:
  NativeBackendUploadHttpWire(NativeBackendMissionUpload& upload,
                              BackendUploadHttpConfig config = {})
      : upload_(upload), config_(config) { config_.validate(); }

  BackendHttpWireResponse handle(const BackendTrustedUploadHttpRequest& input) {
    const auto& http = input.http;
    if (http.body.size() > config_.max_request_bytes)
      return error(413, "REQUEST_BODY_TOO_LARGE");
    const bool is_upload = http.path == kBackendMissionUploadPath;
    const bool is_update = http.path == kBackendMissionUpdatePath;
    const bool is_load = http.path == kBackendMissionLoadPath;
    const std::string prefix = std::string(kBackendMissionDraftPath) + "?mission_run_id=";
    const bool is_draft = http.path.compare(0, prefix.size(), prefix) == 0;
    if (!is_upload && !is_update && !is_load && !is_draft)
      return error(404, "UNKNOWN_API_ROUTE");
    if (http.method != (is_draft ? "GET" : "POST"))
      return error(405, "METHOD_NOT_ALLOWED");
    if (!http.principal.authenticated)
      return error(401, "AUTHENTICATION_REQUIRED");
    if (!std::isfinite(http.host_received_sec) || http.host_received_sec < 0.0)
      return error(400, "INVALID_TRUSTED_RECEIVE_TIME");
    BackendMissionUploadRequest typed;
    typed.path = is_draft ? kBackendMissionDraftPath : http.path;
    typed.method = http.method;
    typed.principal = http.principal;
    typed.host_received_sec = http.host_received_sec;
    typed.trusted_px4_disarmed = input.trusted_px4_disarmed;
    if (is_draft) {
      if (!http.body.empty()) return error(400, "BODY_NOT_ALLOWED");
      typed.mission_run_id = http.path.substr(prefix.size());
      if (!valid_native_mission_run_id(typed.mission_run_id))
        return error(400, "INVALID_DRAFT_QUERY");
    } else if (!upload_json_detail::StrictDraftParser(
                   http.body, config_.max_marking_points)
                   .parse(typed, is_upload, is_update, is_load)) {
      return error(400, "INVALID_MISSION_DRAFT_JSON");
    }
    try {
      const auto result = upload_.handle(typed);
      if (result.http_status != 200) return error(result.http_status, result.code);
      const std::string body = "{\"ok\":true,\"code\":" + http_json::quoted(result.code) +
          ",\"draft\":{\"revision\":" + std::to_string(result.revision) +
          ",\"signature\":" + http_json::quoted(result.signature) +
          ",\"marking_count\":" + std::to_string(result.marking_count) +
          ",\"navigation_count\":" + std::to_string(result.navigation_count) +
          ",\"dummy_count\":" + std::to_string(result.dummy_count) +
          ",\"ready\":" + http_json::boolean(result.draft_ready) +
          ",\"loaded\":" + http_json::boolean(result.loaded) +
          ",\"preview_only\":" + http_json::boolean(result.preview_only) +
          "},\"decision_only\":true,\"hardware_command_emitted\":false}";
      if (body.size() > config_.max_response_bytes)
        return error(500, "RESPONSE_TOO_LARGE");
      return {200, body};
    } catch (const std::exception&) {
      return error(500, "DRAFT_RESPONSE_ENCODING_FAILED");
    }
  }
 private:
  static BackendHttpWireResponse error(int status, const std::string& code) {
    return {status, http_json::error_json(code)};
  }
  NativeBackendMissionUpload& upload_;
  BackendUploadHttpConfig config_{};
};

} // namespace dyx::rpp
