#pragma once
// Phase 2H4G — offline HTTP/JSON wire contract for the Android rover UI.
// NOT an HTTP listener, TLS stack, authentication implementation or WebSocket.
// The caller MUST authenticate independently, enforce TLS and request limits,
// provide monotonic server time and verified principal from a trusted source.
// Never derive principal, operator session or control lease from client JSON.
// All responses are decisions / read-only telemetry; no physical commands.

#include "dyx_rpp_cpp17_core/backend_http_json.hpp"
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace dyx::rpp {

struct BackendHttpWireConfig {
  std::size_t max_request_body_bytes{512};
  std::size_t max_response_body_bytes{16384};
  void validate() const {
    if (max_request_body_bytes < 32 || max_request_body_bytes > 4096 ||
        max_response_body_bytes < 1024 || max_response_body_bytes > 65536)
      throw std::invalid_argument("invalid HTTP wire limits");
  }
};

struct BackendTrustedHttpRequest {
  std::string method{"GET"};
  std::string path{};
  std::string body{}; // untrusted JSON for POST: never contains credentials
  BackendVerifiedPrincipal principal{}; // verified by trusted ingress ONLY
  double host_received_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct BackendHttpWireResponse {
  int status{500};
  std::string body{};
  std::string content_type{"application/json; charset=utf-8"};
  bool physical_command_emitted{false};
  bool network_io_performed{false};

  // Prepared bytes are not transmitted by this class.
  std::string http11_bytes() const {
    const char* reason = nullptr;
    switch (status) {
      case 200: reason = "OK"; break;
      case 400: reason = "Bad Request"; break;
      case 401: reason = "Unauthorized"; break;
      case 403: reason = "Forbidden"; break;
      case 404: reason = "Not Found"; break;
      case 405: reason = "Method Not Allowed"; break;
      case 408: reason = "Request Timeout"; break;
      case 409: reason = "Conflict"; break;
      case 412: reason = "Precondition Failed"; break;
      case 413: reason = "Content Too Large"; break;
      case 422: reason = "Unprocessable Content"; break;
      case 423: reason = "Locked"; break;
      case 500: reason = "Internal Server Error"; break;
      default: throw std::invalid_argument("unknown HTTP status");
    }
    if (content_type != "application/json; charset=utf-8")
      throw std::logic_error("unsafe response content type");
    return "HTTP/1.1 " + std::to_string(status) + " " + reason + "\r\n"
        "Content-Type: application/json; charset=utf-8\r\n"
        "Cache-Control: no-store\r\nPragma: no-cache\r\n"
        "X-Content-Type-Options: nosniff\r\n"
        "Connection: close\r\nContent-Length: " + std::to_string(body.size()) +
        "\r\n\r\n" + body;
  }
};

namespace http_detail {
struct ParsedCommandBody {
  std::string run_id{};
  std::uint64_t sequence{0};
  bool valid{false};
};

// Deliberately narrow, complete JSON subset. POST accepts exactly two keys:
// {"mission_run_id":"...", "sequence":123}; no implicit coercion, escapes,
// duplicates, extra fields, fractional/negative or overflowing integers.
// Invalid inputs cause rejection, never silent interpretation or actuation.
class StrictCommandParser final {
 public:
  explicit StrictCommandParser(const std::string& text) : s_(text) {}
  ParsedCommandBody parse() {
    ParsedCommandBody r;
    ws();
    if (!eat('{')) return {};
    bool seen_run = false, seen_seq = false;
    for (unsigned n = 0; n < 2; ++n) {
      ws();
      std::string key;
      if (!identifier(key)) return {};
      ws();
      if (!eat(':')) return {};
      ws();
      if (key == "mission_run_id" && !seen_run) {
        if (!identifier(r.run_id)) return {};
        seen_run = true;
      } else if (key == "sequence" && !seen_seq) {
        if (!number(r.sequence)) return {};
        seen_seq = true;
      } else return {};
      ws();
      if (n == 0) { if (!eat(',')) return {}; }
    }
    ws();
    if (!eat('}')) return {};
    ws();
    r.valid = seen_run && seen_seq && pos_ == s_.size() &&
        valid_native_mission_run_id(r.run_id) && r.sequence > 0;
    return r;
  }
 private:
  void ws() {
    while (pos_ < s_.size() && (s_[pos_] == ' ' || s_[pos_] == '\t' ||
            s_[pos_] == '\n' || s_[pos_] == '\r')) ++pos_;
  }
  bool eat(char c) {
    if (pos_ >= s_.size() || s_[pos_] != c) return false;
    ++pos_;
    return true;
  }
  bool identifier(std::string& out) {
    if (!eat('"')) return false;
    const auto begin = pos_;
    while (pos_ < s_.size() && s_[pos_] != '"') {
      const unsigned char c = static_cast<unsigned char>(s_[pos_]);
      // Only printable ASCII tokens with no JSON escapes are accepted.
      // This avoids alternate encodings of permission or identity fields.
      if (c < 0x21 || c > 0x7E || c == '\\') return false;
      ++pos_;
    }
    if (pos_ == s_.size()) return false;
    out.assign(s_, begin, pos_ - begin);
    ++pos_;
    return true;
  }
  bool number(std::uint64_t& out) {
    const auto first = pos_;
    if (first >= s_.size() || s_[first] < '0' || s_[first] > '9') return false;
    if (s_[first] == '0' && first + 1 < s_.size() &&
        s_[first + 1] >= '0' && s_[first + 1] <= '9') return false;
    out = 0;
    for (; pos_ < s_.size() && s_[pos_] >= '0' && s_[pos_] <= '9'; ++pos_) {
      const auto digit = static_cast<std::uint64_t>(s_[pos_] - '0');
      if (out > (std::numeric_limits<std::uint64_t>::max() - digit) / 10) return false;
      out = out * 10 + digit;
    }
    return pos_ > first;
  }
  const std::string& s_;
  std::size_t pos_{0};
};
} // namespace http_detail

class NativeBackendHttpWire final {
 public:
  NativeBackendHttpWire(NativeBackendMissionApi& mission,
                        NativeBackendLiveStatus& live,
                        BackendHttpWireConfig config = {})
      : mission_(mission), live_(live), config_(config) { config_.validate(); }

  BackendHttpWireResponse handle(const BackendTrustedHttpRequest& request) {
    // Even malformed/oversized bodies cannot become a mission STOP/START.
    if (request.body.size() > config_.max_request_body_bytes)
      return error(413, "REQUEST_BODY_TOO_LARGE");
    try {
      if (request.path == kBackendLiveStatusPath) {
        if (!request.body.empty()) return error(400, "BODY_NOT_ALLOWED");
        BackendLiveStatusRequest q;
        q.method = request.method;
        q.path = request.path;
        q.host_now_sec = request.host_received_sec;
        q.principal = request.principal;
        const auto v = live_.handle(q);
        return v.http_status == 200 ? response(200, http_json::live_json(v)) :
            error(v.http_status, v.code);
      }
      // A status GET has no side effects. A POST always uses a typed and
      // strictly validated request; client-provided trust flags are rejected.
      BackendMissionApiRequest q;
      q.method = request.method;
      q.path = request.path;
      q.principal = request.principal;
      q.host_received_sec = request.host_received_sec;
      if (request.method == "GET" || request.path == kBackendMissionStatusPath) {
        if (!request.body.empty()) return error(400, "BODY_NOT_ALLOWED");
      } else if (request.method == "POST") {
        // Do not parse/dispatch unknown POST routes.
        if (request.path != kBackendMissionStartPath &&
            request.path != kBackendMissionPausePath &&
            request.path != kBackendMissionResumePath &&
            request.path != kBackendMissionNextPath &&
            request.path != kBackendMissionSkipPath &&
            request.path != kBackendMissionStopPath)
          return error(404, "UNKNOWN_API_ROUTE");
        // Before parsing any body, require an authenticated operator.
        if (!request.principal.authenticated)
          return error(401, "AUTHENTICATION_REQUIRED");
        const auto parsed = http_detail::StrictCommandParser(request.body).parse();
        if (!parsed.valid) return error(400, "INVALID_COMMAND_JSON");
        q.mission_run_id = parsed.run_id;
        q.sequence = parsed.sequence;
      }
      const auto v = mission_.handle(q);
      return v.http_status == 200 ? response(200, http_json::mission_json(v)) :
          error(v.http_status, v.code);
    } catch (const std::exception&) {
      return error(500, "RESPONSE_ENCODING_FAILED");
    }
  }

  // Single authorized read-only SSE frame for a future trusted TLS stream
  // adapter. No socket, replay cache, subscription, timer or stream is created.
  // The event ID must be generated by the trusted stream owner, not Android.
  std::string make_live_sse_frame(const BackendTrustedHttpRequest& request,
                                  std::uint64_t trusted_event_id) {
    if (trusted_event_id == 0 || request.path != kBackendLiveStatusPath ||
        request.method != "GET") return {};
    const auto reply = handle(request);
    if (reply.status != 200) return {}; // no data on auth failure
    return "id: " + std::to_string(trusted_event_id) +
        "\nevent: rover.status\ndata: " + reply.body + "\n\n";
  }
 private:
  BackendHttpWireResponse response(int code, std::string body) const {
    if (body.size() > config_.max_response_body_bytes)
      return error(500, "RESPONSE_TOO_LARGE");
    return {code, std::move(body)};
  }
  static BackendHttpWireResponse error(int code, const std::string& reason) {
    return {code, http_json::error_json(reason)};
  }
  NativeBackendMissionApi& mission_;
  NativeBackendLiveStatus& live_;
  BackendHttpWireConfig config_{};
};

} // namespace dyx::rpp
