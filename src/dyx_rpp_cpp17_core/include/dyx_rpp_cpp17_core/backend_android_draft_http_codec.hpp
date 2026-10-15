#pragma once
// DYX Autonomous Phase 2H4Q: offline Android-editor proposal <-> native draft
// HTTP/JSON codec. NO sockets, TLS, Android runtime, DDS, PX4 or actuation.
// The transport adapter MUST authenticate and bind each response to its
// original request; decoded JSON alone can NEVER establish that trust.

#include "dyx_rpp_cpp17_core/backend_android_mission_editor.hpp"
#include "dyx_rpp_cpp17_core/backend_upload_http_wire.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <utility>

namespace dyx::rpp {

struct AndroidDraftHttpCodecConfig {
  std::size_t max_request_bytes{262144};
  std::size_t max_response_bytes{4096};
  std::size_t max_points{10000};
  double max_reply_age_sec{8.0};
  void validate() const {
    if (max_request_bytes < 256 || max_request_bytes > 1048576 ||
        max_response_bytes < 512 || max_response_bytes > 65536 ||
        max_points < 2 || max_points > 10000 ||
        !std::isfinite(max_reply_age_sec) || max_reply_age_sec <= 0.0 ||
        max_reply_age_sec > 30.0)
      throw std::invalid_argument("invalid Android draft HTTP codec limits");
  }
};

struct AndroidDraftEncodedRequest {
  bool valid{false};
  std::string code{"ANDROID_DRAFT_NOT_ENCODED"};
  std::string method{};
  std::string path{};
  std::string body{};
  std::uint64_t sequence{0};
  bool network_io_performed{false};
  bool hardware_command_emitted{false};
};

// ALL verified_* fields are populated by the privileged transport adapter,
// never parsed from HTTP JSON or Android screen hints. A boolean is not a
// credential; production transport authentication is entirely separate.
struct AndroidTrustedDraftHttpReply {
  BackendHttpWireResponse wire{};
  bool verified_authenticated_transport{false};
  std::uint64_t verified_request_sequence{0};
  std::string verified_request_path{};
  double verified_sent_sec{std::numeric_limits<double>::quiet_NaN()};
  double verified_received_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct AndroidDraftDecodedReply {
  bool valid{false};
  std::string code{"ANDROID_DRAFT_RESPONSE_REJECTED"};
  BackendMissionUploadResponse backend{};
  std::uint64_t request_sequence{0};
  bool network_io_performed{false};
  bool hardware_command_emitted{false};
};

namespace android_draft_detail {
// Exact canonical response emitted by Phase 2H4P; reject all extra/missing,
// duplicate, escaped, alternative-encoding, and trust-bearing response fields.
// Any schema change must be explicitly versioned and retested.
class StrictDraftSuccessReader final {
 public:
  explicit StrictDraftSuccessReader(const std::string& source) : src_(source) {}
  bool parse(BackendMissionUploadResponse& out) {
    if (!take("{\"ok\":true,\"code\":") || !word(out.code) ||
        !take(",\"draft\":{\"revision\":") || !integer(out.revision) ||
        !take(",\"signature\":") || !word(out.signature) ||
        !take(",\"marking_count\":") || !count(out.marking_count) ||
        !take(",\"navigation_count\":") || !count(out.navigation_count) ||
        !take(",\"dummy_count\":") || !count(out.dummy_count) ||
        !take(",\"ready\":") || !boolean(out.draft_ready) ||
        !take(",\"loaded\":") || !boolean(out.loaded) ||
        !take(",\"preview_only\":") || !boolean(out.preview_only) ||
        !take("},\"decision_only\":true,\"hardware_command_emitted\":false}"))
      return false;
    return cursor_ == src_.size();
  }
 private:
  bool take(const char* fragment) {
    const std::string text(fragment);
    if (src_.compare(cursor_, text.size(), text) != 0) return false;
    cursor_ += text.size();
    return true;
  }
  bool word(std::string& output) {
    if (!take("\"")) return false;
    const auto start = cursor_;
    while (cursor_ < src_.size() && src_[cursor_] != '"') {
      const auto c = static_cast<unsigned char>(src_[cursor_]);
      if (c < 0x21 || c > 0x7e || c == '\\' || cursor_ - start >= 128)
        return false;
      ++cursor_;
    }
    if (cursor_ >= src_.size() || cursor_ == start) return false;
    output.assign(src_, start, cursor_ - start);
    return take("\"");
  }
  bool integer(std::uint64_t& result) {
    const auto begin = cursor_;
    if (begin >= src_.size() || src_[begin] < '0' || src_[begin] > '9') return false;
    if (src_[begin] == '0' && begin + 1 < src_.size() &&
        src_[begin + 1] >= '0' && src_[begin + 1] <= '9') return false;
    result = 0;
    while (cursor_ < src_.size() && src_[cursor_] >= '0' && src_[cursor_] <= '9') {
      const auto digit = static_cast<std::uint64_t>(src_[cursor_] - '0');
      if (result > (std::numeric_limits<std::uint64_t>::max() - digit) / 10)
        return false;
      result = result * 10 + digit;
      ++cursor_;
    }
    return cursor_ != begin;
  }
  bool count(std::size_t& result) {
    std::uint64_t value = 0;
    if (!integer(value) || value > std::numeric_limits<std::size_t>::max())
      return false;
    result = static_cast<std::size_t>(value);
    return true;
  }
  bool boolean(bool& result) {
    if (take("true")) { result = true; return true; }
    if (take("false")) { result = false; return true; }
    return false;
  }
  const std::string& src_;
  std::size_t cursor_{0};
};
inline bool draft_mutation_route(const std::string& path) {
  return path == kBackendMissionUploadPath ||
         path == kBackendMissionUpdatePath ||
         path == kBackendMissionLoadPath;
}
inline std::string format_coordinate(double value) {
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(17) << value;
  return stream.str();
}
} // namespace android_draft_detail

class NativeOfflineAndroidDraftHttpCodec final {
 public:
  explicit NativeOfflineAndroidDraftHttpCodec(AndroidDraftHttpCodecConfig cfg={})
      : cfg_(cfg) { cfg_.validate(); }

  AndroidDraftEncodedRequest encode(const AndroidEditorProposal& proposal) const {
    AndroidDraftEncodedRequest result;
    if (!proposal.prepared || !proposal.backend_authorization_required ||
        proposal.network_io_performed || proposal.hardware_command_emitted ||
        proposal.method != "POST" ||
        !android_draft_detail::draft_mutation_route(proposal.path) ||
        !valid_native_mission_run_id(proposal.mission_run_id) ||
        proposal.sequence == 0)
      return result;
    const bool upload = proposal.path == kBackendMissionUploadPath;
    const bool load = proposal.path == kBackendMissionLoadPath;
    if ((upload && (proposal.expected_revision != 0 ||
                    !proposal.expected_signature.empty())) ||
        (!upload && (proposal.expected_revision == 0 ||
                    !valid_path_signature(proposal.expected_signature))))
      return result;
    if (load && !proposal.marking_points.empty()) return result;
    if (!load) {
      if (proposal.marking_points.size() < 2 ||
          proposal.marking_points.size() > cfg_.max_points ||
          !std::isfinite(proposal.extension_distance_m) ||
          proposal.extension_distance_m < 0.10 ||
          proposal.extension_distance_m > 20.0 ||
          (proposal.mission_mode != MissionMode::automatic &&
           proposal.mission_mode != MissionMode::manual)) return result;
      for (const auto& point : proposal.marking_points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) return result;
      }
    }
    result.method = "POST";
    result.path = proposal.path;
    result.sequence = proposal.sequence;
    result.body = "{\"mission_run_id\":" + http_json::quoted(proposal.mission_run_id) +
        ",\"sequence\":" + std::to_string(proposal.sequence);
    if (!upload) {
      result.body += ",\"expected_revision\":" +
          std::to_string(proposal.expected_revision) +
          ",\"expected_signature\":" + http_json::quoted(proposal.expected_signature);
    }
    if (!load) {
      result.body += ",\"marking_points\":[";
      for (std::size_t i=0; i<proposal.marking_points.size(); ++i) {
        const auto& point = proposal.marking_points[i];
        if (i != 0) result.body += ',';
        result.body += '[' + android_draft_detail::format_coordinate(point.x) +
            ',' + android_draft_detail::format_coordinate(point.y) + ']';
        if (result.body.size() > cfg_.max_request_bytes) return {};
      }
      result.body += std::string("],\"extension_enabled\":") +
          http_json::boolean(proposal.extension_enabled) +
          ",\"extension_distance_m\":" +
          android_draft_detail::format_coordinate(proposal.extension_distance_m) +
          ",\"mission_mode\":" + http_json::quoted(
          proposal.mission_mode == MissionMode::automatic ? "automatic" : "manual");
    }
    result.body += '}';
    if (result.body.size() > cfg_.max_request_bytes) return {};
    result.valid = true;
    result.code = "ANDROID_DRAFT_REQUEST_ENCODED_NOT_SENT";
    return result;
  }

  AndroidDraftDecodedReply decode(const AndroidTrustedDraftHttpReply& reply,
       const AndroidDraftEncodedRequest& original) const {
    AndroidDraftDecodedReply result;
    if (!original.valid || original.method != "POST" ||
        !android_draft_detail::draft_mutation_route(original.path) ||
        original.sequence == 0 || !reply.verified_authenticated_transport ||
        reply.verified_request_sequence != original.sequence ||
        reply.verified_request_path != original.path ||
        !std::isfinite(reply.verified_sent_sec) ||
        !std::isfinite(reply.verified_received_sec) ||
        reply.verified_sent_sec < 0.0 ||
        reply.verified_received_sec < reply.verified_sent_sec ||
        reply.verified_received_sec - reply.verified_sent_sec > cfg_.max_reply_age_sec ||
        reply.wire.status != 200 ||
        reply.wire.content_type != "application/json; charset=utf-8" ||
        reply.wire.physical_command_emitted ||
        reply.wire.body.size() > cfg_.max_response_bytes)
      return result;
    auto& output = result.backend;
    if (!android_draft_detail::StrictDraftSuccessReader(reply.wire.body)
             .parse(output)) return result;
    const bool upload = original.path == kBackendMissionUploadPath;
    const bool update = original.path == kBackendMissionUpdatePath;
    const bool load = original.path == kBackendMissionLoadPath;
    const std::string expected_code = upload ? "UPLOAD_PREVIEW_READY" :
        (update ? "UPDATED_PREVIEW_READY" : "MISSION_LOADED_READY_NOT_STARTED");
    if (output.code != expected_code || output.revision == 0 ||
        !valid_path_signature(output.signature) ||
        output.marking_count < 2 ||
        output.navigation_count < output.marking_count ||
        output.dummy_count > output.navigation_count ||
        !output.draft_ready || output.loaded != load ||
        output.preview_only == load ||
        (upload && output.revision != 1)) return result;
    output.http_status = 200;
    output.hardware_command_emitted = false;
    result.valid = true;
    result.code = "ANDROID_DRAFT_BACKEND_DECISION_DECODED";
    result.request_sequence = original.sequence;
    return result;
  }
 private:
  AndroidDraftHttpCodecConfig cfg_{};
};
} // namespace dyx::rpp
