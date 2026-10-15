#pragma once
// DYX Autonomous Phase 2H4U: OFFLINE framing and transaction-binding lab.
// NOT a network HTTP parser, TLS/authentication provider, PX4 attestation
// source, DDS transport or actuator. All evidence is simulated in-process.
// This deliberately accepts ONLY a canonical, laboratory HTTP/1.1 subset.

#include "dyx_rpp_cpp17_core/backend_draft_ingress_lab.hpp"
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace dyx::rpp {

struct DraftHttpFramingLabConfig {
  std::size_t max_header_bytes{2048};
  std::size_t max_body_bytes{262144};
  void validate() const {
    if (max_header_bytes < 256 || max_header_bytes > 8192 ||
        max_body_bytes < 256 || max_body_bytes > 1048576)
      throw std::invalid_argument("invalid offline HTTP framing limits");
  }
};

namespace draft_http_framing_detail {
struct Parsed {
  bool valid{false};
  std::string method{};
  std::string path{};
  std::string body{};
};

// Reject all HTTP extensions that can create different interpretations in
// proxies/servers (transfer-encoding, duplicate/ambiguous length, obs-fold,
// absolute-form target, pipelining, and unrecognized headers). No real socket.
inline Parsed parse(const std::string& raw, const DraftHttpFramingLabConfig& cfg) {
  Parsed out;
  if (raw.size() > cfg.max_header_bytes + cfg.max_body_bytes) return {};
  const auto boundary=raw.find("\r\n\r\n");
  if (boundary==std::string::npos || boundary+4>cfg.max_header_bytes) return {};
  if (raw.find('\0')!=std::string::npos) return {};
  const auto first=raw.find("\r\n");
  if (first==std::string::npos || first>boundary) return {};
  const auto line=raw.substr(0,first);
  const auto separator=line.find(' ');
  if (separator==std::string::npos || separator==0) return {};
  const auto version=line.rfind(" HTTP/1.1");
  if (version==std::string::npos || version<=separator ||
      version+9!=line.size()) return {};
  out.method=line.substr(0,separator);
  out.path=line.substr(separator+1,version-separator-1);
  if ((out.method!="GET" && out.method!="POST") ||
      out.path.empty() || out.path.size()>256 || out.path[0]!='/' ||
      out.path.find(' ')!=std::string::npos ||
      out.path.find('\r')!=std::string::npos ||
      out.path.find('\n')!=std::string::npos ||
      out.path.find('#')!=std::string::npos) return {};
  for (const unsigned char c : out.path)
    if (c<0x21 || c>0x7e || c=='\\') return {};

  bool host=false,content_type=false,content_length=false;
  std::size_t declared_length=0;
  auto offset=first+2;
  while (offset<boundary) {
    const auto end=raw.find("\r\n",offset);
    if (end==std::string::npos || end>boundary || end==offset) return {};
    const auto header=raw.substr(offset,end-offset);
    if (header=="Host: offline.invalid" && !host) host=true;
    else if (header=="Content-Type: application/json" && !content_type)
      content_type=true;
    else if (header.compare(0,16,"Content-Length: ")==0 && !content_length) {
      content_length=true;
      const auto digits=header.substr(16);
      if (digits.empty() || (digits.size()>1 && digits[0]=='0')) return {};
      for (const char c: digits) {
        if (c<'0' || c>'9') return {};
        const auto digit=static_cast<std::size_t>(c-'0');
        if (declared_length>(cfg.max_body_bytes-digit)/10) return {};
        declared_length=declared_length*10+digit;
      }
    } else return {}; // Reject auth/spoofed verified headers, TE, duplicates, etc.
    offset=end+2;
  }
  if (!host || !content_length ||
      (out.method=="POST" ? !content_type : content_type || declared_length!=0))
    return {};
  if (raw.size()-boundary-4 != declared_length ||
      declared_length>cfg.max_body_bytes) return {};
  out.body=raw.substr(boundary+4);
  if (out.method=="POST" ? out.body.empty() : !out.body.empty()) return {};
  out.valid=true;
  return out;
}

inline DraftIngressLabOutcome deny() {
  DraftIngressLabOutcome result;
  result.code="FRAMING_LAB_DENIED";
  result.wire={400,http_json::error_json("FRAMING_LAB_DENIED")};
  return result;
}
} // namespace draft_http_framing_detail

class NativeOfflineDraftHttpFramingLab final {
 public:
  NativeOfflineDraftHttpFramingLab(NativeOfflineDraftIngressLab& ingress,
                                  DraftHttpFramingLabConfig cfg={})
    : ingress_(ingress),cfg_(cfg) { cfg_.validate(); }

  DraftIngressLabOutcome mutation(const std::string& raw_http,
      const AndroidDraftEncodedRequest& expected,
      const DraftIngressLabEvidence& evidence) {
    const auto parsed=draft_http_framing_detail::parse(raw_http,cfg_);
    if (!parsed.valid || parsed.method!="POST" || !expected.valid ||
        expected.method!=parsed.method || expected.path!=parsed.path ||
        expected.body!=parsed.body || expected.sequence==0 ||
        !android_draft_detail::draft_mutation_route(parsed.path))
      return draft_http_framing_detail::deny();
    // Independent syntactic cross-check: request.sequence must agree with the
    // body the backend will parse, NOT an HTTP header or Android UI claim.
    BackendMissionUploadRequest typed;
    const bool upload=parsed.path==kBackendMissionUploadPath;
    const bool update=parsed.path==kBackendMissionUpdatePath;
    const bool load=parsed.path==kBackendMissionLoadPath;
    if (!upload_json_detail::StrictDraftParser(parsed.body,10000)
            .parse(typed,upload,update,load) ||
        typed.sequence!=expected.sequence)
      return draft_http_framing_detail::deny();
    return ingress_.mutation(expected,evidence);
  }

  DraftIngressLabOutcome snapshot(const std::string& raw_http,
      const AndroidDraftSyncRequest& expected,
      const DraftIngressLabEvidence& evidence) {
    const auto parsed=draft_http_framing_detail::parse(raw_http,cfg_);
    if (!parsed.valid || parsed.method!="GET" || !parsed.body.empty() ||
        !expected.valid || expected.method!=parsed.method ||
        expected.path!=parsed.path || expected.request_id==0 ||
        !expected.body.empty()) return draft_http_framing_detail::deny();
    return ingress_.snapshot(expected,evidence);
  }
 private:
  NativeOfflineDraftIngressLab& ingress_;
  DraftHttpFramingLabConfig cfg_{};
};

} // namespace dyx::rpp
