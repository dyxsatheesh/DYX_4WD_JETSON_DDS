#pragma once
// DYX Phase 2H5A: OFFLINE adapter; binds H4Z journal to the actual C++17
// NativeBackendMissionUpload. Never expose directly to the network/actuators.
// CRITICAL: intentionally does NOT restore backend/gateway memory after crash.
// Any journal history on construction quarantines new mutations until a future
// trusted, fully verified recovery protocol exists. STOP/safety paths outside
// this lab must remain independently available.

#include "dyx_rpp_cpp17_core/backend_draft_sqlite_mutation_journal_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_mission_upload.hpp"
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <mutex>
#include <sstream>
#include <string>

namespace dyx::rpp {
namespace journaled_upload_lab_detail {
inline bool safe_token(const std::string& s) {
  return draft_journal_lab_detail::token(s);
}
inline bool safe_signature(const std::string& s) {
  return valid_path_signature(s);
}
inline std::string numeric(double n) {
  if (!std::isfinite(n)) return {};
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17) << n;
  return out.str();
}
inline bool append_points(std::ostringstream& out,
                          const std::vector<Point2D>& points) {
  out << '[';
  for (std::size_t i=0; i<points.size(); ++i) {
    const auto x=numeric(points[i].x), y=numeric(points[i].y);
    if (x.empty() || y.empty()) return false;
    if (i) out << ',';
    out << '[' << x << ',' << y << ']';
  }
  out << ']';
  return true;
}
inline bool json_request(const BackendMissionUploadRequest& r,std::string& body) {
  if (r.method!="POST" || !safe_token(r.mission_run_id) || !r.sequence ||
      !(r.path==kBackendMissionUploadPath || r.path==kBackendMissionUpdatePath ||
        r.path==kBackendMissionLoadPath)) return false;
  const bool upload=r.path==kBackendMissionUploadPath;
  const bool load=r.path==kBackendMissionLoadPath;
  if (!upload && (!r.expected_revision || !safe_signature(r.expected_signature)))
    return false;
  if (load && !r.marking_points.empty()) return false;
  if (!load && (r.marking_points.size()<2 || r.marking_points.size()>10000 ||
      !std::isfinite(r.extension_distance_m) ||
      r.extension_distance_m<0.10 || r.extension_distance_m>20.0 ||
      (r.mission_mode!=MissionMode::automatic &&
       r.mission_mode!=MissionMode::manual))) return false;
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17) << "{\"mission_run_id\":\"" << r.mission_run_id
      << "\",\"sequence\":" << r.sequence;
  if (!upload) out << ",\"expected_revision\":" << r.expected_revision
                  << ",\"expected_signature\":\"" << r.expected_signature << '"';
  if (!load) {
    out << ",\"marking_points\":";
    if (!append_points(out,r.marking_points)) return false;
    out << ",\"extension_enabled\":" << (r.extension_enabled?"true":"false")
        << ",\"extension_distance_m\":" << numeric(r.extension_distance_m)
        << ",\"mission_mode\":\""
        << (r.mission_mode==MissionMode::manual?"manual":"automatic") << '"';
  }
  out << '}';
  body=out.str();
  return body.size()<=262144;
}
inline bool json_snapshot(const BackendMissionUploadResponse& response,
                          const BackendMissionUploadRequest& request,
                          const PreparedPathSnapshot* snapshot,
                          MissionMode mode,std::string& out_json) {
  if (!snapshot || !valid_path_signature(snapshot->signature) ||
      snapshot->signature!=response.signature ||
      !safe_token(request.mission_run_id) || !safe_token(request.principal.operator_session_id))
    return false;
  try {
    if (compute_path_signature(*snapshot)!=snapshot->signature) return false;
  } catch (const std::exception&) { return false; }
  if (response.marking_count!=snapshot->marking_points.size() ||
      response.navigation_count!=snapshot->navigation_points.size() ||
      !response.draft_ready ||
      response.loaded!=(request.path==kBackendMissionLoadPath)) return false;
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17)
      << "{\"mission_run_id\":\"" << request.mission_run_id
      << "\",\"operator_session_id\":\"" << request.principal.operator_session_id
      << "\",\"revision\":" << response.revision
      << ",\"signature\":\"" << snapshot->signature
      << "\",\"mode\":\"" << (mode==MissionMode::manual?"manual":"automatic")
      << "\",\"loaded\":" << (response.loaded?"true":"false")
      << ",\"navigation_points\":";
  if (!append_points(out,snapshot->navigation_points)) return false;
  out << ",\"marking_points\":";
  if (!append_points(out,snapshot->marking_points)) return false;
  out << ",\"point_types\":[";
  for (std::size_t i=0;i<snapshot->point_types.size();++i) {
    if (i) out << ',';
    out << snapshot->point_types[i];
  }
  out << "],\"marking_indices\":[";
  for (std::size_t i=0;i<snapshot->marking_indices.size();++i) {
    if (i) out << ',';
    out << snapshot->marking_indices[i];
  }
  out << "]}";
  out_json=out.str();
  return out_json.size()<=262144;
}
inline bool json_response(const BackendMissionUploadResponse& r,std::string& out) {
  if (!safe_token(r.code) || r.http_status<200 || r.http_status>599) return false;
  out="{\"status\":"+std::to_string(r.http_status)+
      ",\"code\":\""+r.code+"\",\"revision\":"+
      std::to_string(r.revision)+",\"loaded\":"+
      (r.loaded?"true":"false")+",\"hardware_command_emitted\":false}";
  return out.size()<=4096;
}
} // namespace journaled_upload_lab_detail

struct DraftJournaledUploadLabResult {
  BackendMissionUploadResponse backend_reply{};
  bool journal_reserved{false};
  bool backend_invoked{false};
  bool journal_completed{false};
  bool requires_reconciliation{true};
  bool hardware_command_emitted{false};
};

class NativeOfflineDraftJournaledUploadLab final {
 public:
  NativeOfflineDraftJournaledUploadLab(NativeDraftSqliteMutationJournalLab& journal,
                                     NativeBackendMissionUpload& backend)
    : journal_(journal), backend_(backend) {
    // Fresh adapter/process can only write to an EMPTY journal. An already
    // completed or PENDING mutation implies unrecovered backend memory.
    bool history=true, pending=true;
    quarantined_=!journal_.has_history(history) ||
                 !journal_.has_pending(pending) || history || pending ||
                 backend_.has_draft() || backend_.loaded();
  }
  bool quarantined() const {
    std::lock_guard<std::mutex> lock(mu_);
    return quarantined_;
  }

  DraftJournaledUploadLabResult submit(const DraftJournalLabProposal& proposal,
                                      const BackendMissionUploadRequest& request) {
    std::lock_guard<std::mutex> lock(mu_); // serializes backend + journal decisions
    DraftJournaledUploadLabResult result;
    result.backend_reply.http_status=503;
    result.backend_reply.code="JOURNAL_RECOVERY_REQUIRED";
    if (quarantined_) return result;
    if (!request.principal.authenticated || !request.principal.owns_control_lease ||
        !request.trusted_px4_disarmed ||
        !safe_request_binding(proposal,request)) {
      result.backend_reply.http_status=403;
      result.backend_reply.code="TRUSTED_BINDING_REQUIRED";
      return result;
    }
    if (request.path==kBackendMissionLoadPath && !mode_known_) {
      result.backend_reply.http_status=503;
      result.backend_reply.code="DRAFT_MODE_RECOVERY_REQUIRED";
      return result;
    }
    if (!journal_.begin(proposal)) return result;
    result.journal_reserved=true;
    result.backend_invoked=true;
    try {
      const auto backend_reply=backend_.handle(request);
      DraftJournalLabDecision decision;
      decision.http_status=backend_reply.http_status;
      if (!journaled_upload_lab_detail::json_response(backend_reply,decision.response_json) ||
          backend_reply.hardware_command_emitted) {
        quarantined_=true;
        return result;
      }
      if (backend_reply.http_status==200) {
        const auto selected_mode=request.path==kBackendMissionLoadPath ? mode_ : request.mission_mode;
        if (!journaled_upload_lab_detail::json_snapshot(backend_reply,request,
                backend_.preview_snapshot(),selected_mode,decision.complete_snapshot_json)) {
          quarantined_=true;
          return result;
        }
        decision.stage=request.path==kBackendMissionLoadPath?"READY":"PREVIEW";
      } else if (backend_reply.http_status<400 || backend_reply.http_status>599) {
        quarantined_=true;
        return result;
      }
      if (!journal_.conclude({proposal.operator_session_id,proposal.session_epoch,
                              proposal.ingress_nonce},decision)) {
        quarantined_=true; // Uncertain: never report backend acceptance.
        return result;
      }
      result.journal_completed=true;
      result.backend_reply=backend_reply;
      result.requires_reconciliation=false;
      if (backend_reply.http_status==200 &&
          request.path!=kBackendMissionLoadPath) {
        mode_=request.mission_mode;
        mode_known_=true;
      }
      if (backend_reply.http_status==200 && request.path==kBackendMissionLoadPath)
        quarantined_=true; // LOAD -> READY; no further draft mutations.
      return result;
    } catch (const std::exception&) {
      quarantined_=true; // Committed PENDING remains on disk.
      return result;
    }
  }

 private:
  static bool safe_request_binding(const DraftJournalLabProposal& p,
                                   const BackendMissionUploadRequest& r) {
    std::string canonical;
    return p.route==r.path && p.mission_run_id==r.mission_run_id &&
        p.sequence==r.sequence &&
        p.operator_session_id==r.principal.operator_session_id &&
        p.received_sec==r.host_received_sec &&
        p.session_epoch!=0 && p.ingress_nonce!=0 &&
        journaled_upload_lab_detail::json_request(r,canonical) &&
        canonical==p.canonical_request_json;
  }
  mutable std::mutex mu_{};
  NativeDraftSqliteMutationJournalLab& journal_;
  NativeBackendMissionUpload& backend_;
  bool quarantined_{true},mode_known_{false};
  MissionMode mode_{MissionMode::automatic};
};
} // namespace dyx::rpp
