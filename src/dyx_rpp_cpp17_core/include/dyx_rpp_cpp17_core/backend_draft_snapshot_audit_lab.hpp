#pragma once
// DYX Autonomous Phase 2H5C: offline-only, READ-ONLY saved mission snapshot
// consistency audit. This validates persisted H5A JSON against real snapshot
// signature rules but it DOES NOT authenticate its publisher or authorize
// LOAD, START, RESUME, motor, spray, laser, or mission-state restoration.
// No JSON/HTTP listener, DDS publishing, actuator IO or database mutation.

#include "dyx_rpp_cpp17_core/backend_draft_reconciliation_read_lab.hpp"
#include "dyx_rpp_cpp17_core/backend_draft_journaled_upload_lab.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace dyx::rpp {
namespace draft_audit_lab_detail {

// Deliberately a strict decoder for ONE deterministic, H5A-generated JSON
// layout. Not a general-purpose JSON parser and never for untrusted HTTP.
class StrictSnapshotDecoder final {
 public:
  explicit StrictSnapshotDecoder(const std::string& bytes) : s_(bytes) {}

  struct Parsed {
    std::string run, operator_session;
    std::uint64_t revision{0};
    MissionMode mode{MissionMode::automatic};
    bool loaded{false};
    PreparedPathSnapshot snapshot{};
  };

  bool decode(Parsed& out) {
    if (s_.size()<64 || s_.size()>262144) return false;
    Parsed candidate;
    std::string mode;
    if (!eat("{\"mission_run_id\":") || !token(candidate.run) ||
        !eat(",\"operator_session_id\":") || !token(candidate.operator_session) ||
        !eat(",\"revision\":") || !unsigned_integer(candidate.revision) ||
        candidate.revision==0 ||
        !eat(",\"signature\":") || !token(candidate.snapshot.signature) ||
        !eat(",\"mode\":") || !token(mode) ||
        !eat(",\"loaded\":") || !boolean(candidate.loaded) ||
        !eat(",\"navigation_points\":") ||
        !points(candidate.snapshot.navigation_points,200000) ||
        !eat(",\"marking_points\":") ||
        !points(candidate.snapshot.marking_points,10000) ||
        !eat(",\"point_types\":") ||
        !integers(candidate.snapshot.point_types,200000) ||
        !eat(",\"marking_indices\":") ||
        !integers(candidate.snapshot.marking_indices,200000) ||
        !eat("}") || pos_!=s_.size()) return false;
    if (mode=="automatic") candidate.mode=MissionMode::automatic;
    else if (mode=="manual") candidate.mode=MissionMode::manual;
    else return false;
    if (!valid_native_mission_run_id(candidate.run) ||
        !valid_native_mission_run_id(candidate.operator_session) ||
        !valid_path_signature(candidate.snapshot.signature)) return false;
    try {
      if (compute_path_signature(candidate.snapshot)!=candidate.snapshot.signature)
        return false;
    } catch (const std::exception&) { return false; }
    out=std::move(candidate);
    return true;
  }

 private:
  bool eat(const char* text) {
    const auto n=std::char_traits<char>::length(text);
    if (s_.compare(pos_,n,text)!=0) return false;
    pos_+=n;
    return true;
  }
  bool take(char ch) {
    if (pos_>=s_.size() || s_[pos_]!=ch) return false;
    ++pos_; return true;
  }
  bool token(std::string& out) {
    if (!take('"')) return false;
    const auto begin=pos_;
    while (pos_<s_.size() && s_[pos_]!='"') {
      const unsigned char c=static_cast<unsigned char>(s_[pos_++]);
      if (!((c>='0' && c<='9') || (c>='A' && c<='Z') ||
            (c>='a' && c<='z') || c=='-' || c=='_')) return false;
      if (pos_-begin>128) return false;
    }
    if (!take('"') || pos_-begin<=1) return false;
    out=s_.substr(begin,pos_-begin-1);
    return true;
  }
  bool unsigned_integer(std::uint64_t& out) {
    if (pos_>=s_.size() || s_[pos_]<'0' || s_[pos_]>'9') return false;
    if (s_[pos_]=='0' && pos_+1<s_.size() &&
        s_[pos_+1]>='0' && s_[pos_+1]<='9') return false;
    std::uint64_t v=0;
    do {
      const auto digit=static_cast<unsigned>(s_[pos_]-'0');
      if (v>(std::numeric_limits<std::uint64_t>::max()-digit)/10) return false;
      v=v*10+digit;
      ++pos_;
    } while (pos_<s_.size() && s_[pos_]>='0' && s_[pos_]<='9');
    out=v; return true;
  }
  bool boolean(bool& v) {
    if (eat("true")) {v=true;return true;}
    if (eat("false")){v=false;return true;}
    return false;
  }
  bool number(double& out) {
    const std::size_t begin=pos_;
    take('-');
    if (pos_>=s_.size()) return false;
    if (take('0')) {
      if (pos_<s_.size() && s_[pos_]>='0' && s_[pos_]<='9') return false;
    } else {
      if (s_[pos_]<'1' || s_[pos_]>'9') return false;
      do {++pos_;} while (pos_<s_.size() && s_[pos_]>='0' && s_[pos_]<='9');
    }
    if (take('.')) {
      if (pos_>=s_.size() || s_[pos_]<'0' || s_[pos_]>'9') return false;
      do {++pos_;} while (pos_<s_.size() && s_[pos_]>='0' && s_[pos_]<='9');
    }
    if (take('e') || take('E')) {
      if (!take('+')) take('-');
      if (pos_>=s_.size() || s_[pos_]<'0' || s_[pos_]>'9') return false;
      do {++pos_;} while (pos_<s_.size() && s_[pos_]>='0' && s_[pos_]<='9');
    }
    if (pos_-begin>64) return false;
    std::istringstream in(s_.substr(begin,pos_-begin));
    in.imbue(std::locale::classic());
    in>>out;
    return !in.fail() && in.peek()==std::char_traits<char>::eof() &&
           std::isfinite(out);
  }
  bool signed_integer(int& out) {
    const bool neg=take('-');
    std::uint64_t magnitude=0;
    if (!unsigned_integer(magnitude)) return false;
    constexpr auto max=static_cast<std::uint64_t>(std::numeric_limits<int>::max());
    if (magnitude>max+(neg?1U:0U)) return false;
    if (neg && magnitude==max+1U) out=std::numeric_limits<int>::min();
    else out=neg?-static_cast<int>(magnitude):static_cast<int>(magnitude);
    return true;
  }
  bool points(std::vector<Point2D>& output,std::size_t capacity) {
    if (!take('[')) return false;
    if (take(']')) return true;
    for (;;) {
      if (output.size()>=capacity || !take('[')) return false;
      double x=0,y=0;
      if (!number(x) || !take(',') || !number(y) || !take(']')) return false;
      output.push_back({x,y});
      if (take(']')) return true;
      if (!take(',')) return false;
    }
  }
  bool integers(std::vector<int>& output,std::size_t capacity) {
    if (!take('[')) return false;
    if (take(']')) return true;
    for (;;) {
      if (output.size()>=capacity) return false;
      int n=0;
      if (!signed_integer(n)) return false;
      output.push_back(n);
      if (take(']')) return true;
      if (!take(',')) return false;
    }
  }
  const std::string& s_;
  std::size_t pos_{0};
};

} // namespace draft_audit_lab_detail

enum class DraftSnapshotAuditLabState {
  access_denied, storage_error, pending_review, recorded_denial,
  inconsistent_snapshot, consistent_preview, consistent_ready
};

struct DraftSnapshotAuditLabResult {
  DraftSnapshotAuditLabState state{DraftSnapshotAuditLabState::storage_error};
  std::string mission_run_id{};
  std::uint64_t revision{0};
  std::size_t navigation_count{0},marking_count{0};
  std::string snapshot_signature{};
  bool snapshot_consistency_verified{false}; // NOT authentication
  bool can_restore{false};
  bool can_start_or_resume{false};
  bool hardware_command_emitted{false};
};

class NativeOfflineDraftSnapshotAuditLab final {
 public:
  explicit NativeOfflineDraftSnapshotAuditLab(NativeDraftSqliteMutationJournalLab& j)
      : journal_(j),reconciliation_(j) {}

  DraftSnapshotAuditLabResult audit(const DraftReconciliationLabIdentity& identity,
                                   const DraftJournalLabKey& key,
                                   const std::string& original_request) {
    DraftSnapshotAuditLabResult result;
    const auto recon=reconciliation_.inspect(identity,key,original_request);
    switch (recon.state) {
      case DraftReconciliationLabState::denied_access:
        result.state=DraftSnapshotAuditLabState::access_denied;return result;
      case DraftReconciliationLabState::pending_operator_review:
        result.state=DraftSnapshotAuditLabState::pending_review;return result;
      case DraftReconciliationLabState::recorded_denial:
        result.state=DraftSnapshotAuditLabState::recorded_denial;return result;
      case DraftReconciliationLabState::recorded_preview:
      case DraftReconciliationLabState::recorded_ready:break;
      default:return result;
    }
    DraftJournalLabEntry entry;
    if (!journal_.inspect(key,entry)) return result;
    result.state=DraftSnapshotAuditLabState::inconsistent_snapshot;
    if (entry.status!="COMMITTED" ||
        entry.mission_run_id!=recon.mission_run_id ||
        entry.sequence!=recon.sequence ||
        entry.canonical_request_json!=original_request) return result;
    draft_audit_lab_detail::StrictSnapshotDecoder::Parsed parsed;
    if (!draft_audit_lab_detail::StrictSnapshotDecoder(
          entry.decision.complete_snapshot_json).decode(parsed)) return result;
    if (parsed.operator_session!=key.operator_session_id ||
        parsed.run!=entry.mission_run_id ||
        parsed.loaded!=(entry.decision.stage=="READY") ||
        (parsed.loaded && entry.route!=kBackendMissionLoadPath) ||
        (!parsed.loaded && entry.route==kBackendMissionLoadPath)) return result;
    // Re-encode with the actual H5A serializer. A byte-for-byte match catches
    // JSON ambiguity, noncanonical values and unexpected extra fields.
    BackendMissionUploadRequest request;
    request.path=entry.route;
    request.mission_run_id=parsed.run;
    request.principal.operator_session_id=parsed.operator_session;
    BackendMissionUploadResponse response;
    response.http_status=200;
    response.revision=parsed.revision;
    response.signature=parsed.snapshot.signature;
    response.marking_count=parsed.snapshot.marking_points.size();
    response.navigation_count=parsed.snapshot.navigation_points.size();
    response.draft_ready=true;
    response.loaded=parsed.loaded;
    std::string encoded;
    if (!journaled_upload_lab_detail::json_snapshot(
            response,request,&parsed.snapshot,parsed.mode,encoded) ||
        encoded!=entry.decision.complete_snapshot_json) return result;
    result.state=parsed.loaded?DraftSnapshotAuditLabState::consistent_ready:
                               DraftSnapshotAuditLabState::consistent_preview;
    result.mission_run_id=parsed.run;
    result.revision=parsed.revision;
    result.navigation_count=parsed.snapshot.navigation_points.size();
    result.marking_count=parsed.snapshot.marking_points.size();
    result.snapshot_signature=parsed.snapshot.signature;
    result.snapshot_consistency_verified=true;
    // Intentionally no export of geometry or permission to restore/drive.
    return result;
  }
 private:
  NativeDraftSqliteMutationJournalLab& journal_;
  NativeOfflineDraftReconciliationReadLab reconciliation_;
};
} // namespace dyx::rpp
