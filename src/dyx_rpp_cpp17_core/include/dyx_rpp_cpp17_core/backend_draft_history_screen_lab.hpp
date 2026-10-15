#pragma once
// DYX Autonomous H5D: strictly read-only C++17 SQLite mutation-history
// screening. This is an OFFLINE laboratory component, NOT mission restoration,
// trusted service authentication, or permission to arm/start/drive/spray.
// We deliberately do not return a restore-capable snapshot or mutate a backend.
// A caller-provided verified identity is an assertion, not authentication.

#include "dyx_rpp_cpp17_core/backend_draft_snapshot_audit_lab.hpp"
#include <sqlite3.h>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace dyx::rpp {
enum class DraftHistoryScreenLabState {
  access_denied, storage_error, no_saved_mutation, inconsistent_history,
  pending_operator_review, denial_operator_review,
  consistent_preview_history, ready_operator_review
};
struct DraftHistoryScreenLabResult {
  DraftHistoryScreenLabState state{DraftHistoryScreenLabState::storage_error};
  std::size_t committed_count{0};
  std::string mission_run_id{},signature{};
  std::uint64_t revision{0};
  std::size_t navigation_count{0},marking_count{0};
  bool history_consistency_verified{false}; // NOT publisher authentication
  bool can_restore{false};
  bool can_start_or_resume{false};
  bool hardware_command_emitted{false};
};
class NativeOfflineDraftHistoryScreenLab final {
 public:
  explicit NativeOfflineDraftHistoryScreenLab(std::string existing_file)
      : file_(std::move(existing_file)) {}

  DraftHistoryScreenLabResult inspect(const DraftReconciliationLabIdentity& identity) const {
    using namespace draft_journal_lab_detail;
    DraftHistoryScreenLabResult result;
    if (!identity.authenticated || !identity.can_read_mission ||
        !identity.fresh_session_evidence ||
        !token(identity.operator_session_id) || identity.session_epoch==0 ||
        !serializable(identity.session_epoch)) {
      result.state=DraftHistoryScreenLabState::access_denied;
      return result;
    }
    if (file_.empty() || file_==":memory:" || file_.rfind("file:",0)==0)
      return result;
    sqlite3* db=nullptr;
    if (sqlite3_open_v2(file_.c_str(),&db,
        SQLITE_OPEN_READONLY|SQLITE_OPEN_FULLMUTEX,nullptr)!=SQLITE_OK) {
      if (db) sqlite3_close_v2(db);
      return result;
    }
    struct Close { sqlite3* db; ~Close(){sqlite3_close_v2(db);} } close{db};
    sqlite3_busy_timeout(db,0);
    if (!exec(db,"PRAGMA query_only=ON") ||
        !pragma_check(db,"PRAGMA journal_mode","wal") ||
        !pragma_check(db,"PRAGMA quick_check(1)","ok") ||
        !journal_schema_ok(db) || !exec(db,"BEGIN")) return result;
    struct Rollback {sqlite3* db;~Rollback(){sqlite3_exec(db,"ROLLBACK",nullptr,nullptr,nullptr);} } rollback{db};
    Fence fence;
    if (!read_fence(db,fence) || fence.session!=identity.operator_session_id ||
        fence.epoch!=identity.session_epoch) {
      result.state=DraftHistoryScreenLabState::access_denied;
      return result;
    }
    Statement st(db,"SELECT nonce,session_id,epoch,mission_run_id,sequence,route,"
       "request_json,status,http_status,response_json,snapshot_json,stage "
       "FROM dyx_mission_mutations ORDER BY nonce");
    if (!st) return result;
    std::uint64_t last_nonce=0,last_sequence=0,last_revision=0;
    std::string last_signature{},last_run{};
    MissionMode last_mode=MissionMode::automatic;
    bool have_preview=false,have_ready=false;
    std::size_t seen=0;
    int step=SQLITE_ROW;
    while ((step=sqlite3_step(st.st))==SQLITE_ROW) {
      if (++seen>1024 || have_ready ||
          sqlite3_column_type(st.st,0)!=SQLITE_INTEGER ||
          sqlite3_column_type(st.st,2)!=SQLITE_INTEGER ||
          sqlite3_column_type(st.st,4)!=SQLITE_INTEGER ||
          sqlite3_column_type(st.st,8)!=SQLITE_INTEGER) {
        result.state=DraftHistoryScreenLabState::inconsistent_history;return result;
      }
      const auto nonce_raw=sqlite3_column_int64(st.st,0);
      const auto epoch_raw=sqlite3_column_int64(st.st,2);
      const auto sequence_raw=sqlite3_column_int64(st.st,4);
      if (nonce_raw<=0 || epoch_raw<=0 || sequence_raw<=0) {
        result.state=DraftHistoryScreenLabState::inconsistent_history;return result;
      }
      const auto nonce=static_cast<std::uint64_t>(nonce_raw);
      const auto sequence=static_cast<std::uint64_t>(sequence_raw);
      std::string session,run,route,request,status,response,snapshot_json,stage;
      if (nonce<=last_nonce || sequence<=last_sequence ||
          static_cast<std::uint64_t>(epoch_raw)!=identity.session_epoch ||
          !text_col(st.st,1,session,128) || !text_col(st.st,3,run,128) ||
          !text_col(st.st,5,route,64) || !text_col(st.st,6,request,262144) ||
          !text_col(st.st,7,status,16) || !text_col(st.st,9,response,4096) ||
          !text_col(st.st,10,snapshot_json,262144) || !text_col(st.st,11,stage,16) ||
          session!=identity.operator_session_id || !token(run) ||
          !route_valid(route) || (have_preview && run!=last_run)) {
        result.state=DraftHistoryScreenLabState::inconsistent_history;return result;
      }
      last_nonce=nonce;last_sequence=sequence;
      if (status=="PENDING") {
        result.state=DraftHistoryScreenLabState::pending_operator_review;return result;
      }
      if (status=="DENIED") {
        result.state=DraftHistoryScreenLabState::denial_operator_review;return result;
      }
      if (status!="COMMITTED" || sqlite3_column_int(st.st,8)!=200 ||
          !bounded_json(response,4096) || !bounded_json(snapshot_json,262144)) {
        result.state=DraftHistoryScreenLabState::inconsistent_history;return result;
      }
      const bool load=route==kBackendMissionLoadPath;
      if (stage!=(load?"READY":"PREVIEW") ||
          (route==kBackendMissionUploadPath && have_preview) ||
          (route!=kBackendMissionUploadPath && !have_preview)) {
        result.state=DraftHistoryScreenLabState::inconsistent_history;return result;
      }
      draft_audit_lab_detail::StrictSnapshotDecoder::Parsed decoded;
      if (!draft_audit_lab_detail::StrictSnapshotDecoder(snapshot_json).decode(decoded) ||
          decoded.operator_session!=session || decoded.run!=run ||
          decoded.loaded!=load || decoded.revision==0) {
        result.state=DraftHistoryScreenLabState::inconsistent_history;return result;
      }
      // The persisted bytes must exactly match H5A's canonical snapshot
      // serialization, not merely contain a plausible signature.
      BackendMissionUploadRequest reconstruct;
      reconstruct.path=route;reconstruct.mission_run_id=run;
      reconstruct.principal.operator_session_id=session;
      BackendMissionUploadResponse reply;
      reply.http_status=200;reply.revision=decoded.revision;
      reply.signature=decoded.snapshot.signature;
      reply.marking_count=decoded.snapshot.marking_points.size();
      reply.navigation_count=decoded.snapshot.navigation_points.size();
      reply.draft_ready=true;reply.loaded=load;
      std::string recoded;
      if (!journaled_upload_lab_detail::json_snapshot(reply,reconstruct,
          &decoded.snapshot,decoded.mode,recoded) || recoded!=snapshot_json ||
          !request_binding(request,run,sequence,route,have_preview,last_revision,last_signature)) {
        result.state=DraftHistoryScreenLabState::inconsistent_history;return result;
      }
      if (!have_preview) {
        if (decoded.revision!=1 || load) {
          result.state=DraftHistoryScreenLabState::inconsistent_history;return result;
        }
      } else if (load) {
        if (decoded.revision!=last_revision ||
            decoded.snapshot.signature!=last_signature || decoded.mode!=last_mode) {
          result.state=DraftHistoryScreenLabState::inconsistent_history;return result;
        }
      } else if (decoded.revision!=last_revision+1 ||
                 decoded.mode!=last_mode ||
                 last_revision==std::numeric_limits<std::uint64_t>::max()) {
        result.state=DraftHistoryScreenLabState::inconsistent_history;return result;
      }
      have_preview=true;have_ready=load;last_run=run;
      last_revision=decoded.revision;
      last_signature=decoded.snapshot.signature;last_mode=decoded.mode;
      result.committed_count=seen;
      result.mission_run_id=run;result.revision=last_revision;
      result.signature=last_signature;
      result.navigation_count=decoded.snapshot.navigation_points.size();
      result.marking_count=decoded.snapshot.marking_points.size();
    }
    if (step!=SQLITE_DONE) return {};
    if (fence.nonce!=last_nonce) {
      result.state=DraftHistoryScreenLabState::inconsistent_history;
      result.history_consistency_verified=false;return result;
    }
    if (!have_preview) {
      result.state=DraftHistoryScreenLabState::no_saved_mutation;return result;
    }
    result.state=have_ready?DraftHistoryScreenLabState::ready_operator_review:
                            DraftHistoryScreenLabState::consistent_preview_history;
    result.history_consistency_verified=true;
    return result;
  }
 private:
  static bool route_valid(const std::string& route) {
    return draft_journal_lab_detail::route(route);
  }
  static bool request_binding(const std::string& json,const std::string& run,
      std::uint64_t sequence,const std::string& route,bool had_preview,
      std::uint64_t last_revision,const std::string& last_signature) {
    const std::string prefix="{\"mission_run_id\":\""+run+"\",\"sequence\":"+
        std::to_string(sequence);
    if (json.compare(0,prefix.size(),prefix)!=0 ||
        json.size()<=prefix.size()) return false;
    const bool upload=route==kBackendMissionUploadPath;
    if (upload) return !had_preview && json.compare(prefix.size(),19,
        ",\"marking_points\":[")==0;
    if (!had_preview || last_revision==0 ||
        !valid_path_signature(last_signature)) return false;
    const std::string ref=",\"expected_revision\":"+std::to_string(last_revision)+
        ",\"expected_signature\":\""+last_signature+"\"";
    if (json.compare(prefix.size(),ref.size(),ref)!=0) return false;
    const auto tail=prefix.size()+ref.size();
    return route==kBackendMissionLoadPath
        ? json.size()==tail+1 && json[tail]=='}'
        : json.compare(tail,19,",\"marking_points\":[")==0;
  }
  std::string file_;
};
} // namespace dyx::rpp
