#pragma once
// DYX Autonomous Phase 2H5B: strictly offline, read-only lost-reply
// reconciliation policy. Caller-provided verified identity is a test fixture,
// NOT an authenticator. No HTTP listener, DDS, motor, spray, or mission start.
// Persisted snapshots are opaque/UNVERIFIED; never authorize restoration.

#include "dyx_rpp_cpp17_core/backend_draft_sqlite_mutation_journal_lab.hpp"
#include <cstdint>
#include <string>

namespace dyx::rpp {

struct DraftReconciliationLabIdentity {
  bool authenticated{false};
  bool can_read_mission{false};
  bool fresh_session_evidence{false};
  std::string operator_session_id{};
  std::uint64_t session_epoch{0};
};

enum class DraftReconciliationLabState {
  denied_access, no_matching_record, request_binding_conflict,
  pending_operator_review, recorded_denial, recorded_preview,
  recorded_ready, storage_error
};

struct DraftReconciliationLabReply {
  DraftReconciliationLabState state{DraftReconciliationLabState::storage_error};
  int http_status{503};
  // Response fields are ADVISORY ONLY, and never allow continuation/motion.
  std::string mission_run_id{};
  std::uint64_t sequence{0};
  int recorded_http_status{0};
  bool is_verified_snapshot{false};
  bool can_restore_mission{false};
  bool can_start_or_resume{false};
  bool hardware_command_emitted{false};
};

class NativeOfflineDraftReconciliationReadLab final {
 public:
  explicit NativeOfflineDraftReconciliationReadLab(
      NativeDraftSqliteMutationJournalLab& journal) : journal_(journal) {}

  // A privileged service must authenticate independently and supply fresh
  // session evidence. Nothing here accepts a client-claimed principal as proof.
  // The exact original canonical request prevents a nonce being queried as
  // though it belonged to a different command. Never returns raw snapshot JSON.
  DraftReconciliationLabReply inspect(
      const DraftReconciliationLabIdentity& identity,
      const DraftJournalLabKey& key,
      const std::string& original_canonical_request) {
    DraftReconciliationLabReply reply;
    if (!identity.authenticated || !identity.can_read_mission ||
        !identity.fresh_session_evidence ||
        !draft_journal_lab_detail::token(identity.operator_session_id) ||
        identity.operator_session_id != key.operator_session_id ||
        identity.session_epoch == 0 ||
        identity.session_epoch != key.session_epoch) {
      reply.state = DraftReconciliationLabState::denied_access;
      reply.http_status = 403;
      return reply;
    }
    if (!draft_journal_lab_detail::bounded_json(original_canonical_request,262144) ||
        key.ingress_nonce==0) {
      reply.state = DraftReconciliationLabState::request_binding_conflict;
      reply.http_status = 409;
      return reply;
    }
    DraftJournalLabEntry entry;
    if (!journal_.inspect(key,entry)) {
      // Journal `inspect()` does not distinguish absent from I/O/corruption.
      // Deny with a generic 503, never report "safe to retry" or "not found".
      reply.state = DraftReconciliationLabState::storage_error;
      reply.http_status = 503;
      return reply;
    }
    if (entry.canonical_request_json != original_canonical_request ||
        !draft_journal_lab_detail::route(entry.route) ||
        !draft_journal_lab_detail::token(entry.mission_run_id) ||
        entry.sequence == 0) {
      reply.state = DraftReconciliationLabState::request_binding_conflict;
      reply.http_status = 409;
      return reply;
    }
    reply.sequence = entry.sequence;
    reply.mission_run_id = entry.mission_run_id;
    if (entry.status == "PENDING") {
      reply.state = DraftReconciliationLabState::pending_operator_review;
      reply.http_status = 202; // unknown backend outcome: DO NOT RETRY
    } else if (entry.status == "DENIED") {
      reply.state = DraftReconciliationLabState::recorded_denial;
      reply.http_status = 200; // reconciliation read itself was successful
      reply.recorded_http_status = entry.decision.http_status;
    } else if (entry.status == "COMMITTED" &&
               entry.decision.stage == "PREVIEW") {
      reply.state = DraftReconciliationLabState::recorded_preview;
      reply.http_status = 200;
      reply.recorded_http_status = entry.decision.http_status;
    } else if (entry.status == "COMMITTED" &&
               entry.decision.stage == "READY") {
      reply.state = DraftReconciliationLabState::recorded_ready;
      reply.http_status = 200;
      reply.recorded_http_status = entry.decision.http_status;
    } else {
      reply = {};
    }
    return reply;
  }
 private:
  NativeDraftSqliteMutationJournalLab& journal_;
};

} // namespace dyx::rpp
