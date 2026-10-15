// DYX Autonomous Phase 2H4Y: offline SQLite + actual H4W fence interface.
// Not a TLS server, mission journal, DDS reader or actuator path.
#include "dyx_rpp_cpp17_core/backend_draft_sqlite_store_lab.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace dyx::rpp;
namespace fs = std::filesystem;

struct TemporaryReplayDatabase final {
  fs::path dir;
  fs::path path;
  explicit TemporaryReplayDatabase(const std::string& name) {
    const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
    dir = fs::temp_directory_path() /
        ("dyx_h4y_" + name + "_" + std::to_string(tick));
    assert(fs::create_directory(dir));
    path = dir / "replay.sqlite";
  }
  ~TemporaryReplayDatabase() {
    std::error_code ignored;
    fs::remove_all(dir, ignored);
  }
  TemporaryReplayDatabase(const TemporaryReplayDatabase&)=delete;
  TemporaryReplayDatabase& operator=(const TemporaryReplayDatabase&)=delete;
};

inline DraftRestartLabRecord initial_record() {
  return {"SESSION_H4R_OPERATOR", 3, 0, 0.0, 1};
}
inline void create_provisioned_database(const TemporaryReplayDatabase& temp) {
  assert(NativeDraftSqliteStoreLab::provision_new(
      temp.path.string(), initial_record()));
}

// The fixture acts only as simulated privileged evidence. It does NOT
// authenticate a user, lease, transport or PX4 publisher in production.
struct OfflineSource final : DraftBoundaryLabTrustedSource {
  bool clock_ok{true};
  bool session_ok{true};
  bool px4_ok{true};
  double seconds{10.0};
  DraftBoundaryLabSession session{};
  DraftBoundaryLabPx4 px4{};
  int px4_reads{0};

  void next(double now, std::uint64_t nonce) {
    seconds = now;
    session.verified_transport = true;
    session.principal = {true, true, true, "SESSION_H4R_OPERATOR"};
    session.session_epoch = 3;
    session.ingress_nonce = nonce;
    session.verified_auth_sec = now - 0.05;
    px4 = {true, true, now - 0.02};
  }
  bool monotonic_receive_time(double& out) override {
    if (!clock_ok) return false;
    out = seconds;
    return true;
  }
  bool verified_session(DraftBoundaryLabSession& out) override {
    if (!session_ok) return false;
    out = session;
    return true;
  }
  bool independently_observed_px4(DraftBoundaryLabPx4& out) override {
    ++px4_reads;
    if (!px4_ok) return false;
    out = px4;
    return true;
  }
};

// Invoke precisely the two interfaces H4V calls for each verified request.
// This doesn't call a mission backend: no network or actuator behavior.
inline bool reserve(OfflineSource& source,
                    NativeOfflineDraftRestartFenceSource& fence,
                    double now, std::uint64_t nonce) {
  source.next(now, nonce);
  double observed = -1.0;
  if (!fence.monotonic_receive_time(observed)) return false;
  if (observed != now) return false;
  DraftBoundaryLabSession evidence;
  return fence.verified_session(evidence);
}
inline DraftRestartLabRecord read_record(NativeDraftSqliteStoreLab& store) {
  DraftRestartLabRecord out;
  assert(store.load_existing(out));
  return out;
}

void run_reopen() {
  TemporaryReplayDatabase db("reopen");
  create_provisioned_database(db);
  {
    NativeDraftSqliteStoreLab store(db.path.string());
    OfflineSource source;
    NativeOfflineDraftRestartFenceSource fence(source, store);
    assert(reserve(source, fence, 10.02, 1));
    assert(read_record(store).last_ingress_nonce == 1);
  }
  // Fresh SQLite connection AND reconstructed source/fence objects.
  {
    NativeDraftSqliteStoreLab store(db.path.string());
    OfflineSource source;
    NativeOfflineDraftRestartFenceSource fence(source, store);
    assert(!reserve(source, fence, 10.12, 1));
    assert(reserve(source, fence, 10.13, 2));
    const auto persisted = read_record(store);
    assert(persisted.last_ingress_nonce == 2 && persisted.generation == 3);
    assert(persisted.last_received_sec == 10.13);
  }
}

void run_binding() {
  TemporaryReplayDatabase db("binding");
  create_provisioned_database(db);
  NativeDraftSqliteStoreLab store(db.path.string());
  OfflineSource source;
  NativeOfflineDraftRestartFenceSource fence(source, store);
  assert(reserve(source, fence, 20.02, 1));
  source.next(20.12, 2);
  source.session.session_epoch = 4;
  double now = 0.0;
  assert(fence.monotonic_receive_time(now));
  DraftBoundaryLabSession session;
  assert(!fence.verified_session(session));
  source.next(20.12, 2);
  source.session.principal.operator_session_id = "OTHER_OPERATOR";
  assert(fence.monotonic_receive_time(now));
  assert(!fence.verified_session(session));
  source.next(20.12, 2);
  source.session.verified_transport = false;
  assert(fence.monotonic_receive_time(now));
  assert(!fence.verified_session(session));
  assert(!reserve(source, fence, 19.0, 2)); // rollback on old clock
  assert(read_record(store).last_ingress_nonce == 1);
  assert(reserve(source, fence, 20.2, 2));
  assert(read_record(store).last_ingress_nonce == 2);
}

void run_fault() {
  TemporaryReplayDatabase db("fault");
  bool missing = false;
  try { NativeDraftSqliteStoreLab unprovisioned(db.path.string()); }
  catch (const std::exception&) { missing = true; }
  assert(missing);
  create_provisioned_database(db);
  {
    NativeDraftSqliteStoreLab store(db.path.string());
    OfflineSource source;
    NativeOfflineDraftRestartFenceSource fence(source, store);
    source.next(30.02, 1);
    source.clock_ok = false;
    double now = 0.0;
    assert(!fence.monotonic_receive_time(now));
    source.clock_ok = true;
    source.session_ok = false;
    assert(fence.monotonic_receive_time(now));
    DraftBoundaryLabSession session;
    assert(!fence.verified_session(session));
    assert(read_record(store).last_ingress_nonce == 0);
    source.session_ok = true;
    assert(reserve(source, fence, 30.02, 1));
  }
  {
    // A schema-compatible but missing row must not be initialized silently.
    sqlite3* raw = nullptr;
    assert(sqlite3_open_v2(db.path.string().c_str(), &raw,
        SQLITE_OPEN_READWRITE, nullptr) == SQLITE_OK);
    assert(sqlite3_exec(raw, "DELETE FROM replay_fence", nullptr, nullptr,
        nullptr) == SQLITE_OK);
    assert(sqlite3_close_v2(raw) == SQLITE_OK);
  }
  NativeDraftSqliteStoreLab reopened(db.path.string());
  OfflineSource source;
  NativeOfflineDraftRestartFenceSource fence(source, reopened);
  assert(!reserve(source, fence, 30.12, 2));
  DraftRestartLabRecord record;
  assert(!reopened.load_existing(record));
}

void run_concurrent() {
  TemporaryReplayDatabase db("concurrent");
  create_provisioned_database(db);
  constexpr int contestants = 8;
  // Open/configure all connections before contention starts. The test is
  // about atomic compare-and-swap, not concurrent PRAGMA initialization.
  std::vector<std::unique_ptr<NativeDraftSqliteStoreLab>> stores;
  for (int i = 0; i < contestants; ++i)
    stores.emplace_back(std::make_unique<NativeDraftSqliteStoreLab>(db.path.string()));
  std::vector<std::thread> workers;
  bool outcomes[contestants]{};
  for (int i = 0; i < contestants; ++i) {
    workers.emplace_back([&,i]() {
      OfflineSource source;
      NativeOfflineDraftRestartFenceSource fence(source, *stores[i]);
      outcomes[i] = reserve(source, fence, 40.02, 1);
    });
  }
  for (auto& worker : workers) worker.join();
  int accepted = 0;
  for (const auto ok : outcomes) if (ok) ++accepted;
  assert(accepted == 1); // One monotonic nonce across eight connections.
  NativeDraftSqliteStoreLab observer(db.path.string());
  const auto record = read_record(observer);
  assert(record.last_ingress_nonce == 1 && record.generation == 2);
}

int main(int argc, char** argv) {
  assert(argc == 2);
  const std::string mode = argv[1];
  if (mode == "reopen") run_reopen();
  else if (mode == "binding") run_binding();
  else if (mode == "fault") run_fault();
  else if (mode == "concurrent") run_concurrent();
  else return 2;
  std::cout << "PASS sqlite H4W fence " << mode << '\n';
}
