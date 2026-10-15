// Phase 2H4X standalone SQLite persistence policy checks. No PX4/motion.
#include "dyx_rpp_cpp17_core/backend_draft_sqlite_store_lab.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

using namespace dyx::rpp;
namespace fs=std::filesystem;

struct TempDb final {
  fs::path root;
  fs::path db;
  explicit TempDb(const std::string& mode) {
    const auto ticks=std::chrono::steady_clock::now().time_since_epoch().count();
    root=fs::temp_directory_path()/("dyx_h4x_"+mode+"_"+std::to_string(ticks));
    assert(fs::create_directory(root));
    db=root/"replay.sqlite";
  }
  ~TempDb() { std::error_code ignored; fs::remove_all(root,ignored); }
};

inline DraftRestartLabRecord original() {
  return {"SESSION_H4R_OPERATOR",3,0,0.0,1};
}
inline DraftRestartLabRecord advance(const DraftRestartLabRecord& old,
                                       std::uint64_t nonce,double received) {
  auto next=old;
  next.last_ingress_nonce=nonce;
  next.last_received_sec=received;
  ++next.generation;
  return next;
}
inline void provision(const TempDb& tmp) {
  assert(NativeDraftSqliteStoreLab::provision_new(tmp.db.string(),original()));
}
void reopen_test() {
  TempDb tmp("reopen"); provision(tmp);
  assert(!NativeDraftSqliteStoreLab::provision_new(tmp.db.string(),original()));
  {
    NativeDraftSqliteStoreLab store(tmp.db.string());
    DraftRestartLabRecord current;
    assert(store.load_existing(current));
    assert(draft_restart_lab_record_equal(current,original()));
    assert(store.compare_exchange_durable(current,advance(current,1,100.0)));
  }
  {
    NativeDraftSqliteStoreLab store(tmp.db.string());
    DraftRestartLabRecord current;
    assert(store.load_existing(current));
    assert(current.last_ingress_nonce==1 && current.generation==2);
    assert(!store.compare_exchange_durable(original(),advance(original(),1,100.0)));
  }
}
void conflict_test() {
  TempDb tmp("conflict"); provision(tmp);
  NativeDraftSqliteStoreLab first(tmp.db.string()), second(tmp.db.string());
  DraftRestartLabRecord a,b;
  assert(first.load_existing(a) && second.load_existing(b));
  assert(first.compare_exchange_durable(a,advance(a,5,10.0)));
  assert(!second.compare_exchange_durable(b,advance(b,6,11.0)));
  assert(second.load_existing(b) && b.last_ingress_nonce==5);
  assert(second.compare_exchange_durable(b,advance(b,6,11.0)));
  assert(first.load_existing(a) && a.last_ingress_nonce==6 && a.generation==3);
}
void fault_test() {
  TempDb tmp("faults");
  bool missing_threw=false;
  try { NativeDraftSqliteStoreLab missing(tmp.db.string()); }
  catch (const std::exception&) { missing_threw=true; }
  assert(missing_threw && !fs::exists(tmp.db));
  auto invalid=original(); invalid.generation=0;
  assert(!NativeDraftSqliteStoreLab::provision_new(tmp.db.string(),invalid));
  provision(tmp);
  {
    NativeDraftSqliteStoreLab store(tmp.db.string());
    DraftRestartLabRecord old;
    assert(store.load_existing(old));
    auto changed=advance(old,1,20.0);
    changed.session_epoch=4;
    assert(!store.compare_exchange_durable(old,changed));
    changed=advance(old,1,-1.0);
    assert(!store.compare_exchange_durable(old,changed));
    changed=advance(old,1,0.0);
    changed.last_ingress_nonce=0;
    assert(!store.compare_exchange_durable(old,changed));
    assert(store.load_existing(old) && old.last_ingress_nonce==0);
  }
  {
    sqlite3* raw=nullptr;
    assert(sqlite3_open_v2(tmp.db.string().c_str(),&raw,SQLITE_OPEN_READWRITE,nullptr)==SQLITE_OK);
    assert(sqlite3_exec(raw,"DELETE FROM replay_fence",nullptr,nullptr,nullptr)==SQLITE_OK);
    sqlite3_close_v2(raw);
  }
  NativeDraftSqliteStoreLab store(tmp.db.string());
  DraftRestartLabRecord out;
  assert(!store.load_existing(out)); // Never re-provision missing row.
  assert(!store.compare_exchange_durable(original(),advance(original(),1,5.0)));
  {
    TempDb corrupt("corrupt");
    std::ofstream f(corrupt.db,std::ios::binary);
    f << "corrupted SQLite file";
    f.close();
    bool threw=false;
    try { NativeDraftSqliteStoreLab broken(corrupt.db.string()); }
    catch(const std::exception&) { threw=true; }
    assert(threw);
  }
}
void recovery_test() {
  TempDb tmp("recovery"); provision(tmp);
  // Simulated reply loss: committed reservation survives destruction/reopen.
  {
    NativeDraftSqliteStoreLab first(tmp.db.string());
    DraftRestartLabRecord previous;
    assert(first.load_existing(previous));
    assert(first.compare_exchange_durable(previous,advance(previous,2,42.12)));
  }
  {
    NativeDraftSqliteStoreLab next_process(tmp.db.string());
    DraftRestartLabRecord persisted;
    assert(next_process.load_existing(persisted));
    assert(persisted.last_ingress_nonce==2);
    assert(!next_process.compare_exchange_durable(persisted,advance(persisted,2,42.2)));
    assert(!next_process.compare_exchange_durable(persisted,advance(persisted,3,40.0)));
    // Reservation 3 is permitted; this is NOT permission to repeat the mission.
    // Backend state must be reconciled through authenticated GET /draft.
    assert(next_process.compare_exchange_durable(persisted,advance(persisted,3,42.3)));
  }
}
int main(int argc,char** argv) {
  assert(argc==2);
  const std::string mode=argv[1];
  if (mode=="reopen") reopen_test();
  else if (mode=="conflict") conflict_test();
  else if (mode=="faults") fault_test();
  else if (mode=="recovery") recovery_test();
  else return 2;
  std::cout << "PASS " << mode << '\n';
}
