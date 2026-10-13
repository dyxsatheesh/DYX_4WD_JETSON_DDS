#include "dyx_rpp_cpp17_core/actuator_write_ahead_journal.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace dyx::rpp;
#define CHECK(x) do { if (!(x)) { std::cerr << "FAIL line " << __LINE__ << ": " #x "\n"; return 1; } } while (false)
template<typename F> bool rejects(F f) { try { f(); } catch (const std::exception &) {return true;}return false; }
int main() {
  namespace fs=std::filesystem;
  auto dir=fs::temp_directory_path()/("dyx_actuator_journal_safety_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(dir);
  auto path=dir/"actuator.journal";
  auto on=encode_px4_actuator_intent(DdsActuatorOwner::manual_spray,DdsActuatorAction::on,3,"operator-7",-0.2,1);
  auto off=encode_px4_actuator_intent(DdsActuatorOwner::manual_spray,DdsActuatorAction::off,4,"operator-7",-0.6,1);
  FileBackedActuatorJournal journal(path);
  CHECK(rejects([&] { journal.persist_before_off(off); }));
  CHECK(rejects([&] {auto x=on;x.params[1]=0.0;journal.persist_before_on(x);}));
  CHECK(rejects([&] {auto x=on;x.owner=static_cast<DdsActuatorOwner>(4);journal.persist_before_on(x);}));
  journal.persist_before_on(on);
  CHECK(rejects([&] { journal.persist_before_on(on); })); // no replay
  CHECK(rejects([&] { journal.complete_off(off,2,true); })); // not staged
  auto other=encode_px4_actuator_intent(DdsActuatorOwner::laser,DdsActuatorAction::on,1,"laser-8",1,2);
  CHECK(rejects([&] { journal.persist_before_on(other); })); // global single-owner lock
  CHECK(rejects([&] {auto x=off;x.identity="different";journal.persist_before_off(x); }));
  auto dispatch=journal.persist_before_off(off);
  CHECK(dispatch==2);
  journal.record_off_response(off,dispatch,false);
  CHECK(rejects([&] {journal.complete_off(off,2,true);})); // rejected OFF not safe
  journal.record_off_response(off,dispatch,true);
  CHECK(rejects([&] {journal.complete_off(off,dispatch,false);})); // ACK alone not safe
  CHECK(rejects([&] {auto x=off;x.local_sequence=5;journal.complete_off(x,dispatch,true);}));
  CHECK(journal.recovery_required());
  CHECK(rejects([&] {journal.complete_off(off,dispatch+1,true);})); // stale/future ticket
  journal.complete_off(off,dispatch,true);
  CHECK(!journal.recovery_required());
  CHECK(rejects([&] {journal.persist_before_on(on);})); // replay survives OFF
  // No writable WAL location -> ON must NOT be authorized.
  auto missing=dir/"missing-parent"/"journal";
  FileBackedActuatorJournal invalid_target(missing);
  CHECK(rejects([&] {invalid_target.persist_before_on(other);}));
  CHECK(!invalid_target.identity_used(other.owner,other.identity));
  // A corrupt WAL is a hard error. Never silently start with empty history.
  {std::ofstream f(dir/"corrupt.journal");f<<"DYX_ACTUATOR_WAL_V2\nACTIVE 1 616263 9 0\n";}
  CHECK(rejects([&] {FileBackedActuatorJournal bad(dir/"corrupt.journal");}));
  fs::remove_all(dir);
  std::cout << "WAL replay/cross-owner/uncertain ACK/fail-closed tests passed\n";
}
