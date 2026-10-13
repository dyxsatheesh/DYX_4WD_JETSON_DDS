#include "dyx_rpp_cpp17_core/actuator_write_ahead_journal.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
using namespace dyx::rpp;
#define CHECK(x) do { if (!(x)) { std::cerr << "FAIL line " << __LINE__ << ": " #x "\n"; return 1; } } while (false)
int main() {
  namespace fs=std::filesystem;
  const auto name="dyx_actuator_journal_restart_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  auto dir=fs::temp_directory_path()/name;fs::create_directories(dir);
  auto path=dir/"actuator.journal";
  auto on=encode_px4_actuator_intent(DdsActuatorOwner::mission_spray,DdsActuatorAction::on,4,"RUN-P0001",-0.2,1);
  // A restarted FSM's local command sequence is allowed to reset to 1.
  auto off=encode_px4_actuator_intent(DdsActuatorOwner::mission_spray,DdsActuatorAction::off,1,"RUN-P0001",-0.6,1);
  {
    FileBackedActuatorJournal journal(path);
    CHECK(!journal.recovery_required());
    journal.persist_before_on(on);
    CHECK(journal.identity_used(on.owner,on.identity));
    CHECK(journal.recovery_required());
  }
  {
    FileBackedActuatorJournal restarted(path);
    CHECK(restarted.recovery_required());
    CHECK(restarted.identity_used(on.owner,on.identity));
    CHECK(restarted.persist_before_off(off)==2);
  }
  {
    FileBackedActuatorJournal restarted(path);
    CHECK(restarted.recovery_required());
    CHECK(restarted.active_lock()->last_sequence==1);
    restarted.record_off_response(off,2,true);
  }
  {
    FileBackedActuatorJournal restarted(path);
    CHECK(restarted.recovery_required() && restarted.active_lock()->off_response_accepted);
    restarted.complete_off(off,2,true); // external independent OFF attestation
  }
  {
    FileBackedActuatorJournal restarted(path);
    CHECK(!restarted.recovery_required());
    CHECK(restarted.identity_used(on.owner,on.identity));
    auto laser=encode_px4_actuator_intent(DdsActuatorOwner::laser,DdsActuatorAction::on,1,"RUN-P0002",1,2);
    restarted.persist_before_on(laser);
    CHECK(restarted.recovery_required());
    CHECK(restarted.used_count()==2);
  }
  fs::remove_all(dir);
  std::cout << "Crash/restart journal / accepted OFF + independent OFF proof passed\n";
}
