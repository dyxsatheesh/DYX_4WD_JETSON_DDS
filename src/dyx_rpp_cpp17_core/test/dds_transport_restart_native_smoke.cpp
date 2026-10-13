#include "dyx_rpp_cpp17_core/dds_actuator_transport.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
using namespace dyx::rpp;
#define CHECK(x) do{if(!(x)){std::cerr<<"FAIL "<<__LINE__<<" " #x "\n";return 1;}}while(false)
int main(){
  const auto dir=std::filesystem::temp_directory_path() / ("dyx_dds_transport_c_"+
  std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(dir);
  const auto file=dir/"wal";
  const auto on=encode_px4_actuator_intent(DdsActuatorOwner::manual_spray,
    DdsActuatorAction::on,1,"manual-id-789",-0.2,1);
  {
    FileBackedActuatorJournal wal(file);NativeDdsActuatorTransport t(wal);
    CHECK(t.begin_on(on,1.0,true,true,true,true).has_value());
    CHECK(wal.recovery_required());
  }
  {
    FileBackedActuatorJournal wal(file);NativeDdsActuatorTransport t(wal);
    CHECK(wal.recovery_required());
    CHECK(!t.begin_on(on,2,true,true,true,true));
    const auto off=encode_px4_actuator_intent(DdsActuatorOwner::manual_spray,
      DdsActuatorAction::off,1,"manual-id-789",-0.6,1); // sequence reset OK after restart
    CHECK(!t.begin_off(off,2.0,false));
    auto r=t.begin_off(off,2.1,true);
    CHECK(r.has_value() && r->durable_generation==2);
    CHECK(!t.observe_unkeyed_px4_ack({187,0,1,1,true,1000}).changed);
    CHECK(wal.recovery_required());
    const UniquelyCorrelatedReceipt proof{r->intent.owner,r->intent.action,
      r->intent.local_sequence,r->durable_generation,r->intent.identity,2.2,true,true};
    CHECK(t.confirm_unique_receipt(proof,2.2).changed);
    CHECK(wal.recovery_required());
    CHECK(t.prove_physical_off(*r,true).changed);
    CHECK(!wal.recovery_required());
  }
  {
    FileBackedActuatorJournal wal(file);NativeDdsActuatorTransport t(wal);
    CHECK(wal.identity_used(DdsActuatorOwner::manual_spray,"manual-id-789"));
    CHECK(!t.begin_on(on,3,true,true,true,true));
  }
  std::error_code ec;std::filesystem::remove_all(dir,ec);
  std::cout<<"journal restart OFF-only recovery, replay prevention passed\n";
}
