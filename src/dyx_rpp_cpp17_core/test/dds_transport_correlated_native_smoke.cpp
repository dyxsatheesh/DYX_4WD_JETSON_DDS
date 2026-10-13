#include "dyx_rpp_cpp17_core/dds_actuator_transport.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
using namespace dyx::rpp;
#define CHECK(x) do{if(!(x)){std::cerr<<"FAIL "<<__LINE__<<" " #x "\n";return 1;}}while(false)
struct TempJournal {
  std::filesystem::path dir;
  TempJournal(){dir=std::filesystem::temp_directory_path()/(
    "dyx_dds_transport_a_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(dir);}
  ~TempJournal(){std::error_code ec;std::filesystem::remove_all(dir,ec);}
};
static UniquelyCorrelatedReceipt receipt(const DdsDispatchTicket& t,double now,bool ok=true){
  return {t.intent.owner,t.intent.action,t.intent.local_sequence,
    t.durable_generation,t.intent.identity,now,ok,true};
}
int main(){
  TempJournal tmp;
  FileBackedActuatorJournal wal(tmp.dir/"transaction.wal");
  NativeDdsActuatorTransport dds(wal);
  const auto on=encode_px4_actuator_intent(DdsActuatorOwner::mission_spray,
    DdsActuatorAction::on,2,"run1|P0001",-0.2,1);
  CHECK(!dds.begin_on(on,1.0,false,true,true,true));
  CHECK(!dds.begin_on(on,1.0,true,false,true,true));
  CHECK(!dds.begin_on(on,1.0,true,true,false,true));
  CHECK(!dds.begin_on(on,1.0,true,true,true,false));
  const auto ticket=dds.begin_on(on,1.0,true,true,true,true);
  CHECK(ticket.has_value() && wal.recovery_required());
  CHECK(!dds.begin_on(on,1.1,true,true,true,true));
  const ObservedPx4Ack unkeyed{187,0,1,1,true,123456};
  auto observation=dds.observe_unkeyed_px4_ack(unkeyed);
  CHECK(!observation.changed && !observation.publish_authorized);
  CHECK(dds.state()==TransportState::awaiting_receipt);
  auto fake=receipt(*ticket,1.1);fake.correlation_proven=false;
  CHECK(!dds.confirm_unique_receipt(fake,1.1).changed);
  CHECK(dds.confirm_unique_receipt(receipt(*ticket,1.2),1.2).changed);
  CHECK(dds.state()==TransportState::positive_receipt && wal.recovery_required());
  const auto off=encode_px4_actuator_intent(DdsActuatorOwner::mission_spray,
    DdsActuatorAction::off,3,"run1|P0001",-0.6,1);
  const auto rel=dds.begin_off(off,1.3,true);
  CHECK(rel.has_value() && rel->durable_generation>ticket->durable_generation);
  CHECK(!dds.observe_unkeyed_px4_ack(unkeyed).changed);
  CHECK(!dds.confirm_unique_receipt(receipt(*ticket,1.35),1.35).changed);
  CHECK(dds.confirm_unique_receipt(receipt(*rel,1.4),1.4).changed);
  CHECK(dds.state()==TransportState::awaiting_physical_off_proof);
  CHECK(wal.recovery_required() && wal.active_lock()->off_response_accepted);
  CHECK(!dds.prove_physical_off(*rel,false).changed);
  CHECK(dds.prove_physical_off(*rel,true).changed);
  CHECK(dds.state()==TransportState::idle && !wal.recovery_required());
  CHECK(wal.identity_used(DdsActuatorOwner::mission_spray,"run1|P0001"));
  CHECK(!dds.begin_on(on,1.5,true,true,true,true));
  std::cout<<"single-flight keyed receipts and independently verified OFF passed\n";
}
