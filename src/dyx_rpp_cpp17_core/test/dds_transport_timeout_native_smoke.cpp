#include "dyx_rpp_cpp17_core/dds_actuator_transport.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
using namespace dyx::rpp;
#define CHECK(x) do{if(!(x)){std::cerr<<"FAIL "<<__LINE__<<" " #x "\n";return 1;}}while(false)
static UniquelyCorrelatedReceipt receipt(const DdsDispatchTicket& t,double now,bool accepted){
 return {t.intent.owner,t.intent.action,t.intent.local_sequence,t.durable_generation,
 t.intent.identity,now,accepted,true};}
int main(){
 const auto dir=std::filesystem::temp_directory_path() / ("dyx_dds_transport_b_"+
 std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
 std::filesystem::create_directories(dir);
 {
 FileBackedActuatorJournal wal(dir/"wal");NativeDdsActuatorTransport t(wal,0.5);
 const auto on=encode_px4_actuator_intent(DdsActuatorOwner::laser,
   DdsActuatorAction::on,1,"r2|P0001",1.0,2);
 auto first=t.begin_on(on,10.0,true,true,true,true);
 CHECK(first.has_value());
 CHECK(!t.poll(10.49).changed);
 CHECK(t.poll(10.5).state==TransportState::timed_out);
 CHECK(!t.begin_on(on,10.6,true,true,true,true)); // no ON retries
 auto off1=encode_px4_actuator_intent(DdsActuatorOwner::laser,
   DdsActuatorAction::off,2,"r2|P0001",-1.0,2);
 const auto r1=t.begin_off(off1,10.7,true);
 CHECK(r1.has_value());
 CHECK(!t.confirm_unique_receipt(receipt(*first,10.71,true),10.71).changed);
 CHECK(t.confirm_unique_receipt(receipt(*r1,10.75,false),10.75).state==TransportState::negative_receipt);
 CHECK(wal.recovery_required());
 auto off2=encode_px4_actuator_intent(DdsActuatorOwner::laser,
   DdsActuatorAction::off,3,"r2|P0001",-1.0,2);
 auto r2=t.begin_off(off2,10.80,true);
 CHECK(r2.has_value() && r2->durable_generation>r1->durable_generation);
 CHECK(!t.confirm_unique_receipt(receipt(*r1,10.81,true),10.81).changed);
 CHECK(t.confirm_unique_receipt(receipt(*r2,10.9,true),10.9).changed);
 CHECK(!t.prove_physical_off(*r1,true).changed);
 CHECK(t.prove_physical_off(*r2,true).changed);
 CHECK(!wal.recovery_required());
 }
 std::error_code ec;std::filesystem::remove_all(dir,ec);
 std::cout<<"timeout, no ON retry, release priority and stale ACK rejection passed\n";
}
