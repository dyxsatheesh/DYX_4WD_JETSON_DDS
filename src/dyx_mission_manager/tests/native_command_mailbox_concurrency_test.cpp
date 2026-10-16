#include "dyx_mission_manager/native_command_mailbox.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

using dyx::http_backend::NativeCommandMailbox;
static NativeCommandMailbox::Request req(const char* path, unsigned long long seq){
 NativeCommandMailbox::Request r{};
 r.method="POST";r.path=path;r.sequence=seq;
 r.principal.authenticated=true;
 r.principal.owns_control_lease=true;
 r.principal.operator_session_id="trusted_test_session";
 r.mission_run_id="trusted_test_run";
 return r;
}
int main(){
 NativeCommandMailbox m;
 using R=NativeCommandMailbox::SubmitResult;
 std::atomic<bool> launch{false};
 std::vector<std::thread> threads;
 for(int i=0;i<8;++i){threads.emplace_back([&,i]{
   while(!launch.load(std::memory_order_acquire)) std::this_thread::yield();
   for(int j=0;j<1000;++j)
     (void)m.submit_verified(req(dyx::rpp::kBackendMissionStartPath,1+i*1000+j),true);
 });}
 launch.store(true,std::memory_order_release);
 for(auto& t:threads)t.join();
 assert(m.queued_normal()<=NativeCommandMailbox::kCapacity);
 assert(m.submit_verified(req(dyx::rpp::kBackendMissionStopPath,90000),true)==R::accepted);
 assert(m.stopped_latched());
 assert(m.queued_normal()==0);
 threads.clear();
 for(int i=0;i<8;++i){threads.emplace_back([&,i]{
   for(int j=0;j<1000;++j)
     assert(m.submit_verified(req(dyx::rpp::kBackendMissionResumePath,100000+i*1000+j),true)==R::stopped_latched);
 });}
 for(auto& t:threads)t.join();
 auto stop=m.pop_for_ros();
 assert(stop && stop->path==dyx::rpp::kBackendMissionStopPath);
 assert(!m.pop_for_ros());
 std::cout<<"PASS: Concurrent producers, STOP cancellation and lockout" << std::endl;
}
