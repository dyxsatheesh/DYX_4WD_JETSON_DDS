#include "dyx_mission_manager/native_mission_command_preflight.hpp"
#include <cassert>
#include <iostream>
#include <cstdint>
int main() {
  using T=dyx::http_backend::NativeMissionCommandPreflight;
  nlohmann::json j={{"mission_run_id","run_0001"},{"sequence",1}};
  assert(T::validate(j).ok);
  j["sequence"]=-1; assert(!T::validate(j).ok);
  j["sequence"]=0; assert(!T::validate(j).ok);
  j["sequence"]=1.5; assert(!T::validate(j).ok);
  j["sequence"]=std::uint64_t{2}; assert(T::validate(j).ok);
  j["trusted_px4_disarmed"]=true; assert(!T::validate(j).ok);
  j.erase("trusted_px4_disarmed");
  j["mission_run_id"]="../bad"; assert(!T::validate(j).ok);
  j["mission_run_id"]="run_0001";
  auto blocked=T::deny(nlohmann::json::object());
  assert(blocked.at("state")=="COMMAND_BLOCKED");
  assert(blocked.at("command_queued")==false);
  assert(blocked.at("motor_commands_enabled")==false);
  assert(blocked.at("spray_commands_enabled")==false);
  assert(blocked.at("physical_command_emitted")==false);
  std::cout << "PASS: Strict command identity/sequence and uncommissioned gateway hard lockout\\n";
}
