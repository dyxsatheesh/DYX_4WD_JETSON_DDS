#pragma once
#include <chrono>
#include <string>
#include <stdexcept>
#include <cmath>
namespace dyx::bridge {
struct BridgeAudit {
  bool mission_seen{false};
  bool mission_hold{true};
  bool mission_stopped{true};
  bool mission_command_disabled{true};
  std::chrono::steady_clock::time_point mission_received{};
  static bool field_is(const std::string& s,const std::string& key,const std::string& expected){
    const auto needle=";"+key+"="+expected;
    if(s.rfind(key+"="+expected+";",0)==0)return true;
    auto at=s.find(needle);
    if(at==std::string::npos)return false;
    const auto end=at+needle.size();
    return end==s.size() || s[end]==';';
  }
  void observe(const std::string& status,std::chrono::steady_clock::time_point now){
    mission_seen=true;mission_received=now;
    mission_hold=field_is(status,"hold_zero","true");
    mission_stopped=field_is(status,"emergency_stop","true");
    mission_command_disabled=field_is(status,"physical_command_emitted","false") &&
      field_is(status,"start_enabled","false");
  }
  bool fresh(std::chrono::steady_clock::time_point now,double timeout=0.25)const{
    if(!mission_seen || !std::isfinite(timeout) || timeout<=0.0 || timeout>0.5)return false;
    const double age=std::chrono::duration<double>(now-mission_received).count();
    return std::isfinite(age)&&age>=0.0&&age<=timeout;
  }
  std::string reason(std::chrono::steady_clock::time_point now,
                     std::size_t mode_writers,std::size_t trajectory_writers)const{
    if(mode_writers>0 || trajectory_writers>0)return "OTHER_PX4_DDS_WRITER_DETECTED";
    if(!fresh(now))return "MISSION_STATUS_MISSING_OR_STALE";
    if(!mission_hold || !mission_stopped || !mission_command_disabled)
      return "MISSION_AUTHORITY_NOT_FAIL_CLOSED";
    return "COMMISSIONING_LOCKED";
  }
};
}
