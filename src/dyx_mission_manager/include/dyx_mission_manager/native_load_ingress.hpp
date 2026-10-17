#pragma once
// Authenticated LOAD queue: one pending request, ROS-only consumption.
// Never creates disarm attestations or actuates hardware.
#include "dyx_rpp_cpp17_core/mission_runtime_install.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
namespace dyx::http_backend {
class NativeLoadIngress final {
 public:
  using Clock=std::chrono::steady_clock;
  struct Load { bool ok{false}; std::string run_id; std::string signature; std::uint64_t revision{0}; };
  struct Pending { Load load; std::string session_id; std::uint64_t sequence; Clock::time_point at; };
  static Load validate(const nlohmann::json& j) {
    Load l;
    if (!j.is_object() || j.size()!=3 || !j.contains("mission_run_id") ||
        !j.contains("signature") || !j.contains("revision") ||
        !j.at("mission_run_id").is_string() || !j.at("signature").is_string() ||
        !j.at("revision").is_number_integer()) return l;
    l.run_id=j.at("mission_run_id").get<std::string>();
    l.signature=j.at("signature").get<std::string>();
    // Signed negative revisions must never wrap into uint64.
    if (!j.at("revision").is_number_unsigned() &&
        j.at("revision").get<std::int64_t>() <= 0)
      return l;
    l.revision=j.at("revision").get<std::uint64_t>();
    l.ok=dyx::rpp::valid_native_mission_run_id(l.run_id) &&
        dyx::rpp::valid_path_signature(l.signature) && l.revision>0;
    return l;
  }
  bool enqueue(const std::string& session,Load load,bool stopped){
    std::lock_guard<std::mutex> lk(mu_);
    if(stopped||stopped_||pending_||session.empty()||!load.ok)return false;
    pending_=Pending{std::move(load),session,++next_sequence_,Clock::now()};
    status_={{"state","LOAD_QUEUED"},{"mission_loaded",false},{"decision_only",true}};
    owner_session_=session;
    return true;
  }
  std::optional<Pending> pop_for_ros(bool stopped){
    std::lock_guard<std::mutex> lk(mu_);
    if(stopped||stopped_){stop_locked();return std::nullopt;}
    if(!pending_)return std::nullopt;
    if(Clock::now()-pending_->at>std::chrono::milliseconds(500)){
      status_={{"state","LOAD_REJECTED"},{"detail","EXPIRED_BEFORE_ROS_EXECUTION"},{"mission_loaded",false}};
      pending_.reset();return std::nullopt;
    }
    auto result=std::move(pending_);pending_.reset();return result;
  }
  void publish_result(const std::string& session,int status,const std::string& code){
    std::lock_guard<std::mutex> lk(mu_);
    if(stopped_||session!=owner_session_)return;
    status_={{"state",status==200?"LOAD_INSTALLED_READY_NOT_STARTED":"LOAD_REJECTED"},
             {"result_http_status",status},{"detail",code},
             {"mission_loaded",status==200},{"mission_commands_enabled",false},
             {"motor_commands_enabled",false},{"spray_commands_enabled",false},
             {"physical_command_emitted",false},{"decision_only",true}};
  }
  nlohmann::json status_for(const std::string& session,bool stopped)const{
    std::lock_guard<std::mutex> lk(mu_);
    if(stopped||stopped_)return {{"state","STOP_LATCHED"},{"decision_only",true}};
    if(session.empty()||session!=owner_session_)return {{"state","CONTROL_LEASE_REQUIRED"}};
    return status_;
  }
  void cancel_on_stop(){std::lock_guard<std::mutex>lk(mu_);stop_locked();}
 private:
  void stop_locked(){stopped_=true;pending_.reset();owner_session_.clear();status_={{"state","STOP_LATCHED"}};}
  mutable std::mutex mu_;
  std::optional<Pending>pending_;
  std::string owner_session_;
  nlohmann::json status_{{"state","NO_LOAD_REQUEST"}};
  std::uint64_t next_sequence_{0};
  bool stopped_{false};
};
} // namespace dyx::http_backend
