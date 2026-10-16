#pragma once
// H6E2A: in-process C++17 adapter for TWO existing DYX Android REST GET routes.
// It forwards COMPLETE verified legacy JSON bodies byte-for-byte, not invented
// partial payloads. There is NO socket, authentication provider, ROS2 node,
// command dispatcher, PX4 publisher or motor/spray interface here.
// An independently trusted runtime MUST authenticate reader principals and
// produce provenance-verified, fresh legacy-format snapshots. No such runtime
// is connected in H6E2A; the default response is unavailable, never READY.
#include "dyx_backend_cpp17_legacy_readonly/strict_json.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace dyx::h6e2 {
inline constexpr const char* mission_path="/api/mission/status";
inline constexpr const char* telemetry_path="/api/telemetry/latest";

enum class Stream { mission, telemetry };
struct TrustedSnapshot {
  std::string body{};
  std::uint64_t epoch{0};
  std::uint64_t sequence{0};
  double host_received_sec{-1.0};
  bool provenance_verified{false};  // caller must establish this independently
};
struct ReadPrincipal {
  bool authenticated{false};  // only verified ingress may set true
  bool may_read{false};
};
struct ReadRequest {
  std::string method{"GET"};
  std::string path{};
  std::string body{};
  ReadPrincipal principal{};
  double host_now_sec{-1.0};
};
struct ReadResponse {
  int http_status{503};
  std::string body{"{\"detail\":\"Native read-only source unavailable.\"}"};
  bool physical_command_emitted{false};
  bool status_verified_snapshot{false};
  bool read_only{true};
};
struct LegacyReadOnlyConfig {
  double mission_max_age_sec{2.0};
  double telemetry_max_age_sec{0.75};
  std::size_t mission_max_bytes{262144};
  std::size_t telemetry_max_bytes{65536};
  void validate() const {
    if (!std::isfinite(mission_max_age_sec) || mission_max_age_sec<=0.0 || mission_max_age_sec>10.0 ||
        !std::isfinite(telemetry_max_age_sec) || telemetry_max_age_sec<=0.0 || telemetry_max_age_sec>5.0 ||
        mission_max_bytes<1024 || mission_max_bytes>262144 ||
        telemetry_max_bytes<1024 || telemetry_max_bytes>65536)
      throw std::invalid_argument("INVALID_H6E2_LIMITS");
  }
};

inline bool type_key(const JsonValue& v,const char* key, JsonValue::Type type){
  const auto* p=v.find(key);return p && p->is(type);
}
inline bool mission_contract(const JsonValue& root){
  using T=JsonValue::Type;
  if(!root.is(T::object) || !type_key(root,"success",T::boolean) ||
     root.find("success")->scalar!="true" || !type_key(root,"mission",T::object) ||
     !type_key(root,"report",T::object))return false;
  const auto& m=*root.find("mission");
  if(!type_key(m,"state",T::string))return false;
  for(const char* k:{"loaded","ready","accepted_for_start","mission_enable",
                      "emergency_stop","rtk_fixed","marking_active","resume_available"})
    if(!type_key(m,k,T::boolean))return false;
  for(const char* k:{"total_points","completed_points","failed_points","skipped_points",
                      "remaining_points"})if(!type_key(m,k,T::number))return false;
  if(!type_key(m,"point_status",T::array) || !type_key(m,"point_results",T::object))return false;
  return true;
}
inline bool telemetry_contract(const JsonValue& root){
  using T=JsonValue::Type;
  if(!root.is(T::object) || !type_key(root,"generated_at",T::string) ||
     !type_key(root,"revision",T::number))return false;
  for(const char* k:{"backend","ros","vehicle","position","gps","estimator",
                      "rtk","battery","accuracy","safety","obstacle","mission"})
    if(!type_key(root,k,T::object))return false;
  for(const char* k:{"emergency_stop","mission_enable","rtk_fixed","armed","connected",
                      "battery_mission_lockout","marking_active"})
    if(!type_key(root,k,T::boolean))return false;
  if(!type_key(root,"mission_state",T::string)||!type_key(root,"gps_fix",T::number))return false;
  // Both legacy top-level and embedded safety flags must be consistent.
  const auto& safety=*root.find("safety");
  if(!type_key(safety,"emergency_stop",T::boolean) ||
     !type_key(safety,"mission_enable",T::boolean) ||
     safety.find("emergency_stop")->scalar != root.find("emergency_stop")->scalar ||
     safety.find("mission_enable")->scalar != root.find("mission_enable")->scalar)
    return false;
  return true;
}

class NativeLegacyReadOnlyAdapter final {
 public:
  explicit NativeLegacyReadOnlyAdapter(LegacyReadOnlyConfig c={}):cfg_(c){cfg_.validate();}
  // Trusted ingress / publisher only; not a network API. This is not an auth
  // implementation: provenance_verified is a caller attestation, NOT proof.
  bool ingest(Stream stream,const TrustedSnapshot& s){
    if(latched_)return false;
    if(!s.provenance_verified || !s.epoch || !s.sequence ||
       !std::isfinite(s.host_received_sec)||s.host_received_sec<0.0 ||
       (last_host_sec_ && s.host_received_sec<*last_host_sec_) ||
       (epoch_ && s.epoch!=*epoch_))return latch();
    const auto max_bytes=stream==Stream::mission?cfg_.mission_max_bytes:cfg_.telemetry_max_bytes;
    if(s.body.size()>max_bytes || s.body.empty())return latch();
    Slot& slot=stream==Stream::mission?mission_:telemetry_;
    if(slot.current && s.sequence<=slot.current->sequence)return latch();
    try {
      const auto parsed=StrictJson(s.body).parse();
      if(stream==Stream::mission?!mission_contract(parsed):!telemetry_contract(parsed))return latch();
    }catch(const std::exception&){return latch();}
    epoch_=s.epoch;
    last_host_sec_=s.host_received_sec;
    slot.current=s;
    return true;
  }
  ReadResponse handle(const ReadRequest& r){
    if(r.path!=mission_path && r.path!=telemetry_path)
      return {404,"{\"detail\":\"Not Found\"}"};
    if(r.method!="GET")return {405,"{\"detail\":\"Method Not Allowed\"}"};
    if(!r.body.empty())return {400,"{\"detail\":\"Request body not allowed.\"}"};
    if(!r.principal.authenticated)return {401,"{\"detail\":\"Not authenticated\"}"};
    if(!r.principal.may_read)return {403,"{\"detail\":\"Read scope required.\"}"};
    if(!std::isfinite(r.host_now_sec)||r.host_now_sec<0.0 ||
       (last_host_sec_&&r.host_now_sec<*last_host_sec_)){
      latch();return unavailable();
    }
    if(latched_)return unavailable();
    const auto& slot=r.path==mission_path?mission_:telemetry_;
    const double max_age=r.path==mission_path?cfg_.mission_max_age_sec:cfg_.telemetry_max_age_sec;
    if(!slot.current || r.host_now_sec<slot.current->host_received_sec ||
       r.host_now_sec-slot.current->host_received_sec>max_age)return unavailable();
    return {200,slot.current->body,false,true,true};
  }
  // No automatic recovery after malformed, replayed, source/epoch or clock fault.
  // A higher-level trusted operator can reset ONLY with authenticated approval
  // and independently confirmed PX4 DISARMED, never by an HTTP request.
  bool reset_new_epoch(bool operator_verified,bool px4_disarmed,std::uint64_t new_epoch){
    if(!operator_verified||!px4_disarmed||!new_epoch || (epoch_ && new_epoch<=*epoch_))return false;
    mission_={};telemetry_={};last_host_sec_.reset();epoch_=new_epoch;latched_=false;
    return true;
  }
  bool fault_latched()const{return latched_;}
 private:
  struct Slot {std::optional<TrustedSnapshot> current{};};
  static ReadResponse unavailable(){return {503,"{\"detail\":\"Native read-only source unavailable.\"}"};}
  bool latch(){latched_=true;mission_={};telemetry_={};return false;}
  LegacyReadOnlyConfig cfg_{};
  Slot mission_{},telemetry_{};
  std::optional<double> last_host_sec_{};
  std::optional<std::uint64_t> epoch_{};
  bool latched_{false};
};
} // namespace dyx::h6e2
