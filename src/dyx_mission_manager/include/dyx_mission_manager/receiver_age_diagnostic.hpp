#pragma once
// Diagnostic-only ROS text transport. NOT a trusted receiver provenance adapter.
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>

namespace dyx::http_backend {
class ReceiverAgeDiagnostic final {
 public:
  bool observe(const std::string& wire, double now) {
    if (!std::isfinite(now) || now < 0 || wire.size() > 2048 ||
        (seen_ && now < received_)) { invalidate(); return false; }
    std::unordered_map<std::string,std::string> kv;
    std::size_t pos=0;
    while(pos < wire.size()) {
      auto end=wire.find(';',pos);
      if(end==std::string::npos)end=wire.size();
      const auto field=wire.substr(pos,end-pos);
      if(field=="CXX17_NTRIP") { /* schema identifier */ }
      else {
        const auto eq=field.find('=');
        if(eq==std::string::npos || eq==0 || eq+1==field.size() ||
            !kv.emplace(field.substr(0,eq),field.substr(eq+1)).second) {
          invalidate(); return false;
        }
      }
      if(end==wire.size())break;
      pos=end+1;
    }
    if(wire.compare(0,12,"CXX17_NTRIP;")!=0 ||
        kv["receiver_age_provenance"]!="LOCAL_SERIAL_DIAGNOSTIC_ONLY" ||
        kv["receiver_age_report_fresh"]!="true" ||
        kv["receiver_age_observer_enabled"]!="true" ||
        kv["mission_age_adapter_connected"]!="false") {
      invalidate(); return false;
    }
    const auto& age=kv["receiver_reported_correction_age_sec"];
    if(age.empty() || age.size()>16){invalidate();return false;}
    unsigned points=0;
    for(char c:age){
      if(c=='.') {if(++points>1){invalidate();return false;}}
      else if(c<'0'||c>'9'){invalidate();return false;}
    }
    try {
      std::size_t used=0;
      const auto parsed=std::stod(age,&used);
      if(used!=age.size() || !std::isfinite(parsed) || parsed<0.0 || parsed>600.0){
        invalidate();return false;
      }
      age_=parsed;
    } catch(...) {invalidate();return false;}
    // ROS subscriber freshness is not equivalent to the underlying GGA sample freshness.
    // C4 supplies an already aged figure; this merely bounds further transport delay.
    received_=now;seen_=true;valid_=true;
    return true;
  }
  bool fresh(double now) const {
    return seen_ && valid_ && std::isfinite(now) && now>=received_ && now-received_<=1.5;
  }
  double age(double now) const {
    return fresh(now)?age_+(now-received_):std::numeric_limits<double>::quiet_NaN();
  }
  bool diagnostic_under_20s(double now) const {return fresh(now)&&age(now)<=20.0;}
  bool motion_authorized() const {return false;}
  bool provenance_verified() const {return false;}
 private:
  void invalidate(){valid_=false;}
  bool seen_{false},valid_{false};double received_{-1},age_{-1};
};
} // namespace dyx::http_backend
