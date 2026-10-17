#pragma once
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace dyx::obstacle {
struct RangeDecision {
  bool sample_valid{false};
  bool fresh{false};
  bool obstacle_near{false};
  bool clear_distance_observed{false};
  double range_m{0.0};
};
class RangeGuard {
 public:
  RangeGuard(double stop_m=0.8,double release_m=1.0,double timeout_s=0.3)
    :stop_m_(stop_m),release_m_(release_m),timeout_s_(timeout_s) {
    if(!std::isfinite(stop_m)||!std::isfinite(release_m)||!std::isfinite(timeout_s)||
       stop_m<=0.0||release_m<=stop_m||timeout_s<=0.0||timeout_s>10.0)
      throw std::invalid_argument("Invalid obstacle guard limits");
  }
  void observe(double value,double sensor_min,double sensor_max,double received_s){
    if(!std::isfinite(received_s)||received_s<0.0) {invalidate();return;}
    if(seen_&&received_s<=received_s_) {invalidate();return;}
    received_s_=received_s;seen_=true;
    valid_=std::isfinite(value)&&std::isfinite(sensor_min)&&std::isfinite(sensor_max)&&
       sensor_min>=0.0&&sensor_max>sensor_min&&value>=sensor_min&&value<=sensor_max;
    if(!valid_){latched_near_=true;return;}
    range_m_=value;
    if(value<=stop_m_)latched_near_=true;
    else if(value>=release_m_)latched_near_=false;
  }
  RangeDecision inspect(double now_s) const {
    RangeDecision d;
    d.fresh=seen_&&std::isfinite(now_s)&&now_s>=received_s_&&now_s-received_s_<=timeout_s_;
    d.sample_valid=valid_&&d.fresh;
    d.range_m=range_m_;
    // An invalid or stale sample is NOT positive evidence of obstacle clearance.
    d.obstacle_near=d.sample_valid&&latched_near_;
    d.clear_distance_observed=d.sample_valid&&!latched_near_;
    return d;
  }
  void invalidate(){valid_=false;latched_near_=true;seen_=false;}
 private:
  double stop_m_,release_m_,timeout_s_,received_s_{-1.0},range_m_{0.0};
  bool seen_{false},valid_{false},latched_near_{true};
};
} // namespace dyx::obstacle
