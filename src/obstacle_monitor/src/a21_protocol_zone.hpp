#pragma once
// Tested Python a21_protocol.py + zones.py policy translated to C++17.
// Pure parsing and observation, no serial, control, clearance or actuation authority.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <vector>
namespace dyx::obstacle::a21 {
enum class Kind { distance, no_echo, blind_zone, interference };
struct Reading {
  Kind kind;
  std::optional<int> distance_mm;
  bool usable() const noexcept { return kind != Kind::interference; }
};
inline Reading classify(std::uint16_t raw) noexcept {
  if(raw==0xFFFE) return {Kind::interference,std::nullopt};
  if(raw>=0xFFFD) return {Kind::no_echo,std::nullopt};
  if(raw==0) return {Kind::blind_zone,0};
  return {Kind::distance,static_cast<int>(raw)};
}
class FrameParser final {
 public:
  std::uint64_t bad_checksums{0};
  std::vector<Reading> feed(const std::uint8_t* bytes, std::size_t size) {
    if(size && !bytes) throw std::invalid_argument("null A21 bytes");
    for(std::size_t i=0;i<size;++i) buffer_.push_back(bytes[i]);
    while(buffer_.size()>256) buffer_.pop_front();
    std::vector<Reading> output;
    while(buffer_.size()>=4) {
      if(buffer_.front()!=0xFF) {buffer_.pop_front();continue;}
      const auto high=buffer_[1],low=buffer_[2],checksum=buffer_[3];
      if(static_cast<std::uint8_t>(0xFFu+high+low)!=checksum) {
        ++bad_checksums;buffer_.pop_front();continue;
      }
      output.push_back(classify(static_cast<std::uint16_t>((high<<8)|low)));
      for(int i=0;i<4;++i) buffer_.pop_front();
    }
    return output;
  }
  std::vector<Reading> feed(const std::vector<std::uint8_t>& bytes) {
    return feed(bytes.data(),bytes.size());
  }
 private:
  std::deque<std::uint8_t> buffer_;
};
enum class Zone { clear, warning, danger, unknown };
struct ZoneConfig {
  int danger_mm{1500},warning_mm{3000},debounce_count{3},hysteresis_mm{200};
  double health_timeout_sec{0.5};
  int immediate_danger_mm{800};
  void validate() const {
    if(danger_mm<300 || warning_mm<=danger_mm || warning_mm>7500 ||
       debounce_count<1 || hysteresis_mm<0 ||
       !std::isfinite(health_timeout_sec) || health_timeout_sec<=0.0 ||
       immediate_danger_mm<0)
      throw std::invalid_argument("invalid A21 zone configuration");
  }
};
class ZoneTracker final {
 public:
  explicit ZoneTracker(ZoneConfig c={}) : config_(c) {config_.validate();}
  void reconfigure(ZoneConfig c) {c.validate(); config_=c; candidate_count_=0;}
  bool update(const Reading& r, double now) {
    if(!std::isfinite(now) || now<0.0) return false;
    if(!r.usable()) {++interference;return false;}
    last_reading_at_=now;
    ++readings;
    distance_mm=r.distance_mm;
    const Zone z=raw_zone(r);
    if(z==candidate_) ++candidate_count_;
    else {candidate_=z;candidate_count_=1;}
    const bool immediate=r.kind==Kind::blind_zone ||
      (r.distance_mm && *r.distance_mm <=
       std::min(config_.immediate_danger_mm,config_.danger_mm));
    if(candidate_count_ >= (immediate?1:config_.debounce_count) && z!=confirmed_zone) {
      confirmed_zone=z;return true;
    }
    return false;
  }
  bool healthy(double now) const noexcept {
    return last_reading_at_ && std::isfinite(now) && now>=*last_reading_at_ &&
      now-*last_reading_at_<=config_.health_timeout_sec;
  }
  Zone zone(double now) const noexcept {return healthy(now)?confirmed_zone:Zone::unknown;}
  Zone confirmed_zone{Zone::clear};
  std::optional<int> distance_mm{};
  std::uint64_t readings{0},interference{0};
 private:
  Zone raw_zone(const Reading& r) const noexcept {
    if(r.kind==Kind::no_echo) return Zone::clear;
    if(r.kind==Kind::blind_zone) return Zone::danger;
    const int mm=r.distance_mm.value_or(0);
    if(confirmed_zone==Zone::danger && mm<=config_.danger_mm+config_.hysteresis_mm)
      return Zone::danger;
    if(mm<=config_.danger_mm) return Zone::danger;
    if(mm<=config_.warning_mm) return Zone::warning;
    return Zone::clear;
  }
  ZoneConfig config_;
  Zone candidate_{Zone::clear};
  int candidate_count_{0};
  std::optional<double> last_reading_at_{};
};
}  // namespace dyx::obstacle::a21
