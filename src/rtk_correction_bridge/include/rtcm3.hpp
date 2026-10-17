#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include <cmath>

namespace dyx::rtcm {
inline std::uint32_t crc24q(const std::uint8_t* data,std::size_t count){
  std::uint32_t c=0;
  for(std::size_t i=0;i<count;++i){
    c ^= std::uint32_t(data[i])<<16;
    for(int b=0;b<8;++b){c<<=1;if(c&0x1000000u)c^=0x1864CFBu;}
    c&=0xFFFFFFu;
  }
  return c;
}
struct Frame {std::vector<std::uint8_t> bytes; int type{-1};};
struct Counters {std::uint64_t valid{}, crc_bad{}, headers_bad{}, discarded{}, partial_timeout{};};
class Parser {
 public:
  explicit Parser(double timeout=2.0):timeout_(timeout){if(!std::isfinite(timeout)||timeout<=0)throw std::invalid_argument("invalid partial timeout");}
  std::vector<Frame> feed(const std::uint8_t* input,std::size_t n,double now){
    if(!std::isfinite(now)|| (seen_ && now<last_))throw std::invalid_argument("nonmonotonic RTCM clock");
    if(n && !input)throw std::invalid_argument("null RTCM input");
    last_=now;seen_=true;
    if(n>65536)throw std::invalid_argument("RTCM input chunk excessive");
    if(n)buffer_.insert(buffer_.end(),input,input+n);
    std::vector<Frame> result;
    while(!buffer_.empty()){
      auto it=std::find(buffer_.begin(),buffer_.end(),0xD3);
      if(it!=buffer_.begin()){
        const auto drop=std::size_t(it-buffer_.begin());c_.discarded+=drop;
        buffer_.erase(buffer_.begin(),it);start_=-1;
        if(buffer_.empty())break;
      }
      if(start_<0)start_=now;
      if(now-start_>=timeout_){++c_.partial_timeout;drop_one();continue;}
      if(buffer_.size()<3)break;
      if((buffer_[1]&0xFCu)!=0){++c_.headers_bad;drop_one();continue;}
      const std::size_t payload=((std::size_t(buffer_[1])&3u)<<8)|buffer_[2];
      const std::size_t total=payload+6;
      if(buffer_.size()<total)break;
      const auto actual=crc24q(buffer_.data(),payload+3);
      const auto sent=(std::uint32_t(buffer_[payload+3])<<16)|(std::uint32_t(buffer_[payload+4])<<8)|buffer_[payload+5];
      if(actual!=sent){++c_.crc_bad;drop_one();continue;}
      Frame f;f.bytes.assign(buffer_.begin(),buffer_.begin()+total);
      if(payload>=2)f.type=(int(buffer_[3])<<4)|(buffer_[4]>>4);
      result.push_back(std::move(f));++c_.valid;
      buffer_.erase(buffer_.begin(),buffer_.begin()+total);start_=-1;
    }
    if(buffer_.size()>65536)throw std::runtime_error("RTCM receive buffer bounds");
    return result;
  }
  const Counters& counters()const{return c_;}
  std::size_t buffered()const{return buffer_.size();}
  void clear(){buffer_.clear();start_=-1;seen_=false;last_=0;}
 private:
  void drop_one(){buffer_.erase(buffer_.begin());++c_.discarded;start_=-1;}
  std::vector<std::uint8_t> buffer_;
  Counters c_{};double timeout_,start_{-1},last_{};bool seen_{false};
};
}
