#include "a21_protocol_zone.hpp"
#include <cassert>
#include <cstdint>
#include <vector>
using namespace dyx::obstacle::a21;
static std::vector<std::uint8_t> frame(int mm) {
  const auto hi=static_cast<std::uint8_t>((mm>>8)&255),lo=static_cast<std::uint8_t>(mm&255);
  return {0xFF,hi,lo,static_cast<std::uint8_t>(0xFFu+hi+lo)};
}
int main() {
  FrameParser p;
  auto a=frame(1234),b=frame(56);
  std::vector<std::uint8_t> x={0x12,0x34};x.insert(x.end(),a.begin(),a.end());x.insert(x.end(),b.begin(),b.end());
  const auto first=p.feed(x.data(),5),second=p.feed(x.data()+5,x.size()-5);
  assert(first.empty());assert(second.size()==2);
  assert(second[0].distance_mm==1234 && second[1].distance_mm==56);
  auto bad=frame(1234);bad.back()=0;
  bad.insert(bad.end(),a.begin(),a.end());
  auto parsed=p.feed(bad);assert(parsed.size()==1 && parsed[0].distance_mm==1234);
  assert(p.bad_checksums==1);
  assert(classify(0xFFFE).kind==Kind::interference);
  assert(classify(0xFFFD).kind==Kind::no_echo);
  assert(classify(0xFFFF).kind==Kind::no_echo);
  assert(classify(0).kind==Kind::blind_zone);
  ZoneTracker t;assert(t.zone(0)==Zone::unknown);
  assert(t.update(classify(1000),0.0)==false);
  assert(t.update(classify(1000),0.1)==false);
  assert(t.update(classify(1000),0.2)==true);
  assert(t.zone(0.3)==Zone::danger);
  for(int i=0;i<5;++i)assert(!t.update(classify(1650),0.3+i*0.01));
  for(int i=0;i<3;++i)t.update(classify(1750),0.4+i*0.01);
  assert(t.confirmed_zone==Zone::warning);
  assert(t.update(classify(0),0.5));assert(t.confirmed_zone==Zone::danger);
  assert(t.update(classify(0xFFFE),0.6)==false);
  assert(t.interference==1 && t.zone(1.2)==Zone::unknown);
  for(int i=0;i<3;++i)t.update(classify(0xFFFD),2.0+i*0.01);
  assert(t.confirmed_zone==Zone::clear);
  bool rejected=false;try{ZoneConfig bad_config;bad_config.danger_mm=5000;bad_config.validate();}
  catch(const std::invalid_argument&){rejected=true;}assert(rejected);
  t.reconfigure(ZoneConfig{2500,4000,3,200,0.5,800});
  for(int i=0;i<3;++i)t.update(classify(2000),3.0+i*0.01);
  assert(t.confirmed_zone==Zone::danger);
  return 0;
}
