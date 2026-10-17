#pragma once
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace dyx::rtcm {
struct ReceiverAgeSample {
 double age_sec{std::numeric_limits<double>::quiet_NaN()};
 int fix_quality{0};
 bool rtk_fixed{false};
 bool rtk_float{false};
};
// Strict, byte-stream NMEA GGA decoding; no authority is conferred by content.
class ReceiverGgaAge final {
public:
 std::optional<ReceiverAgeSample> feed(char byte) {
   if(byte=='$') { line_.clear();line_='$';return std::nullopt; }
   if(line_.empty())return std::nullopt;
   if(byte=='\n'){
     const auto line=line_;line_.clear();
     return parse(line);
   }
   if(line_.size()>=127 || (static_cast<unsigned char>(byte)<32 && byte!='\r')) {
     line_.clear();return std::nullopt;
   }
   if(byte!='\r')line_+=byte;
   return std::nullopt;
 }
 static std::optional<ReceiverAgeSample> parse(const std::string& line) {
   if(line.size()<16 || line.size()>127 || line.front()!='$')return std::nullopt;
   const auto star=line.rfind('*');
   if(star==std::string::npos || star+3!=line.size())return std::nullopt;
   auto h=[](char c)->int {if(c>='0'&&c<='9')return c-'0';if(c>='A'&&c<='F')return c-'A'+10;if(c>='a'&&c<='f')return c-'a'+10;return -1;};
   const int h1=h(line[star+1]),h2=h(line[star+2]);
   if(h1<0||h2<0)return std::nullopt;
   std::uint8_t crc=0;
   for(std::size_t i=1;i<star;++i)crc^=static_cast<std::uint8_t>(line[i]);
   if(crc != ((h1<<4)|h2))return std::nullopt;
   std::vector<std::string> fields;std::size_t start=1;
   for(std::size_t i=1;i<=star;++i)if(i==star||line[i]==','){
     fields.push_back(line.substr(start,i-start));start=i+1;
   }
   if(fields.size()<14 || (fields[0]!="GPGGA" && fields[0]!="GNGGA" && fields[0]!="GLGGA" && fields[0]!="GAGGA"))return std::nullopt;
   // NMEA GGA: quality field 6; age-of-differential-data field 13.
   const auto &quality=fields[6];
   if(quality.size()!=1 || quality[0]<'0'||quality[0]>'9')return std::nullopt;
   const int q=quality[0]-'0';
   if(q!=4 && q!=5)return std::nullopt;
   const auto &age=fields[13];
   if(age.empty()||age.size()>12)return std::nullopt;
   for(char c:age)if(!((c>='0'&&c<='9')||c=='.'))return std::nullopt;
   char* end=nullptr;const double value=std::strtod(age.c_str(),&end);
   if(end!=age.c_str()+age.size() || !std::isfinite(value) || value<0 || value>600)return std::nullopt;
   return ReceiverAgeSample{value,q,q==4,q==5};
 }
private: std::string line_;
};
} // namespace dyx::rtcm
