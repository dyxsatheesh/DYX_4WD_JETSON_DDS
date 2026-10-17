#pragma once
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace dyx::rtcm {
inline std::string basic64(const std::string& s){
 static constexpr char abc[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
 std::string out;unsigned value=0;int bits=-6;
 for(unsigned char c:s){value=(value<<8)|c;bits+=8;while(bits>=0){out.push_back(abc[(value>>bits)&63]);bits-=6;}}
 if(bits>-6)out.push_back(abc[((value<<8)>>(bits+8))&63]);
 while(out.size()%4)out.push_back('=');
 return out;
}
inline void validate_host(const std::string& host){
 if(host.empty()||host.size()>253)throw std::invalid_argument("invalid NTRIP host");
 for(unsigned char c:host)if(!(std::isalnum(c)||c=='.'||c=='-'||c==':'))throw std::invalid_argument("invalid NTRIP host characters");
}
inline std::string request(const std::string& host,const std::string& mount,const std::string& user,const std::string& pass){
 validate_host(host);
 if(mount.empty()||mount.size()>128)throw std::invalid_argument("invalid mountpoint");
 for(unsigned char c:mount)if(!(std::isalnum(c)||c=='_'||c=='-'||c=='.'))throw std::invalid_argument("invalid mountpoint characters");
 for(unsigned char c:user+pass)if(c<32||c==127)throw std::invalid_argument("invalid credential characters");
 if(user.empty()!=pass.empty())throw std::invalid_argument("NTRIP user/password must both be set");
 std::string r="GET /"+mount+" HTTP/1.0\r\nHost: "+host+"\r\nUser-Agent: DYX-Cpp17-NTRIP/1.0\r\nNtrip-Version: Ntrip/2.0\r\nConnection: close\r\n";
 if(!user.empty())r+="Authorization: Basic "+basic64(user+":"+pass)+"\r\n";
 return r+"\r\n";
}
struct Response {bool complete{};std::size_t header_bytes{};};
inline Response response_header(const std::vector<std::uint8_t>& bytes){
 if(bytes.size()>8192)throw std::runtime_error("NTRIP response headers exceed limit");
 const std::string s(bytes.begin(),bytes.end());
 const auto end=s.find("\r\n\r\n");
 if(end==std::string::npos)return {};
 const auto first_end=s.find("\r\n");
 const auto first=s.substr(0,first_end);
 if(first!="ICY 200 OK"&&first.rfind("HTTP/1.0 200 ",0)!=0&&first.rfind("HTTP/1.1 200 ",0)!=0)
   throw std::runtime_error("NTRIP caster refused stream or returned non-200 response");
 // A chunked response cannot be fed to RTCM3 as raw bytes.
 std::string lower=s.substr(0,end);std::transform(lower.begin(),lower.end(),lower.begin(),[](unsigned char c){return char(std::tolower(c));});
 if(lower.find("transfer-encoding: chunked")!=std::string::npos)throw std::runtime_error("NTRIP chunked response unsupported");
 return {true,end+4};
}
}
