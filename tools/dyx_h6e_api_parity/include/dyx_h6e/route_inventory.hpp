#pragma once
// H6E1: static API-contract evidence. NEVER approves production cutover.
#include <algorithm>
#include <cctype>
#include <array>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace dyx::h6e {
namespace fs = std::filesystem;
struct Route {
  std::string method;
  std::string path;
  std::string source;
  std::string key() const {return method+" "+path;}
};
struct Inventory {
  std::vector<Route> legacy;
  std::vector<Route> native;
  std::vector<Route> unmapped;
  unsigned exact_matches{0};
  bool socketio_declared{false};
  bool production_ready{false}; // This static audit can NEVER certify production.
};
struct Router {const char* filename; const char* symbol;};
inline constexpr std::array<Router,8> routers{{
  {"auth.py","auth_router"}, {"system_routes.py","system_router"},
  {"mission_routes.py","mission_router"}, {"rtk_routes.py","rtk_router"},
  {"rtk_policy_routes.py","rtk_policy_router"},
  {"spray_routes.py","spray_router"}, {"obstacle_routes.py","obstacle_router"},
  {"voice_routes.py","voice_router"}
}};
inline std::string load(const fs::path& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("required source missing: "+path.generic_string());
  std::ostringstream o; o<<f.rdbuf();
  if (!f.eof() && f.fail()) throw std::runtime_error("source read failure");
  return o.str();
}
inline std::string router_prefix(const std::string& s,const std::string& router) {
  const std::regex declaration("\\b"+router+R"(\s*=\s*APIRouter\s*\()",std::regex::ECMAScript);
  std::smatch found;
  if (!std::regex_search(s,found,declaration))
    throw std::runtime_error("router declaration missing: "+router);
  const std::size_t begin=static_cast<std::size_t>(found.position()+found.length());
  const auto close=s.find(')',begin);
  if (close==std::string::npos || close-begin>512)
    throw std::runtime_error("router declaration invalid: "+router);
  const auto block=s.substr(begin,close-begin);
  const std::regex prefix(R"REG(\bprefix\s*=\s*"([^"]*)")REG");
  if (std::regex_search(block,found,prefix)) return found[1].str();
  return {};
}
inline std::vector<Route> python_routes(const fs::path& root) {
  std::vector<Route> routes;
  for (const auto& definition:routers) {
    const auto path=root/"src/rover_backend/rover_backend"/definition.filename;
    const auto s=load(path);
    const auto prefix=router_prefix(s,definition.symbol);
    const std::regex decorator(R"REG(@([A-Za-z_][A-Za-z0-9_]*)\s*\.\s*(get|post|put|patch|delete)\s*\(\s*"([^"]*)")REG");
    for (auto it=std::sregex_iterator(s.begin(),s.end(),decorator);it!=std::sregex_iterator();++it){
      const auto& match=*it;
      if (match[1].str()!=definition.symbol) continue;
      auto method=match[2].str();
      std::transform(method.begin(),method.end(),method.begin(),
                     [](unsigned char c){return static_cast<char>(std::toupper(c));});
      routes.push_back({method,prefix+match[3].str(),definition.filename});
    }
  }
  return routes;
}
inline std::vector<Route> cpp_routes(const fs::path& root) {
  const auto folder=root/"src/dyx_rpp_cpp17_core/include/dyx_rpp_cpp17_core";
  std::vector<Route> result;
  for(const auto& file:{"backend_mission_api.hpp","backend_live_status.hpp"}){
    const auto s=load(folder/file);
    const std::regex constant(R"REG(\bkBackend([A-Za-z0-9_]+)Path\s*=\s*"([^"]+)")REG");
    for(auto it=std::sregex_iterator(s.begin(),s.end(),constant);it!=std::sregex_iterator();++it){
      const auto& match=*it;
      const auto label=match[1].str();
      const auto method=(label=="MissionStatus" || label=="LiveStatus")?"GET":"POST";
      result.push_back({method,match[2].str(),file});
    }
  }
  return result;
}
inline Inventory scan(const fs::path& root) {
  if (!fs::is_directory(root/"src") || !fs::is_directory(root/"tools"))
    throw std::runtime_error("repository root missing src/ or tools/");
  Inventory o;
  o.legacy=python_routes(root);
  o.native=cpp_routes(root);
  const auto realtime=load(root/"src/rover_backend/rover_backend/realtime.py");
  o.socketio_declared=realtime.find("socketio.AsyncServer(")!=std::string::npos;
  std::set<std::string> native_keys;
  for(const auto& r:o.native) {
    if (!native_keys.insert(r.key()).second) throw std::runtime_error("duplicate native route");
  }
  std::set<std::string> legacy_keys;
  for(const auto& r:o.legacy){
    if (!legacy_keys.insert(r.key()).second) throw std::runtime_error("duplicate legacy route");
    if (native_keys.count(r.key())) ++o.exact_matches;
    else o.unmapped.push_back(r);
  }
  return o;
}
inline std::string quote(const std::string& s) {
  static constexpr char hex[]="0123456789abcdef";
  std::string r="\"";
  for(unsigned char c:s) {
    if (c=='"' || c=='\\'){r+='\\';r+=static_cast<char>(c);}
    else if (c=='\n') r+="\\n";
    else if (c=='\r') r+="\\r";
    else if (c=='\t') r+="\\t";
    else if (c<32){r+="\\u00";r+=hex[c>>4];r+=hex[c&15];}
    else r+=static_cast<char>(c);
  }
  return r+'"';
}
inline std::string json(const Inventory& a) {
  std::ostringstream o;
  o<<"{\n  \"production_ready\": false,\n  \"phase\": \"H6E1_API_PARITY_INVENTORY\",\n"
    <<"  \"limitations\": \"Static source inventory only; not full HTTP, authentication, Socket.IO, JSON, timing or hardware parity\",\n"
    <<"  \"legacy_route_count\": "<<a.legacy.size()<<",\n"
    <<"  \"native_model_route_count\": "<<a.native.size()<<",\n"
    <<"  \"exact_method_path_matches\": "<<a.exact_matches<<",\n"
    <<"  \"legacy_without_exact_native_method_path\": "<<a.unmapped.size()<<",\n"
    <<"  \"legacy_socketio_declared\": "<<(a.socketio_declared?"true":"false")<<",\n"
    <<"  \"routes\": [\n";
  for(std::size_t i=0;i<a.legacy.size();++i){
    const auto& r=a.legacy[i];
    const bool matched=std::none_of(a.unmapped.begin(),a.unmapped.end(),[&](const Route& q){return q.key()==r.key();});
    o<<"    {\"method\":"<<quote(r.method)<<",\"path\":"<<quote(r.path)
      <<",\"source\":"<<quote(r.source)<<",\"exact_native_method_path\":"<<(matched?"true":"false")<<"}";
    if(i+1<a.legacy.size())o<<',';
    o<<'\n';
  }
  o<<"  ],\n  \"native_model_routes\": [\n";
  for(std::size_t i=0;i<a.native.size();++i){
    const auto& r=a.native[i];
    o<<"    {\"method\":"<<quote(r.method)<<",\"path\":"<<quote(r.path)<<"}";
    if(i+1<a.native.size())o<<',';
    o<<'\n';
  }
  o<<"  ]\n}\n";
  return o.str();
}
} // namespace dyx::h6e
