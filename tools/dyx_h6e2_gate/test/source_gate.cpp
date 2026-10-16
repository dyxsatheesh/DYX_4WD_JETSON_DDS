#include <fstream>
#include <cassert>
#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>
static std::string read(const std::string& p){std::ifstream f(p,std::ios::binary);if(!f)throw std::runtime_error("missing source: "+p);std::ostringstream x;x<<f.rdbuf();return x.str();}
static bool no_actuation(const std::string& s){
  for(const std::string needle:{"create_publisher<","/fmu/in/","/rpp/command","/spray/","/servo/",
                                 "socket(","bind(","listen(","MotorCommand","VehicleCommand","std::system("})
    if(s.find(needle)!=std::string::npos)return false;
  return s.find("physical_command_emitted{false}")!=std::string::npos &&
         s.find("/api/mission/status")!=std::string::npos &&
         s.find("/api/telemetry/latest")!=std::string::npos &&
         s.find("provenance_verified")!=std::string::npos;
}
int main(int argc,char**argv){
 if(argc!=3)return 64;
 try{
  const auto src=read(argv[1]);
  const auto root=std::string(argv[2]);
  auto routes=read(root+"/src/rover_backend/rover_backend/mission_routes.py");
  auto sys=read(root+"/src/rover_backend/rover_backend/system_routes.py");
  if(!no_actuation(src)||no_actuation(src+"\ncreate_publisher<px4_msgs::msg::VehicleCommand>()"))
    throw std::runtime_error("unsafe source or mutation accepted");
  if(routes.find("@mission_router.get(\"/status\")")==std::string::npos ||
     sys.find("@system_router.get(\"/api/telemetry/latest\")")==std::string::npos ||
     routes.find("\"report\": mission_report_store.status()") == std::string::npos)
    throw std::runtime_error("legacy API source drift");
  std::cout<<"H6E2A NON-ACTUATION AND SOURCE ROUTE GATE PASSED\n";
  return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
