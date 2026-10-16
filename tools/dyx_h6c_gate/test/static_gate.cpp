#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stdexcept>
static std::string read(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  if (!f) throw std::runtime_error("cannot read mission preview source");
  std::ostringstream out;out<<f.rdbuf();return out.str();
}
static bool safe(const std::string& s) {
  if (s.find("declare_parameter<bool>(\"preview_enabled\", false)")==std::string::npos)
    return false;
  if (s.find("mission.load(")==std::string::npos ||
      s.find("MissionStage::ready")==std::string::npos) return false;
  if (s.find("/dyx/mission/preview_goal_ned")==std::string::npos) return false;
  for (const char* bad : {
     "create_publisher<px4_msgs", "create_publisher<rpp_interfaces",
     "create_publisher<sensor_msgs", "create_publisher<actuator",
     "create_client<", "create_service<", "MissionSequencer::start(",
     "mission.start(", "mission.next(", "mission.skip(",
     "\"/fmu/in/", "\"/rpp/command", "\"/spray/", "\"/servo/"})
    if (s.find(bad)!=std::string::npos) return false;
  const std::string needle="create_publisher<";
  std::size_t offset=0,count=0;
  while ((offset=s.find(needle,offset))!=std::string::npos) {
    const auto close=s.find('>',offset+needle.size());
    if (close==std::string::npos) return false;
    const auto type=s.substr(offset+needle.size(),close-offset-needle.size());
    if (type!="std_msgs::msg::String" &&
        type!="geometry_msgs::msg::PointStamped") return false;
    ++count; offset=close+1;
  }
  return count==2;
}
int main(int argc,char**argv){
  if(argc!=2){std::cerr<<"Required source path\n";return 64;}
  try {
    auto s=read(argv[1]);
    if (!safe(s)) throw std::runtime_error("source boundary failed");
    if (safe(s+"\ncreate_publisher<px4_msgs::msg::VehicleCommand>()"))
      throw std::runtime_error("mutation test failed to reject motor publisher");
    if (safe(s+"\nmission.start(true)"))
      throw std::runtime_error("mutation test failed to reject mission start");
    std::cout<<"H6C PREVIEW SOURCE GATE PASSED; NOT AN ACTUATOR CERTIFICATE\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr<<"H6C REJECTED: "<<e.what()<<'\n';return 1;
  }
}
