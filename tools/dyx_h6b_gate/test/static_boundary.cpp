// Build-time source publication gate; no Python interpreter or ROS required.
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
int main(int argc, char** argv) {
  if (argc != 2) return 64;
  std::ifstream f(argv[1]);
  if (!f) {std::cerr << "cannot read ROS node source\n"; return 1;}
  std::ostringstream buf;buf << f.rdbuf();
  const std::string s = buf.str();
  const auto has = [&](const char* x){return s.find(x)!=std::string::npos;};
  bool ok = true;
  const auto gate = [&](const char* label, bool value){
    std::cout << (value?"PASS ":"FAIL ") << label << "\n";
    if(!value)ok=false;
  };
  gate("no PX4 input topic publisher", !has("create_publisher<px4_msgs::msg::"));
  gate("no RppCommand publisher", !has("create_publisher<rpp_interfaces::msg::"));
  gate("no production control topic", !has("create_publisher<rpp_interfaces") && !has("\"/rpp/command\""));
  gate("only String status publisher", has("create_publisher<std_msgs::msg::String>") &&
    s.find("create_publisher<", s.find("create_publisher<")+1) == std::string::npos);
  gate("default preview disabled", has("declare_parameter<bool>(\"preview_enabled\", false)"));
  gate("20ms preview tick", has("create_wall_timer(20ms"));
  gate("NED frame checked", has("msg->header.frame_id != \"px4_local_ned\""));
  if(ok)std::cout<<"H6B NO-ACTUATION SOURCE GATE PASSED (STATIC ONLY)\n";
  return ok?0:2;
}
