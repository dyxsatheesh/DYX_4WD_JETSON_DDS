#include "dyx_mission_manager/native_command_decision_audit.hpp"
#include <cassert>
#include <iostream>
int main() {
 using dyx::http_backend::NativeCommandDecisionAudit;
 NativeCommandDecisionAudit a;
 assert(a.reserve("sessionA","runA",1,false));
 assert(!a.reserve("sessionA","runA",1,false));
 assert(!a.reserve("sessionB","runA",2,false));
 assert(!a.reserve("sessionA","runB",2,false));
 assert(a.reserve("sessionA","runA",2,false));
 a.denied("sessionA","runA",2);
 auto s=a.status("sessionA",false);
 assert(s.at("ros_safety_denials")==1);
 assert(s.at("motor_commands_enabled")==false);
 assert(a.status("sessionB",false).at("state")=="COMMAND_SESSION_REQUIRED");
 a.stop();
 assert(!a.reserve("sessionA","runA",3,false));
 assert(a.status("sessionA",true).at("state")=="STOP_LATCHED");
 std::cout << "PASS: session binding, replay rejection, ROS denial audit, STOP lockout\n";
}
