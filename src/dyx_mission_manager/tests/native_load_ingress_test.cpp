#include "dyx_mission_manager/native_load_ingress.hpp"
#include <cassert>
#include <iostream>
int main(){
 using I=dyx::http_backend::NativeLoadIngress;
 I q;
 const std::string sig(64,'a');
 auto load=I::validate({{"mission_run_id","native_run_1234"},{"signature",sig},{"revision",1}});
 assert(load.ok);
 assert(!I::validate({{"mission_run_id","native_run_1234"},{"signature",sig},{"revision",1},{"trusted_px4_disarmed",true}}).ok);
 assert(!I::validate({{"mission_run_id","bad"},{"signature",sig},{"revision",1}}).ok);
 assert(q.enqueue("session_0001",load,false));
 assert(!q.enqueue("session_0001",load,false));
 assert(q.status_for("other",false).at("state")=="CONTROL_LEASE_REQUIRED");
 auto p=q.pop_for_ros(false);assert(p&&p->session_id=="session_0001");
 q.publish_result("session_0001",412,"PX4_DISARM_REQUIRED");
 assert(q.status_for("session_0001",false).at("detail")=="PX4_DISARM_REQUIRED");
 assert(q.enqueue("session_0001",load,false));
 q.cancel_on_stop();assert(!q.pop_for_ros(false));
 assert(!q.enqueue("session_0001",load,false));
 std::cout<<"PASS: LOAD schema, isolated status, bounded queue, STOP lockout and disarm-denial status\n";
}
