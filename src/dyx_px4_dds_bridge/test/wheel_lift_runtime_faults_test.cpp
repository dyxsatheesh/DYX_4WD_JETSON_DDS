#include "dyx_px4_dds_bridge/wheel_lift_runtime_faults.hpp"
#include <cassert>
int main() {
 using dyx::bridge::wheel_lift_runtime_fault;
 auto normal=wheel_lift_runtime_fault(false,false,false,false,false);
 assert(!normal.latch && !normal.request_disarm);
 auto reject_armed=wheel_lift_runtime_fault(true,true,false,false,false);
 assert(reject_armed.latch && reject_armed.request_disarm);
 auto reject_disarmed=wheel_lift_runtime_fault(false,true,false,false,false);
 assert(reject_disarmed.latch && !reject_disarmed.request_disarm);
 auto timedout=wheel_lift_runtime_fault(true,false,true,false,false);
 assert(timedout.latch && timedout.request_disarm);
 auto lost=wheel_lift_runtime_fault(true,false,false,true,false);
 assert(lost.latch && lost.request_disarm);
 auto active=wheel_lift_runtime_fault(true,false,false,true,true);
 assert(!active.latch && !active.request_disarm);
 return 0;
}
