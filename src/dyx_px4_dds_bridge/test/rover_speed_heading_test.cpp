#include "dyx_px4_dds_bridge/rover_speed_heading_builder.hpp"
#include <cassert>
#include <cmath>
int main(){
 using dyx::bridge::prepare_rover_speed_heading;
 constexpr double PI=3.14159265358979323846;
 auto north=prepare_rover_speed_heading(.15,PI/2,100,true);
 assert(north.valid && north.mode.velocity && !north.mode.position && !north.mode.body_rate);
 assert(north.speed.speed_body_x==.15f && std::isnan(north.speed.speed_body_y));
 assert(std::abs(north.attitude.yaw_setpoint)<1e-6);
 assert(!north.publish_authorized);
 auto east=prepare_rover_speed_heading(-.1,0,101,true);
 assert(east.valid && east.speed.speed_body_x<0);
 assert(std::abs(east.attitude.yaw_setpoint-PI/2)<1e-6);
 auto west=prepare_rover_speed_heading(0,PI,102,true);
 assert(west.valid && std::abs(west.attitude.yaw_setpoint+PI/2)<1e-6);
 assert(!prepare_rover_speed_heading(.2,NAN,103,true).valid);
 assert(!prepare_rover_speed_heading(.2,0,104,false).valid);
 assert(!prepare_rover_speed_heading(1.2,0,105,true).valid);
 assert(!prepare_rover_speed_heading(0,0,0,true).valid);
}
