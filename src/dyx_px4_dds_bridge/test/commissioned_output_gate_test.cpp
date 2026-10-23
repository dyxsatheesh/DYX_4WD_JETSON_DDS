#include "dyx_px4_dds_bridge/commissioned_output_gate.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace dyx::bridge;
static CommissionedEvidence good(){
 CommissionedEvidence e{};
 e.requested_enable=e.independent_hardware_interlock_ok=e.operator_deadman_verified=true;
 e.rc_takeover_verified=e.estop_released_verified=e.estop_fresh=true;
 e.px4_status_fresh=e.px4_armed=e.px4_offboard=true;
 e.mission_authorized=true; e.mission_stop=false;
 e.command_fresh=e.heartbeat_fresh=e.estimator_valid=e.writer_quiet=true;
 e.forward_mps=0.08; e.yaw_rate_radps=0.05;
 return e;
}
int main(){
 int checks=0;
 auto denied=[&](const CommissionedEvidence& e){assert(!evaluate_commissioned_output(e).allow_publish);++checks;};
 denied(CommissionedEvidence{});
 auto e=good(); assert(evaluate_commissioned_output(e).allow_publish);++checks;
 e=good();e.requested_enable=false;denied(e);
 e=good();e.independent_hardware_interlock_ok=false;denied(e);
 e=good();e.operator_deadman_verified=false;denied(e);
 e=good();e.rc_takeover_verified=false;denied(e);
 e=good();e.estop_released_verified=false;denied(e);
 e=good();e.estop_fresh=false;denied(e);
 e=good();e.px4_status_fresh=false;denied(e);
 e=good();e.px4_armed=false;denied(e);
 e=good();e.px4_offboard=false;denied(e);
 e=good();e.mission_authorized=false;denied(e);
 e=good();e.mission_stop=true;denied(e);
 e=good();e.command_fresh=false;denied(e);
 e=good();e.heartbeat_fresh=false;denied(e);
 e=good();e.estimator_valid=false;denied(e);
 e=good();e.writer_quiet=false;denied(e);
 e=good();e.forward_mps=0.151;denied(e);
 e=good();e.forward_mps=-0.151;denied(e);
 e=good();e.yaw_rate_radps=0.101;denied(e);
 e=good();e.yaw_rate_radps=-0.101;denied(e);
 e=good();e.forward_mps=std::numeric_limits<double>::quiet_NaN();denied(e);
 e=good();e.yaw_rate_radps=std::numeric_limits<double>::infinity();denied(e);
 e=good();e.forward_mps=0.15;e.yaw_rate_radps=0.10;
 assert(evaluate_commissioned_output(e).allow_publish);++checks;
 std::cout << "PASS: commissioned output policy " << checks << " tests (hardware not attested)\n";
}
