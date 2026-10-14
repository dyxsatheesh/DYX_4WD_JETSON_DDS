#pragma once
// Pure synthetic trusted-context tests; never feed these booleans from HTTP.
#include "dyx_rpp_cpp17_core/backend_control_sessions.hpp"
#include <cassert>
#include <limits>
#include <string>
using namespace dyx::rpp;
inline TrustedIngressIdentity operator_a() {
  return {true,true,true,"OPERATOR_A_0001","SESSION_A_0001"};
}
inline TrustedIngressIdentity operator_b() {
  return {true,true,true,"OPERATOR_B_0002","SESSION_B_0002"};
}
inline TrustedIngressIdentity viewer() {
  return {true,true,false,"VIEWER_A_0003","SESSION_VIEW_0003"};
}
