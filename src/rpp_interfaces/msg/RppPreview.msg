# Advisory geometry from the C++17 RPP controller. NEVER an actuator command.
std_msgs/Header header
uint64 path_revision
int32 active_marking_index
bool pose_fresh
bool semantic_valid
bool guidance_valid
float64 preview_forward_mps
float64 preview_yaw_rate_enu_radps
float64 heading_enu_rad
float64 remaining_m
bool motion_authorized
string reason
