#include <rclcpp/rclcpp.hpp>
#include "dyx_px4_dds_bridge/wheel_lift_stop_policy.hpp"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/rover_speed_setpoint.hpp>
#include <px4_msgs/msg/rover_rate_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <memory>
#include <iostream>
using namespace std::chrono_literals;

// Strictly separate from autonomous mission processing; no software bypass for
// missing physical input signals. Wire both monitored contacts to a verified
// motor-power-interrupting E-stop/deadman circuit before commissioning.
class WheelLiftCommissioner final : public rclcpp::Node {
  using Steady = std::chrono::steady_clock;
  using Clock = Steady;
 public:
  WheelLiftCommissioner() : Node("dyx_wheel_lift_commissioner") {
    const std::string chip=declare_parameter<std::string>("gpio_chip","");
    const int estop=declare_parameter<int>("estop_verified_line",-1);
    const int deadman=declare_parameter<int>("deadman_verified_line",-1);
    if (chip.empty() || estop < 0 || deadman < 0 || estop == deadman)
      throw std::runtime_error("Physical GPIO evidence required; no fallback");
    fd_=::open(chip.c_str(),O_RDONLY|O_CLOEXEC);
    if(fd_<0) throw std::runtime_error("Cannot open GPIO chip");
    gpiohandle_request req{};
    req.lines=2;
    req.lineoffsets[0]=static_cast<unsigned>(estop);
    req.lineoffsets[1]=static_cast<unsigned>(deadman);
    req.flags=GPIOHANDLE_REQUEST_INPUT;
    if(ioctl(fd_,GPIO_GET_LINEHANDLE_IOCTL,&req)<0 || req.fd<0)
      throw std::runtime_error("Cannot reserve two physical safety inputs");
    linefd_=req.fd;
    const auto qos=rclcpp::QoS(5).reliable();
    mode_=create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode",qos);
    speed_=create_publisher<px4_msgs::msg::RoverSpeedSetpoint>("/fmu/in/rover_speed_setpoint",qos);
    rate_=create_publisher<px4_msgs::msg::RoverRateSetpoint>("/fmu/in/rover_rate_setpoint",qos);
    status_=create_publisher<std_msgs::msg::String>("/dyx/wheel_lift/status",10);
    vehicle_command_=create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command",qos);
    ack_=create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack",rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleCommandAck::ConstSharedPtr p){
        // Do not accept unsolicited or late responses when no ACK is pending.
        if(!awaiting_ack_ || p->command!=pending_command_) return;
        last_ack_result_=p->result;
        awaiting_ack_=false;
        if(p->result!=px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
          latch_=true;
      });
    // 1=ARM; 2=OFFBOARD; 3=STOP+DISARM. Single, deliberate operator events.
    operation_=create_subscription<std_msgs::msg::UInt8>(
      "/dyx/wheel_lift/operation",rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::UInt8::ConstSharedPtr p){
        if(p->data==3){
          cmd_valid_=false; forward_=0; yaw_=0; stop_requested_=true;
          return;
        }
        if(p->data!=1 && p->data!=2){ latch_=true; return; }
        // An untrusted software operation request is never sufficient: physical
        // E-stop and deadman must still be continuously verified on each tick.
        if(!had_prestream_ || latch_ || !physical_ok() || !fresh(px4_seen_,0.25)
           || !px4_ok_ || awaiting_ack_) return;
        if(p->data==1 && !armed_) requested_arm_=true;
        if(p->data==2 && armed_ && !offboard_ && prestream_ready())
          requested_offboard_=true;
      });
    vehicle_=create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v1",rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr p){
        armed_=p->arming_state==px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        offboard_=p->nav_state==px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
        px4_ok_=p->timestamp!=0 && !p->failsafe && p->failure_detector_status==0;
        px4_seen_=Steady::now();
      });
    cmd_=create_subscription<std_msgs::msg::Float32MultiArray>("/dyx/wheel_lift/request",rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr p){
        if(p->data.size()!=2 || !std::isfinite(p->data[0]) || !std::isfinite(p->data[1]) ||
           std::abs(p->data[0])>0.15f || std::abs(p->data[1])>0.10f) {
          cmd_valid_=false; latch_=true; return;
        }
        if(latch_ || stop_requested_) return;
        forward_=p->data[0]; yaw_=p->data[1]; cmd_valid_=true; cmd_seen_=Steady::now();
      });
    timer_=create_wall_timer(20ms,[this]{tick();});
    RCLCPP_WARN(get_logger(),"PHYSICAL-COMMISSIONING NODE. Must use independent power-cut E-stop + deadman. Never test with wheels on ground.");
  }
  ~WheelLiftCommissioner(){ if(linefd_>=0) ::close(linefd_); if(fd_>=0) ::close(fd_); }
 private:
  bool fresh(Steady::time_point t,double sec) const {
    return t.time_since_epoch().count()!=0 &&
      std::chrono::duration<double>(Steady::now()-t).count()>=0.0 &&
      std::chrono::duration<double>(Steady::now()-t).count()<=sec;
  }
  bool physical_ok() const {
    if(linefd_<0) return false;
    gpiohandle_data d{};
    return ioctl(linefd_,GPIOHANDLE_GET_LINE_VALUES_IOCTL,&d)==0 &&
           d.values[0]==1 && d.values[1]==1;
  }
  void tick() {
    const bool gpio=physical_ok();
    const bool px4=fresh(px4_seen_,0.25)&&px4_ok_;
    const bool alone=count_publishers("/fmu/in/offboard_control_mode")==1 &&
      count_publishers("/fmu/in/rover_speed_setpoint")==1 &&
      count_publishers("/fmu/in/rover_rate_setpoint")==1 &&
      count_publishers("/fmu/in/trajectory_setpoint")==0 &&
      count_publishers("/fmu/in/rover_attitude_setpoint")==0;
    // Once any physical guard or exclusivity condition fails, require process restart.
    // Do not latch on initial GPIO/PX4 initialization; only after a successful prestream.
    const auto stop_policy = dyx::bridge::wheel_lift_stop_policy({
      had_prestream_, gpio, px4, alone, armed_, was_offboard_,
      cmd_valid_, fresh(cmd_seen_,0.15)});
    if (stop_policy.latch) latch_=true;
    if (stop_policy.request_disarm) stop_requested_=true;
    const bool guarded=gpio&&px4&&alone&&!latch_;
    if(!guarded){ requested_arm_=false; requested_offboard_=false; }
    if(guarded && !had_prestream_) prestream_since_=Steady::now();
    // A status-driven command cannot override PX4 arming health checks.
    // Each request is sent once; PX4 acknowledgement is recorded.
    // STOP is an independent, highest-priority request; it must remain
    // available even when a motion guard fails. It never grants motion.
    if(stop_requested_) {
      cmd_valid_=false; forward_=0; yaw_=0;
      requested_arm_=false; requested_offboard_=false;
      latch_=true;
    }
    // STOP takes priority even if an ARM/OFFBOARD acknowledgement is pending.
    // Keep requesting disarm at a limited interval until PX4 status confirms
    // disarmed. ACK alone is not evidence that motors stopped.
    if(stop_requested_ && ts_ready()) {
      if (!armed_) {
        stop_requested_=false;
        awaiting_ack_=false;
      } else if (!stop_disarm_sent_ || !fresh(last_stop_disarm_,0.5)) {
        awaiting_ack_=false;  // supersede lower-priority command
        send_vehicle_command(400,0.0f,0.0f);
        last_stop_disarm_=Steady::now();
        stop_disarm_sent_=true;
      }
    } else if(!stop_requested_ && guarded && !latch_ && ts_ready()
              && !awaiting_ack_) {
      if(requested_arm_ && !armed_) {
        send_vehicle_command(400,1.0f,0.0f);
        requested_arm_=false;
      } else if(requested_offboard_ && armed_ && !offboard_
                && prestream_ready()) {
        // MAV_CMD_DO_SET_MODE: custom mode Offboard=6 (PX4).
        send_vehicle_command(176,1.0f,6.0f);
        requested_offboard_=false;
      }
    }
    if(awaiting_ack_ && !fresh(request_sent_,0.8)) {
      awaiting_ack_=false; latch_=true;
    }
    // Losing an established Offboard/armed state is a stop condition.
    if(was_offboard_ && (!offboard_ || !armed_)) latch_=true;
    if(offboard_ && armed_) was_offboard_=true;
    const auto now_ns=get_clock()->now().nanoseconds();
    const auto ts=now_ns>0?static_cast<uint64_t>(now_ns/1000):0;
    if(guarded && !latch_ && ts) {
      // Prestream Offboard heartbeat *with zero setpoints* before switching modes.
      px4_msgs::msg::OffboardControlMode m{};
      m.timestamp=ts; m.velocity=true;
      px4_msgs::msg::RoverSpeedSetpoint s{};
      s.timestamp=ts; s.speed_body_x=0.0f; s.speed_body_y=std::nanf("");
      px4_msgs::msg::RoverRateSetpoint r{};
      r.timestamp=ts; r.yaw_rate_setpoint=0.0f;
      const bool live=armed_&&offboard_&&cmd_valid_&&fresh(cmd_seen_,0.15);
      if(live){s.speed_body_x=forward_; r.yaw_rate_setpoint=yaw_;}
      // Publish paired zero setpoints whenever not ready, avoiding last-command hold.
      speed_->publish(s); rate_->publish(r); mode_->publish(m);
      had_prestream_=true;
    } else if(had_prestream_ && ts) {
      // Explicit zero once on transition to inhibit, then stop heartbeat.
      if(!zero_sent_){
        px4_msgs::msg::RoverSpeedSetpoint s{};
        s.timestamp=ts;s.speed_body_x=0;s.speed_body_y=std::nanf("");
        px4_msgs::msg::RoverRateSetpoint r{};
        r.timestamp=ts;r.yaw_rate_setpoint=0;
        speed_->publish(s); rate_->publish(r);zero_sent_=true;
      }
    }
    if(++ticks_%50==0){
      std_msgs::msg::String msg;
      msg.data=std::string("gpio_ok=")+(gpio?"true":"false")+
        ";px4_fresh="+(px4?"true":"false")+
        ";writer_quiet="+(alone?"true":"false")+
        ";latched="+(latch_?"true":"false")+
        ";offboard="+(offboard_?"true":"false")+
        ";armed="+(armed_?"true":"false")+
        ";awaiting_ack="+(awaiting_ack_?"true":"false")+
        ";last_ack_result="+std::to_string(last_ack_result_)+
        ";command_fresh="+((cmd_valid_&&fresh(cmd_seen_,0.15))?"true":"false")+
        ";active_motion="+((guarded&&armed_&&offboard_&&cmd_valid_&&fresh(cmd_seen_,0.15))?"true":"false");
      status_->publish(msg);
    }
  }
  bool ts_ready() {return get_clock()->now().nanoseconds()>0;}
  bool prestream_ready() const {
    return prestream_since_.time_since_epoch().count()!=0 &&
      std::chrono::duration<double>(Steady::now()-prestream_since_).count()>=1.2;
  }
  void send_vehicle_command(uint32_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand v{};
    v.timestamp=static_cast<uint64_t>(get_clock()->now().nanoseconds()/1000);
    v.command=static_cast<uint16_t>(command);
    v.param1=param1; v.param2=param2;
    v.target_system=1; v.target_component=1;
    v.source_system=1; v.source_component=191;
    v.from_external=true;
    pending_command_=v.command;
    awaiting_ack_=true;
    request_sent_=Steady::now();
    vehicle_command_->publish(v);
  }
  int fd_{-1},linefd_{-1};
  bool armed_{false},offboard_{false},px4_ok_{false};
  bool latch_{false},had_prestream_{false},zero_sent_{false},cmd_valid_{false};
  bool requested_arm_{false},requested_offboard_{false},stop_requested_{false};
  bool awaiting_ack_{false},was_offboard_{false};
  bool stop_disarm_sent_{false};
  Steady::time_point last_stop_disarm_{};
  uint16_t pending_command_{0};
  uint8_t last_ack_result_{255};
  float forward_{0},yaw_{0};unsigned ticks_{0};
  Steady::time_point px4_seen_{},cmd_seen_{},prestream_since_{},request_sent_{};
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_;
  rclcpp::Publisher<px4_msgs::msg::RoverSpeedSetpoint>::SharedPtr speed_;
  rclcpp::Publisher<px4_msgs::msg::RoverRateSetpoint>::SharedPtr rate_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr operation_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<WheelLiftCommissioner>());}
  catch(const std::exception&e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
