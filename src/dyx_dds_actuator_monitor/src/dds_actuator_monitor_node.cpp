// Phase 2H3C. OBSERVATION ONLY: no publisher, service client, setpoint,
// command serialization, or actuator API exists in this program.
// It can run on the Jetson even while AUX5/AUX6 outputs remain disconnected.
#include "dyx_rpp_cpp17_core/dds_actuator_monitor.hpp"
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
using namespace std::chrono_literals;
using dyx::rpp::DdsObservationResult;
using dyx::rpp::NativeDdsActuatorMonitor;
using dyx::rpp::ObservedPx4Ack;

class DdsActuatorMonitorNode final : public rclcpp::Node {
 public:
  DdsActuatorMonitorNode() : rclcpp::Node("dyx_dds_actuator_monitor") {
    const auto topic=this->declare_parameter<std::string>(
        "ack_topic","/fmu/out/vehicle_command_ack");
    if (topic.empty()) throw std::invalid_argument("DDS ACK topic must not be empty");
    // PX4 XRCE-DDS output topics commonly use BEST_EFFORT + VOLATILE.
    const auto qos=rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    subscription_=this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
        topic,qos,[this](px4_msgs::msg::VehicleCommandAck::ConstSharedPtr msg){
          const auto stamp=std::chrono::steady_clock::now();
          const auto now=std::chrono::duration<double>(stamp.time_since_epoch()).count();
          ObservedPx4Ack ack{};
          ack.command=msg->command;
          ack.result=msg->result;
          ack.target_system=msg->target_system;
          ack.target_component=msg->target_component;
          ack.from_external=msg->from_external;
          ack.px4_timestamp_us=msg->timestamp;
          auto result=monitor_.observe(ack,now);
          if (result==DdsObservationResult::observed) {
            RCLCPP_INFO(this->get_logger(),
                "Observed PX4 command=187 ACK result=%u. No unique receipt, no physical actuator proof.",
                static_cast<unsigned>(ack.result));
          } else if(result==DdsObservationResult::invalid_clock ||
                    result==DdsObservationResult::duplicate_or_reordered) {
            RCLCPP_WARN(this->get_logger(),"DDS ACK observation rejected by monotonic/ordering checks");
          }
        });
    timer_=this->create_wall_timer(5s,[this](){
      const auto now=std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      const auto s=monitor_.status(now);
      // ROS graph introspection only: never create a service client or send a request.
      bool service_found=false;
      for (const auto &named : this->get_service_names_and_types()) {
        if (named.first=="/fmu/vehicle_command") { service_found=true;break; }
      }
      RCLCPP_INFO(this->get_logger(),
        "MONITOR ONLY | command ACKs=%zu accepted_observed=%zu fresh=%s vehicle_command_service=%s | TX DISABLED",
        s.observed_actuator_acks,s.accepted_actuator_acks,
        s.last_ack_fresh?"yes":"no",service_found?"discovered":"not discovered");
    });
    RCLCPP_WARN(this->get_logger(),
      "DRY RUN: DDS actuator monitor has NO publish/call path. It cannot actuate AUX5/AUX6.");
  }
 private:
  NativeDdsActuatorMonitor monitor_{};
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr subscription_{};
  rclcpp::TimerBase::SharedPtr timer_{};
};
} // namespace

int main(int argc, char **argv){
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<DdsActuatorMonitorNode>());}
  catch(const std::exception &e){
    RCLCPP_ERROR(rclcpp::get_logger("dyx_dds_actuator_monitor"),"Monitor exited: %s",e.what());
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();
  return 0;
}
