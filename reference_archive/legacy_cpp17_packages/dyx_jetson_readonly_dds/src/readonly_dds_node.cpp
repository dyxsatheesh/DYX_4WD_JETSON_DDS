// Phase H5Y read-only PX4 DDS telemetry; DOES NOT publish control setpoints.
#include "dyx_jetson_readonly_dds/status_guard.hpp"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <chrono>
#include <memory>
using namespace std::chrono_literals;
class DyxJetsonReadOnlyPx4 final : public rclcpp::Node {
 public:
  DyxJetsonReadOnlyPx4():Node("dyx_jetson_readonly_px4"){
    const auto qos=rclcpp::SensorDataQoS();
    status_sub_=create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status",qos,
      [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr m){
        const auto t=age_sec();
        guard_.observe_status(
          m->arming_state==px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED,
          m->nav_state==px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD,t);
      });
    position_sub_=create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position",qos,
      [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr m){
        guard_.observe_position(m->xy_valid && m->z_valid,age_sec());
      });
    report_timer_=create_wall_timer(1000ms,[this](){
      RCLCPP_INFO(get_logger(),
        "OBSERVE_ONLY output_enabled=false px4_armed=%d offboard=%d fresh=%d",
        guard_.px4_armed,guard_.px4_offboard,guard_.observation_fresh(age_sec()));
    });
    RCLCPP_WARN(get_logger(),"READ-ONLY OBSERVER; NOT a PX4 safety controller");
  }
 private:
  static double age_sec(){
    return std::chrono::duration<double>(
       std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  dyx::readonly_dds::State guard_{};
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<DyxJetsonReadOnlyPx4>());
  rclcpp::shutdown();return 0;
}
