#include "range_obstacle_guard.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <chrono>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
using namespace std::chrono_literals;

// Native ROS Range measurement observer. Never grants mission motion permission.
// Real serial protocol, mounting geometry and physical clearance attestation
// remain unverified and must not be inferred from a ROS message alone.
class ObstacleMonitorNode final : public rclcpp::Node {
 public:
  ObstacleMonitorNode() : Node("obstacle_monitor_node"),
      guard_(declare_parameter<double>("stop_distance_m",0.8),
             declare_parameter<double>("release_distance_m",1.0),
             declare_parameter<double>("range_timeout_s",0.3)) {
    if(declare_parameter<bool>("enable_serial",false))
      throw std::invalid_argument("Serial protocol adapter not implemented; enable_serial forbidden");
    const auto topic=declare_parameter<std::string>("range_topic","/obstacle/front_range");
    if(topic.empty() || topic[0]!='/')throw std::invalid_argument("Absolute range topic required");
    range_sub_=create_subscription<sensor_msgs::msg::Range>(topic,rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Range::ConstSharedPtr m){
          std::lock_guard<std::mutex> lock(mu_);
          // Timestamp clock domains differ between PX4 and ROS; freshness is
          // assessed from monotonic local receipt, not from untrusted header.
          guard_.observe(m->range,m->min_range,m->max_range,steady_s());
          if(m->radiation_type != sensor_msgs::msg::Range::ULTRASOUND &&
             m->radiation_type != sensor_msgs::msg::Range::INFRARED)
            guard_.invalidate();
        });
    pub_=create_publisher<std_msgs::msg::String>("/obstacle/status",10);
    timer_=create_wall_timer(100ms,[this]{
      std::lock_guard<std::mutex> lock(mu_);
      const auto d=guard_.inspect(steady_s());
      const char* zone= !d.sample_valid ? "unknown" : (d.obstacle_near ? "blocked" : "clear_distance_observed");
      std::ostringstream o;
      o<<std::fixed<<std::setprecision(3)
       <<"{\"schema\":\"dyx4wd/obstacle@1\",\"enabled\":true,\"healthy\":"
       <<(d.sample_valid?"true":"false")<<",\"zone\":\""<<zone
       <<"\",\"range_m\":"<<d.range_m
       <<",\"sample_fresh\":"<<(d.fresh?"true":"false")
       <<",\"obstacle_near\":"<<(d.obstacle_near?"true":"false")
       <<",\"clear_distance_observed\":"<<(d.clear_distance_observed?"true":"false")
       <<",\"physical_clearance_verified\":false,\"serial_error\":\"DRIVER_NOT_INTEGRATED\"}";
      std_msgs::msg::String m;m.data=o.str();pub_->publish(m);
    });
    RCLCPP_WARN(get_logger(),"Range observation active, physical clearance NOT verified");
  }
 private:
  static double steady_s(){using C=std::chrono::steady_clock;return std::chrono::duration<double>(C::now().time_since_epoch()).count();}
  std::mutex mu_;
  dyx::obstacle::RangeGuard guard_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<ObstacleMonitorNode>());}catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("obstacle_monitor"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
