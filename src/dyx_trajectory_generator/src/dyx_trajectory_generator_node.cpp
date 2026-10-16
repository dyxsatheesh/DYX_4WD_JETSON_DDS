#include "dyx_rpp_cpp17_core/trajectory_builder.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
using namespace std::chrono_literals;
class TrajectoryGeneratorNode final : public rclcpp::Node {
public:
  TrajectoryGeneratorNode() : Node("dyx_trajectory_generator") {
    const bool enabled=declare_parameter<bool>("preview_enabled", false);
    const auto coords=declare_parameter<std::vector<double>>("markings_enu_flat", std::vector<double>{});
    publisher_=create_publisher<std_msgs::msg::String>("/dyx/trajectory_generator/preview_status",rclcpp::QoS(5));
    if (enabled) {
      if(coords.size()<4 || coords.size()%2!=0 || coords.size()>20000)
        throw std::invalid_argument("Expected 2..10000 marking ENU pairs");
      std::vector<dyx::rpp::Point2D> points;
      for(std::size_t i=0;i<coords.size();i+=2) {
        if(!std::isfinite(coords[i])||!std::isfinite(coords[i+1]))
          throw std::invalid_argument("Nonfinite ENU coordinate");
        points.push_back({coords[i],coords[i+1]});
      }
      dyx::rpp::TrajectoryBuildConfig cfg;
      cfg.extension_enabled=declare_parameter<bool>("extension_enabled",false);
      cfg.extension_distance_m=declare_parameter<double>("extension_distance_m",4.0);
      cfg.extension_trigger_m=declare_parameter<double>("extension_trigger_m",3.0);
      const auto t=dyx::rpp::NativeTrajectoryBuilder(cfg).build(points);
      status_="PREVIEW_ONLY;markings="+std::to_string(points.size())+
        ";navigation="+std::to_string(t.navigation.size())+
        ";dummies="+std::to_string(t.dummy_count);
    } else status_="PREVIEW_DISABLED";
    timer_=create_wall_timer(1s,[this]{
      std_msgs::msg::String msg;
      msg.data=status_+";PHYSICAL_COMMAND_PERMITTED=false";
      publisher_->publish(msg);
    });
  }
private:
  std::string status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<TrajectoryGeneratorNode>());}
  catch(const std::exception &e){RCLCPP_ERROR(rclcpp::get_logger("trajectory_generator"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}