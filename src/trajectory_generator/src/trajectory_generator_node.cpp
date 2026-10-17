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
class TrajectoryGeneratorNode final: public rclcpp::Node {
public:
 TrajectoryGeneratorNode():Node("trajectory_generator_node") {
  const bool enabled=declare_parameter<bool>("preview_enabled",false);
  const auto coords=declare_parameter<std::vector<double>>("markings_enu_flat",std::vector<double>{});
  out_=create_publisher<std_msgs::msg::String>("/dyx/trajectory_generator/preview_status",5);
  if(enabled){
   if(coords.size()<4||coords.size()%2||coords.size()>20000)throw std::invalid_argument("Expected 2..10000 marking coordinate pairs");
   std::vector<dyx::rpp::Point2D> points;
   for(size_t i=0;i<coords.size();i+=2){if(!std::isfinite(coords[i])||!std::isfinite(coords[i+1])) throw std::invalid_argument("nonfinite coordinate");points.push_back({coords[i],coords[i+1]});}
   dyx::rpp::TrajectoryBuildConfig cfg;
   cfg.extension_enabled=declare_parameter<bool>("extension_enabled",false);
   cfg.extension_distance_m=declare_parameter<double>("extension_distance_m",4.0);
   cfg.extension_trigger_m=declare_parameter<double>("extension_trigger_m",3.0);
   auto built=dyx::rpp::NativeTrajectoryBuilder(cfg).build(points);
   status_="TRAJECTORY_PREVIEW;markings="+std::to_string(points.size())+";navigation="+std::to_string(built.navigation.size())+";dummies="+std::to_string(built.dummy_count);
  } else status_="TRAJECTORY_PREVIEW_DISABLED";
  timer_=create_wall_timer(1s,[this]{std_msgs::msg::String m;m.data=status_+";motion_authorized=false";out_->publish(m);});
 }
private: std::string status_; rclcpp::Publisher<std_msgs::msg::String>::SharedPtr out_; rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<TrajectoryGeneratorNode>());}catch(const std::exception&e){RCLCPP_ERROR(rclcpp::get_logger("trajectory_generator_node"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
