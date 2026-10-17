#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
using namespace std::chrono_literals;
// Sensor interface not connected: no fabricated obstacle measurements.
class ObstacleMonitorNode final:public rclcpp::Node{
public:ObstacleMonitorNode():Node("obstacle_monitor_node"){
 if(declare_parameter<bool>("enable_serial",false))throw std::invalid_argument("A21 serial driver not ported; enable_serial forbidden");
 pub_=create_publisher<std_msgs::msg::String>("/obstacle/status",10);
 timer_=create_wall_timer(100ms,[this]{std_msgs::msg::String m;m.data=R"({"schema":"dyx4wd/obstacle@1","enabled":false,"healthy":false,"zone":"unknown","serial_error":"CXX17_A21_SERIAL_NOT_IMPLEMENTED","physical_clearance_verified":false})";pub_->publish(m);});
 }
private:rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;rclcpp::TimerBase::SharedPtr timer_;
};int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<ObstacleMonitorNode>());rclcpp::shutdown();return 0;}
