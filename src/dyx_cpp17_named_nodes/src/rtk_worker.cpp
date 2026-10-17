#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
using namespace std::chrono_literals;
class RtkWorker final:public rclcpp::Node{
public:RtkWorker():Node("rtk_worker"){
 pub_=create_publisher<std_msgs::msg::String>("/rtk_worker/status",5);
 timer_=create_wall_timer(1s,[this]{std_msgs::msg::String m;m.data=R"({"rtcm_receiver_age_verified":false,"ntrip_connected":false,"corrections_fresh":false,"injection_enabled":false,"reason":"NATIVE_NTRIP_TRANSPORT_NOT_PORTED"})";pub_->publish(m);});}
private:rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;rclcpp::TimerBase::SharedPtr timer_;
};int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<RtkWorker>());rclcpp::shutdown();return 0;}
