#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
using namespace std::chrono_literals;
class EstimatorStatusRelay final:public rclcpp::Node{
public:EstimatorStatusRelay():Node("estimator_status_relay"){
  sub_=create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1",rclcpp::SensorDataQoS(),[this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr v){last_=std::chrono::steady_clock::now();seen_=true;valid_=v->xy_valid&&v->v_xy_valid&&v->heading_good_for_control&&!v->dead_reckoning;});
  pub_=create_publisher<std_msgs::msg::String>("/estimator_status_relay/status",5);
  timer_=create_wall_timer(100ms,[this]{const bool fresh=seen_&&std::chrono::duration<double>(std::chrono::steady_clock::now()-last_).count()<=0.25;std_msgs::msg::String m;m.data=std::string("CXX17_ESTIMATOR_OBSERVE;fresh=")+(fresh?"true":"false")+";valid="+((fresh&&valid_)?"true":"false")+";motion_authorized=false";pub_->publish(m);});}
private:bool seen_{false},valid_{false};std::chrono::steady_clock::time_point last_{};rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;rclcpp::TimerBase::SharedPtr timer_;
};int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<EstimatorStatusRelay>());rclcpp::shutdown();return 0;}
