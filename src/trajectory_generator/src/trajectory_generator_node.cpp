#include "dyx_rpp_cpp17_core/trajectory_builder.hpp"
#include "enu_csv.hpp"
#include "geo_csv.hpp"
#include "px4_origin_guard.hpp"
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <functional>
using namespace std::chrono_literals;

// Explicit ENU-only trajectory preparation. Geographic placement and PX4
// gp_origin calibration are NOT implemented and must not be bypassed.
class TrajectoryGeneratorNode final : public rclcpp::Node {
public:
  TrajectoryGeneratorNode(): Node("trajectory_generator") {
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    waypoints_pub_ = create_publisher<nav_msgs::msg::Path>("/mission_waypoints", qos);
    navigation_pub_ = create_publisher<nav_msgs::msg::Path>("/nav_path", qos);
    types_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/trajectory_generator/path_types", qos);
    indexes_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/trajectory_generator/marking_indices", qos);
    signature_pub_ = create_publisher<std_msgs::msg::String>("/trajectory_generator/path_signature", qos);
    survey_pub_ = create_publisher<std_msgs::msg::String>("/trajectory_generator/survey_targets", qos);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/trajectory_generator/ready", qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("/trajectory_generator/status", qos);
    preview_pub_ = create_publisher<std_msgs::msg::String>("/dyx/trajectory_generator/preview_status", 5);
    declare_parameter<std::vector<double>>("markings_enu_flat", std::vector<double>{});
    declare_parameter<std::string>("mission_enu_csv_file", "");
    declare_parameter<std::string>("mission_geo_csv_file", "");
    declare_parameter<bool>("explicit_geo_diagnostic_enabled", false);
    declare_parameter<bool>("live_px4_origin_enabled", false);
    declare_parameter<double>("diagnostic_px4_origin_latitude_deg", 999.0);
    declare_parameter<double>("diagnostic_px4_origin_longitude_deg", 999.0);
    declare_parameter<bool>("explicit_enu_preparation_enabled", false);
    declare_parameter<bool>("extension_enabled",false);
    declare_parameter<double>("extension_distance_m",4.0);
    declare_parameter<double>("extension_trigger_m",3.0);
    declare_parameter<double>("interpolation_spacing_m",0.05);
    declare_parameter<std::string>("frame_id","map");
    origin_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr m){
        std::lock_guard<std::mutex> lock(mutex_);
        dyx::trajectory::Px4OriginSample sample{};
        sample.observed=true;
        sample.latitude_deg=m->ref_lat;
        sample.longitude_deg=m->ref_lon;
        sample.ref_timestamp_us=m->ref_timestamp;
        sample.sample_timestamp_us=m->timestamp_sample;
        sample.xy_reset_counter=m->xy_reset_counter;
        sample.heading_reset_counter=m->heading_reset_counter;
        sample.xy_global=m->xy_global;
        sample.xy_valid=m->xy_valid;
        sample.received=std::chrono::steady_clock::now();
        origin_guard_.observe(sample);
        if (prepared_live_origin_ && !origin_guard_.good(sample.received)) {
          clear_state();
          state_="INVALIDATED_PX4_ORIGIN_CHANGED_OR_RESET";
          publish_all();
        }
      });
    prepare_srv_ = create_service<std_srvs::srv::Trigger>("/trajectory_generator/prepare",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        try { prepare(); response->success=true; response->message="Explicit ENU-only diagnostic trajectory prepared; not authorized for rover movement"; }
        catch(const std::exception& e) { clear_state(); response->success=false; response->message=e.what(); }
        publish_all();
      });
    clear_srv_ = create_service<std_srvs::srv::Trigger>("/trajectory_generator/clear",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_); clear_state(); publish_all();
        response->success=true; response->message="Diagnostic trajectory cleared";
      });
    timer_ = create_wall_timer(1s,[this]{
      std::lock_guard<std::mutex> lock(mutex_);
      if(prepared_live_origin_ && !origin_guard_.good(std::chrono::steady_clock::now())) {
        clear_state();
        state_="INVALIDATED_PX4_ORIGIN_STALE";
      }
      publish_all();
    });
    clear_state(); publish_all();
  }
private:
  void clear_state() {
    ready_=false; prepared_live_origin_=false; nav_.poses.clear(); marking_.poses.clear(); types_.data.clear(); indexes_.data.clear();
    signature_.data.clear(); survey_.data="[]";
    state_="IDLE: awaiting explicit ENU prepare; geographic placement not implemented";
  }
  void prepare() {
    const bool using_live_geo = !get_parameter("mission_geo_csv_file").as_string().empty() &&
      get_parameter("live_px4_origin_enabled").as_bool();
    if (!using_live_geo && !get_parameter("explicit_enu_preparation_enabled").as_bool())
      throw std::runtime_error("DENIED: neither explicit ENU nor PX4 live-origin geographic mode enabled");
    const auto coordinates=get_parameter("markings_enu_flat").as_double_array();
    const auto csv_file=get_parameter("mission_enu_csv_file").as_string();
    const auto geo_file=get_parameter("mission_geo_csv_file").as_string();
    if(!geo_file.empty() && (!csv_file.empty() || !coordinates.empty()))
      throw std::invalid_argument("DENIED: geographic and ENU sources cannot be mixed");
    if(!geo_file.empty() && !using_live_geo &&
       !get_parameter("explicit_geo_diagnostic_enabled").as_bool())
      throw std::invalid_argument("DENIED: geographic conversion not enabled");
    if(!geo_file.empty() && using_live_geo &&
       get_parameter("explicit_geo_diagnostic_enabled").as_bool())
      throw std::invalid_argument("DENIED: live origin and diagnostic origin cannot be mixed");
    if(!csv_file.empty() && !coordinates.empty())
      throw std::invalid_argument("DENIED: choose one local ENU source, CSV or flat coordinates");
    std::vector<dyx::rpp::Point2D> source;
    std::string geodetic_survey="[]";
    const bool geographic=!geo_file.empty();
    if(geographic){
      const auto origin = using_live_geo ?
        origin_guard_.origin(std::chrono::steady_clock::now()) :
        dyx::trajectory::GeoPoint{
          get_parameter("diagnostic_px4_origin_latitude_deg").as_double(),
          get_parameter("diagnostic_px4_origin_longitude_deg").as_double()};
      auto mission=dyx::trajectory::load_geographic_csv(geo_file,origin);
      source=std::move(mission.enu);
      geodetic_survey=dyx::trajectory::survey_json(mission.survey);
    } else if(!csv_file.empty()) {
      source=dyx::trajectory::load_local_enu_csv(csv_file);
    } else {
      if (coordinates.size()<4 || coordinates.size()%2 || coordinates.size()>20000)
        throw std::invalid_argument("Expected 2 to 10000 ENU marking-point pairs");
      source.reserve(coordinates.size()/2);
      for(size_t i=0;i<coordinates.size();i+=2) {
        if(!std::isfinite(coordinates[i])||!std::isfinite(coordinates[i+1]))
          throw std::invalid_argument("Nonfinite ENU coordinate");
        source.push_back({coordinates[i],coordinates[i+1]});
      }
    }
    dyx::rpp::TrajectoryBuildConfig cfg;
    cfg.extension_enabled=get_parameter("extension_enabled").as_bool();
    cfg.extension_distance_m=get_parameter("extension_distance_m").as_double();
    cfg.extension_trigger_m=get_parameter("extension_trigger_m").as_double();
    cfg.interpolation_spacing_m=get_parameter("interpolation_spacing_m").as_double();
    const auto result=dyx::rpp::NativeTrajectoryBuilder(cfg).build(source);
    const auto frame=get_parameter("frame_id").as_string();
    if(frame!="map" && frame!="odom") throw std::invalid_argument("Explicit ENU must use map or odom frame");
    nav_.header.frame_id=frame; marking_.header.frame_id=frame;
    nav_.poses.clear(); marking_.poses.clear(); types_.data.clear(); indexes_.data.clear();
    const auto append=[&frame](nav_msgs::msg::Path& path,dyx::rpp::Point2D p){
      geometry_msgs::msg::PoseStamped pose;pose.header.frame_id=frame;
      pose.pose.position.x=p.x;pose.pose.position.y=p.y;pose.pose.position.z=0.0;
      pose.pose.orientation.w=1.0;path.poses.push_back(pose);
    };
    for(const auto &p:source) append(marking_,p);
    for(const auto &p:result.navigation){
      append(nav_,p.point);
      types_.data.push_back(static_cast<uint8_t>(p.point_type));
    }
    for(const auto idx:result.marking_raw_indices) indexes_.data.push_back(static_cast<int32_t>(idx));
    // Diagnostic identifier only; not a Python-compatible mission path signature.
    std::ostringstream summary;summary<<"ENU_PREVIEW_ONLY:n="<<source.size()<<":nav="<<nav_.poses.size()
                                    <<":dummy="<<result.dummy_count;
    signature_.data=summary.str();
    survey_.data=geodetic_survey;
    if (using_live_geo && !origin_guard_.good(std::chrono::steady_clock::now()))
      throw std::runtime_error("PX4 origin changed during trajectory compilation");
    ready_=true;
    prepared_live_origin_=using_live_geo;
    state_=using_live_geo ?
      "GEO_PROJECTED_FROM_LIVE_PX4_ORIGIN_NOT_MOTION_CERTIFIED;mission_start_authorized=false;motion_authorized=false" : geographic ?
      "GEO_DIAGNOSTIC_PROJECTED_UNVERIFIED_PX4_ORIGIN;mission_start_authorized=false;motion_authorized=false" :
      "ENU_PREVIEW_PREPARED_NOT_PX4_PLACED;mission_start_authorized=false;motion_authorized=false";
  }
  void publish_all() {
    // Publish no false readiness: /ready remains FALSE until geo/PX4 reference parity.
    std_msgs::msg::Bool ready; ready.data=false; ready_pub_->publish(ready);
    const auto stamp=now();nav_.header.stamp=stamp;marking_.header.stamp=stamp;
    for(auto& p:nav_.poses)p.header.stamp=stamp;
    for(auto& p:marking_.poses)p.header.stamp=stamp;
    navigation_pub_->publish(nav_);waypoints_pub_->publish(marking_);
    types_pub_->publish(types_);indexes_pub_->publish(indexes_);
    signature_pub_->publish(signature_);survey_pub_->publish(survey_);
    std_msgs::msg::String state;state.data=state_;status_pub_->publish(state);
    std_msgs::msg::String preview;preview.data=state_+";enu_preview_available="+(ready_?"true":"false")+";physical_command_emitted=false";
    preview_pub_->publish(preview);
  }
  std::mutex mutex_;
  bool ready_{false}; bool prepared_live_origin_{false}; std::string state_;
  dyx::trajectory::Px4OriginGuard origin_guard_{};
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr origin_sub_;
  nav_msgs::msg::Path nav_,marking_;
  std_msgs::msg::UInt8MultiArray types_;
  std_msgs::msg::Int32MultiArray indexes_;
  std_msgs::msg::String signature_,survey_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr navigation_pub_,waypoints_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr types_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr indexes_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr signature_pub_,survey_pub_,status_pub_,preview_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prepare_srv_,clear_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<TrajectoryGeneratorNode>());}catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("trajectory_generator"),"%s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
