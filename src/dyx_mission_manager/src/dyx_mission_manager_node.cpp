// H6C: opt-in, ROS 2 C++17 mission geometry -> H6B preview-goal bridge.
// Does not call MissionSequencer::start/next or produce motor/spray messages.
#include "dyx_mission_manager/selection.hpp"
#include "dyx_rpp_cpp17_core/trajectory_builder.hpp"
#include "dyx_rpp_cpp17_core/mission_sequence.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
using namespace std::chrono_literals;

class DyxH6cMissionPreview final : public rclcpp::Node {
 public:
  DyxH6cMissionPreview() : Node("dyx_mission_manager_preview") {
    enabled_ = declare_parameter<bool>("preview_enabled", false);
    const auto flattened = declare_parameter<std::vector<double>>(
      "markings_ned_flat", std::vector<double>{});
    const auto index = declare_parameter<std::int64_t>("preview_marking_index", 0);
    const auto extension_enabled = declare_parameter<bool>("extension_enabled", false);
    const auto extension_distance = declare_parameter<double>("extension_distance_m", 4.0);
    const auto extension_trigger = declare_parameter<double>("extension_trigger_m", 3.0);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/dyx/mission_manager/preview_status", rclcpp::QoS(5));
    // H6B consumes this only as a diagnostic preview goal; no command publisher.
    goal_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/dyx/mission/preview_goal_ned", rclcpp::QoS(5).reliable());
    if (enabled_) {
      const auto selection = dyx::mission_preview::validate_selection(flattened, index);
      dyx::rpp::TrajectoryBuildConfig cfg;
      cfg.extension_enabled = extension_enabled;
      cfg.extension_distance_m = extension_distance;
      cfg.extension_trigger_m = extension_trigger;
      std::vector<dyx::rpp::Point2D> enu;
      enu.reserve(selection.markings.size());
      for (const auto p : selection.markings) enu.push_back({p.east_m,p.north_m});
      auto trajectory = dyx::rpp::NativeTrajectoryBuilder(cfg).build(enu);
      std::vector<dyx::rpp::MarkingGoal> semantic;
      semantic.reserve(trajectory.marking_raw_indices.size());
      for (std::size_t i = 0; i < trajectory.marking_raw_indices.size(); ++i) {
        const auto raw = trajectory.marking_raw_indices[i];
        if (raw > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
            i > static_cast<std::size_t>(std::numeric_limits<int>::max()))
          throw std::invalid_argument("H6C raw or semantic index overflow");
        const std::string label = "PREVIEW_P" + std::to_string(i + 1);
        semantic.push_back({static_cast<int>(raw), static_cast<int>(i),
                            label, "H6C_NOT_CERTIFIED:" + label});
      }
      // Bind to the tested semantic mission sequencer in READY state only.
      // No admission to APPROACHING, SPRAY_ACK or marking authorization.
      dyx::rpp::MissionSequencer mission;
      mission.load(std::move(semantic), dyx::rpp::MissionMode::manual);
      if (mission.stage() != dyx::rpp::MissionStage::ready)
        throw std::logic_error("H6C mission not READY");
      const auto raw_index = trajectory.marking_raw_indices[selection.selected_index];
      const auto& selected = trajectory.navigation.at(raw_index);
      if (selected.point_type != dyx::rpp::point_marking ||
          selected.marking_index != static_cast<int>(selection.selected_index) ||
          selected.point.x != selection.chosen.east_m ||
          selected.point.y != selection.chosen.north_m)
        throw std::logic_error("H6C trajectory mutated the marking anchor");
      selected_north_ = selection.chosen.north_m;
      selected_east_ = selection.chosen.east_m;
      selected_index_ = selection.selected_index;
      navigation_count_ = trajectory.navigation.size();
      dummy_count_ = trajectory.dummy_count;
    }
    tick_ = create_wall_timer(500ms, [this] { tick(); });
    RCLCPP_WARN(get_logger(),
      "H6C MISSION PREVIEW ONLY; enabled=%s; requires separate H6B preview; "
      "no mission progression or hardware actuation",
      enabled_ ? "true" : "false");
  }
 private:
  void tick() {
    std_msgs::msg::String diagnostic;
    if (!enabled_) {
      diagnostic.data = "H6C_DISABLED; PHYSICAL_COMMAND_PERMITTED=false";
      status_pub_->publish(diagnostic);
      return;
    }
    geometry_msgs::msg::PointStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "px4_local_ned";
    goal.point.x = selected_north_;
    goal.point.y = selected_east_;
    goal.point.z = 0.0;
    goal_pub_->publish(goal);
    diagnostic.data = "H6C_PREVIEW_ONLY;index=" + std::to_string(selected_index_) +
      ";navigation_count=" + std::to_string(navigation_count_) +
      ";dummy_count=" + std::to_string(dummy_count_) +
      ";mission_stage=READY;PHYSICAL_COMMAND_PERMITTED=false";
    status_pub_->publish(diagnostic);
  }
  bool enabled_{false};
  double selected_north_{0.0}, selected_east_{0.0};
  std::size_t selected_index_{0}, navigation_count_{0}, dummy_count_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr tick_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<DyxH6cMissionPreview>()); }
  catch (const std::exception& e) {
    std::cerr << "H6C startup failed: " << e.what() << '\n';
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
