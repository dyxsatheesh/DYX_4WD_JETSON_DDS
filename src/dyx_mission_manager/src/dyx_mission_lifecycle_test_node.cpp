#include "dyx_rpp_cpp17_core/mission_sequence.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using namespace dyx::rpp;

class MissionLifecycleTestNode final : public rclcpp::Node {
public:
    MissionLifecycleTestNode()
        : Node("dyx_mission_lifecycle_test")
    {
        const bool simulation_only =
            declare_parameter<bool>("simulation_only", false);

        if (!simulation_only) {
            throw std::runtime_error(
                "Lifecycle test node requires simulation_only=true");
        }

        std::vector<MarkingGoal> goals{
            {0, 0, "P1", "SIM_ONLY_P1"},
            {80, 1, "P2", "SIM_ONLY_P2"}
        };

        mission_.load(std::move(goals), MissionMode::manual);

        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/dyx_test/mission_lifecycle/status", 10);

        command_sub_ = create_subscription<std_msgs::msg::String>(
            "/dyx_test/mission_lifecycle/command",
            10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                handle_command(msg->data);
            });

        timer_ = create_wall_timer(100ms, [this]() {
            MissionSequenceInput input{};
            input.monotonic_time_sec =
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now()
                        .time_since_epoch()).count();

            // Synthetic test-only inputs.
            // Never connect these values to real rover authorization.
            input.motion_health_authorized = true;
            input.emergency_stop = false;

            const auto out = mission_.step(input);
            publish_status(
                std::string("STATE=") +
                mission_stage_name(out.stage) +
                ";INDEX=" + std::to_string(out.current_marking) +
                ";COMPLETED=" + std::to_string(out.completed) +
                ";SKIPPED=" + std::to_string(out.skipped) +
                ";HOLD_ZERO=" + (out.hold_zero ? "true" : "false"));
        });

        RCLCPP_WARN(
            get_logger(),
            "SIMULATION-ONLY lifecycle adapter; "
            "no physical rover commands");
    }

private:
    void publish_status(const std::string& value) {
        std_msgs::msg::String msg;
        msg.data =
            "SIM_ONLY;" + value +
            ";PHYSICAL_COMMAND_PERMITTED=false";
        status_pub_->publish(msg);
    }

    void handle_command(const std::string& command) {
        try {
            if (command == "START") {
                mission_.start(true);
            } else if (command == "PAUSE") {
                mission_.pause();
            } else if (command == "RESUME") {
                mission_.resume(true);
            } else if (command == "SKIP") {
                mission_.skip();
            } else if (command == "NEXT") {
                mission_.next();
            } else if (command == "STOP") {
                mission_.stop();
            } else {
                throw std::invalid_argument("Unknown test command");
            }

            publish_status("COMMAND_ACCEPTED=" + command);

        } catch (const std::exception& e) {
            publish_status(
                "COMMAND_REJECTED=" + command +
                ";REASON=" + e.what());
        }
    }

    MissionSequencer mission_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        status_pub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        command_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(
            std::make_shared<MissionLifecycleTestNode>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            rclcpp::get_logger("mission_lifecycle_test"),
            "%s", e.what());

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
