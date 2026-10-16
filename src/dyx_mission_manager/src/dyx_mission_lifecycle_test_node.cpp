#include "dyx_rpp_cpp17_core/mission_sequence.hpp"
#include "dyx_rpp_cpp17_core/mission_safety.hpp"

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
        if (!declare_parameter<bool>("simulation_only", false)) {
            throw std::runtime_error(
                "Lifecycle test requires simulation_only=true");
        }

        // Opt-in synthetic startup for isolated ROS 2 testing only.
        // Never trusted as real rover health evidence.
        if (declare_parameter<bool>("simulation_initial_healthy", false)) {
            rtk_ = dyx::safety::RtkState::fixed;
            battery_v_ = 25.0;
            estop_ = false;
            ekf_valid_ = true;
            px4_heartbeat_enabled_ = true;
            backend_heartbeat_enabled_ = true;
        }

        mission_.load({
            {0, 0, "P1", "SIM_ONLY_P1"},
            {80, 1, "P2", "SIM_ONLY_P2"}
        }, MissionMode::manual);

        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/dyx_test/mission_lifecycle/status", 10);

        command_sub_ = create_subscription<std_msgs::msg::String>(
            "/dyx_test/mission_lifecycle/command", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                command(msg->data);
            });

        timer_ = create_wall_timer(100ms, [this]() {
            auto decision = evaluate_safety();

            MissionSequenceInput input{};
            input.monotonic_time_sec = monotonic_now();
            input.motion_health_authorized =
                decision.continue_allowed;
            input.emergency_stop =
                decision.emergency_latched;

            const auto out = mission_.step(input);

            publish(
                "STATE=" +
                std::string(mission_stage_name(out.stage)) +
                ";INDEX=" + std::to_string(out.current_marking) +
                ";HOLD_ZERO=" +
                (out.hold_zero ? "true" : "false") +
                ";SAFETY=" + decision.reason +
                ";START_ALLOWED=" +
                (decision.start_resume_allowed ? "true" : "false") +
                ";CONTINUE_ALLOWED=" +
                (decision.continue_allowed ? "true" : "false") +
                ";PAUSE_REQUIRED=" +
                (decision.pause_required ? "true" : "false"));
        });

        RCLCPP_WARN(
            get_logger(),
            "H7 SIMULATION ONLY. Synthetic safety inputs. "
            "No motor, PX4 or spray command publishers.");
    }

private:
    static double monotonic_now() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now()
                .time_since_epoch()).count();
    }

    dyx::safety::Decision evaluate_safety() {
        const double now = monotonic_now();

        dyx::safety::Input input{};
        input.now_s = now;
        input.rtk = rtk_;
        input.battery_v = battery_v_;
        input.emergency_stop = estop_;
        input.estimator_valid = ekf_valid_;

        if (px4_heartbeat_enabled_)
            last_px4_heartbeat_ = now;

        if (backend_heartbeat_enabled_)
            last_backend_heartbeat_ = now;

        input.px4_heartbeat_s = last_px4_heartbeat_;
        input.backend_heartbeat_s = last_backend_heartbeat_;

        last_decision_ = safety_.evaluate(input);
        return last_decision_;
    }

    void publish(const std::string& value) {
        std_msgs::msg::String msg;
        msg.data = "SIM_ONLY;" + value +
                   ";PHYSICAL_COMMAND_PERMITTED=false";
        status_pub_->publish(msg);
    }

    void command(const std::string& value) {
        try {
            // These SIM_* commands are artificial test inputs.
            // They never represent authenticated hardware evidence.
            if (value == "SIM_HEALTHY") {
                rtk_ = dyx::safety::RtkState::fixed;
                battery_v_ = 25.0;
                estop_ = false;
                ekf_valid_ = true;
                px4_heartbeat_enabled_ = true;
                backend_heartbeat_enabled_ = true;
            } else if (value == "SIM_FLOAT") {
                rtk_ = dyx::safety::RtkState::floating;
            } else if (value == "SIM_BATTERY_LOW") {
                battery_v_ = 21.5;
            } else if (value == "SIM_ESTOP") {
                estop_ = true;
            } else if (value == "SIM_EKF_INVALID") {
                ekf_valid_ = false;
            } else if (value == "SIM_PX4_STALE") {
                px4_heartbeat_enabled_ = false;
            } else if (value == "SIM_BACKEND_STALE") {
                backend_heartbeat_enabled_ = false;
            } else if (value == "STOP") {
                mission_.stop();
            } else if (value == "PAUSE") {
                mission_.pause();
            } else if (value == "START") {
                const auto decision = evaluate_safety();
                mission_.start(decision.start_resume_allowed);
            } else if (value == "RESUME") {
                const auto decision = evaluate_safety();
                mission_.resume(decision.start_resume_allowed);
            } else if (value == "NEXT") {
                mission_.next();
            } else if (value == "SKIP") {
                const auto decision = evaluate_safety();
                if (!decision.continue_allowed)
                    throw std::logic_error("Safety gate denies SKIP");
                mission_.skip();
            } else {
                throw std::invalid_argument("Unknown test command");
            }

            publish("COMMAND_ACCEPTED=" + value);

        } catch (const std::exception& e) {
            publish("COMMAND_REJECTED=" + value +
                    ";REASON=" + e.what());
        }
    }

    MissionSequencer mission_;
    dyx::safety::Arbiter safety_;
    dyx::safety::Decision last_decision_{};

    dyx::safety::RtkState rtk_{
        dyx::safety::RtkState::unknown
    };

    double battery_v_{0.0};
    bool estop_{true};
    bool ekf_valid_{false};

    bool px4_heartbeat_enabled_{false};
    bool backend_heartbeat_enabled_{false};

    double last_px4_heartbeat_{-1.0};
    double last_backend_heartbeat_{-1.0};

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
            rclcpp::get_logger("dyx_h7_lifecycle"),
            "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
