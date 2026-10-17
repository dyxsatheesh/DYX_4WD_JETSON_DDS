// Step 7A: independent authenticated READ-ONLY native HTTP service.
// Mission commands remain in the single mission_manager_node process until
// a bounded authenticated IPC protocol is implemented and regression-tested.
#include "dyx_mission_manager/native_session_auth.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <nlohmann/json.hpp>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;
using json = nlohmann::json;
using namespace std::chrono_literals;

class RoverBackend final : public rclcpp::Node {
 public:
  RoverBackend() : Node("rover_backend"), acceptor_(ioc_) {
    const char* db = std::getenv("DYX_CPP17_AUTH_DB");
    sessions_ = std::make_shared<dyx::http_backend::NativeSessionAuth>(db ? db : "");
    mission_sub_ = create_subscription<std_msgs::msg::String>(
        "/mission_manager/status", rclcpp::QoS(10),
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          // This ROS string is UNTRUSTED observation, never a command or permission.
          std::lock_guard<std::mutex> lk(status_mutex_);
          last_status_ = m->data.substr(0, 8192);
          last_status_time_ = std::chrono::steady_clock::now();
          seen_status_ = true;
        });
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/rover_backend/migration_status", 10);
    timer_ = create_wall_timer(100ms, [this]() {
      std_msgs::msg::String m;
      m.data = "CXX17_BACKEND;read_only_http_ready=true;mission_commands_enabled=false;"
               "motor_commands_enabled=false;spray_commands_enabled=false;"
               "physical_command_emitted=false";
      status_pub_->publish(m);
    });
    const auto port = declare_parameter<int>("readonly_http_port", 18767);
    if (port < 1024 || port > 65535 || port == 18766)
      throw std::invalid_argument("invalid read-only backend port");
    const tcp::endpoint endpoint(asio::ip::make_address("127.0.0.1"),
                                 static_cast<unsigned short>(port));
    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(asio::socket_base::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen(8);
    acceptor_.non_blocking(true);
    worker_ = std::thread([this] { serve(); });
    RCLCPP_WARN(get_logger(),
      "Read-only authenticated HTTP on localhost:%ld; mission commands DISABLED", port);
  }

  ~RoverBackend() override {
    stopping_.store(true);
    boost::system::error_code ignored;
    acceptor_.close(ignored);
    if (worker_.joinable()) worker_.join();
  }

 private:
  json held_status() {
    std::lock_guard<std::mutex> lk(status_mutex_);
    const double age = seen_status_
        ? std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         last_status_time_).count()
        : -1.0;
    if (!seen_status_ || age < 0.0 || age > 0.5)
      return {{"detail", "MISSION_OWNER_STATUS_STALE_OR_MISSING"},
              {"mission_commands_enabled", false},
              {"ready_for_physical_motion", false}};
    // Diagnostic only: deliberately does not infer trusted authorization
    // from the mission's semicolon-delimited ROS status string.
    return {{"mission_status_observation", last_status_},
            {"observation_age_sec", age},
            {"mission_commands_enabled", false},
            {"ready_for_physical_motion", false},
            {"physical_command_emitted", false}};
  }

  std::pair<http::status, json> route(const http::request<http::string_body>& req) {
    if (req.method() == http::verb::get && req.target() == "/api/native/health") {
      return {http::status::ok, {
          {"service", "dyx_cpp17_rover_backend"},
          {"state", "READ_ONLY_BACKEND_MIGRATION"},
          {"authentication_configured", sessions_->configured()},
          {"mission_commands_enabled", false},
          {"motor_commands_enabled", false},
          {"spray_commands_enabled", false},
          {"physical_command_emitted", false}}};
    }

    // No command routes exist in this process in Step 7A. Never forward a
    // command to the mission owner or treat HTTP status as PX4 authority.
    if (req.method() != http::verb::get)
      return {http::status::service_unavailable,
              {{"detail", "COMMAND_IPC_NOT_COMMISSIONED"},
               {"mission_commands_enabled", false}}};

    if (req.target() != "/api/native/auth/session" &&
        req.target() != "/api/native/mission/status")
      return {http::status::not_found, {{"detail", "UNKNOWN_READ_ONLY_ROUTE"}}};

    if (!sessions_->configured())
      return {http::status::service_unavailable,
              {{"detail", "NATIVE_SESSION_DB_NOT_CONFIGURED"}}};

    std::string token;
    const auto bearer = req.find(http::field::authorization);
    if (bearer != req.end()) {
      const std::string header = bearer->value().to_string();
      if (header.rfind("Bearer ", 0) == 0) token = header.substr(7);
    }
    if (token.empty()) {
      const auto alternate = req.find("X-Rover-Token");
      if (alternate != req.end()) token = alternate->value().to_string();
    }
    const auto session = sessions_->verify(token);
    if (!session.authenticated)
      return {http::status::unauthorized,
              {{"detail", "AUTHENTICATION_REQUIRED"}}};

    if (req.target() == "/api/native/auth/session")
      return {http::status::ok,
              {{"authenticated", true},
               {"session", {{"session_id", session.session_id},
                             {"username", session.username}}},
               {"decision_only", true},
               {"mission_commands_enabled", false}}};

    json status = held_status();
    if (status.contains("detail"))
      return {http::status::service_unavailable, std::move(status)};
    return {http::status::ok, std::move(status)};
  }

  void serve() {
    while (!stopping_.load()) {
      boost::system::error_code ec;
      tcp::socket socket(ioc_);
      acceptor_.accept(socket, ec);
      if (ec == asio::error::would_block || ec == asio::error::try_again) {
        std::this_thread::sleep_for(10ms);
        continue;
      }
      if (ec) {
        if (!stopping_.load())
          RCLCPP_ERROR(get_logger(), "Backend accept: %s", ec.message().c_str());
        continue;
      }
      try {
        beast::tcp_stream stream(std::move(socket));
        stream.expires_after(2s);
        beast::flat_buffer buffer;
        http::request_parser<http::string_body> parser;
        parser.body_limit(8192);
        http::read(stream, buffer, parser, ec);
        if (!ec) {
          auto result = route(parser.get());
          http::response<http::string_body> response{result.first, 11};
          response.set(http::field::content_type, "application/json");
          response.set(http::field::server, "DYX-CPP17-ROVER-BACKEND");
          response.keep_alive(false);
          response.body() = result.second.dump();
          response.prepare_payload();
          stream.expires_after(2s);
          http::write(stream, response, ec);
        }
        boost::system::error_code ignored;
        stream.socket().shutdown(tcp::socket::shutdown_both, ignored);
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Backend request error: %s", e.what());
      }
    }
  }

  asio::io_context ioc_{};
  tcp::acceptor acceptor_;
  std::atomic<bool> stopping_{false};
  std::thread worker_{};
  std::shared_ptr<dyx::http_backend::NativeSessionAuth> sessions_{};
  std::mutex status_mutex_{};
  std::string last_status_{};
  std::chrono::steady_clock::time_point last_status_time_{};
  bool seen_status_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RoverBackend>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("rover_backend"),
                 "Backend startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
