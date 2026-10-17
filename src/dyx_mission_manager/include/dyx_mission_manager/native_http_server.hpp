#pragma once

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <nlohmann/json.hpp>
#include "dyx_mission_manager/native_session_auth.hpp"
#include "dyx_mission_manager/native_command_mailbox.hpp"
#include "dyx_mission_manager/native_mission_preview.hpp"

#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <functional>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace dyx::http_backend {

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;

using tcp = asio::ip::tcp;

// Written only by the ROS executor. Read only by HTTP workers.
// A stale ROS control loop must never appear as live telemetry.
class NativeStatusSnapshot final {
 public:
  void publish(nlohmann::json value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = std::move(value);
    updated_ = std::chrono::steady_clock::now();
    available_ = true;
  }

  std::pair<bool, nlohmann::json> read() const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!available_)
      return {false, {{"detail", "ROS_STATUS_NOT_READY"}}};

    const double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - updated_).count();

    if (age > 0.50)
      return {false, {
          {"detail", "ROS_STATUS_STALE"},
          {"snapshot_age_sec", age}
      }};

    auto result = value_;
    result["snapshot_age_sec"] = age;
    result["ros_loop_fresh"] = true;
    return {true, std::move(result)};
  }

 private:
  mutable std::mutex mutex_;
  nlohmann::json value_{};
  std::chrono::steady_clock::time_point updated_{};
  bool available_{false};
};

class HttpSession final
    : public std::enable_shared_from_this<HttpSession> {
 public:
  HttpSession(
      tcp::socket socket,
      std::shared_ptr<NativeStatusSnapshot> snapshot,
      std::shared_ptr<NativeSessionAuth> sessions,
      NativeCommandMailbox& mailbox)
      : stream_(std::move(socket)),
        snapshot_(std::move(snapshot)),
        sessions_(std::move(sessions)), mailbox_(mailbox) {}

  void start() {
    parser_.body_limit(8192);
    stream_.expires_after(std::chrono::seconds(2));

    http::async_read(
        stream_,
        buffer_,
        parser_,
        [self = shared_from_this()](
            beast::error_code ec,
            std::size_t) {
          if (!ec) self->respond();
        });
  }

 private:
  void respond() {
    const auto request = parser_.release();

    auto response =
        std::make_shared<http::response<http::string_body>>();

    response->version(11);
    response->set(http::field::server, "DYX-CPP17");
    response->set(
        http::field::content_type,
        "application/json; charset=utf-8");
    response->keep_alive(false);

    const bool native_health =
        request.method() == http::verb::get &&
        request.target() == "/api/native/health";

    if (native_health) {
      response->result(http::status::ok);
      response->body() = nlohmann::json{
          {"ok", true},
          {"service", "dyx_cpp17_http"},
          {"state", "PREVIEW_ONLY"},
          {"authentication_ready", false},
          {"mission_commands_enabled", false},
          {"motor_commands_enabled", false},
          {"spray_commands_enabled", false}
      }.dump();
    } else if (request.method() == http::verb::get &&
               request.target() == "/api/native/auth/session") {
      if (!sessions_->configured()) {
        response->result(http::status::service_unavailable);
        response->body() = R"({"detail":"NATIVE_SESSION_DB_NOT_CONFIGURED"})";
      } else {
        std::string token;
        const auto bearer = request.find(http::field::authorization);
        if (bearer != request.end()) {
          std::string value = bearer->value().to_string();
          if (value.size()>=7 && value.compare(0,7,"Bearer ")==0)
            token = value.substr(7);
        }
        if (token.empty()) {
          const auto header = request.find("X-Rover-Token");
          if (header!=request.end()) token=header->value().to_string();
        }
        const auto session = sessions_->verify(token);
        if (!session.authenticated) {
          response->result(http::status::unauthorized);
          response->body() = R"({"detail":"AUTHENTICATION_REQUIRED"})";
        } else {
          response->result(http::status::ok);
          response->body() = nlohmann::json{
              {"authenticated",true},
              {"session",{{"session_id",session.session_id},
                          {"username",session.username}}},
              {"decision_only",true},
              {"mission_commands_enabled",false}}
              .dump();
        }
      }
    } else if (request.method() == http::verb::get &&
               request.target() == "/api/native/mission/capabilities") {
      response->result(http::status::ok);
      response->body() = nlohmann::json{
          {"service", "dyx_cpp17_native_mission_preview"},
          {"auth_database_configured", sessions_->configured()},
          {"authenticated_preview_supported", true},
          {"preview_coordinate_frame", "LOCAL_ENU_METRES"},
          {"live_android_api_compatible", false},
          {"mission_load_enabled", false},
          {"mission_commands_enabled", false},
          {"motor_commands_enabled", false},
          {"spray_commands_enabled", false},
          {"physical_command_emitted", false}
      }.dump();
    } else if (request.method() == http::verb::post &&
               request.target() == "/api/native/mission/preview") {
      // Never accept a network value as proof of control lease or disarm.
      if (!sessions_->configured()) {
        response->result(http::status::service_unavailable);
        response->body() = R"({"detail":"NATIVE_SESSION_DB_NOT_CONFIGURED"})";
      } else {
        std::string token;
        const auto bearer = request.find(http::field::authorization);
        if (bearer != request.end()) {
          const std::string value = bearer->value().to_string();
          if (value.size() >= 7 && value.compare(0, 7, "Bearer ") == 0)
            token = value.substr(7);
        }
        if (token.empty()) {
          const auto header = request.find("X-Rover-Token");
          if (header != request.end()) token = header->value().to_string();
        }
        const auto verified = sessions_->verify(token);
        if (!verified.authenticated) {
          response->result(http::status::unauthorized);
          response->body() = R"({"detail":"AUTHENTICATION_REQUIRED"})";
        } else {
          // Bounded by the existing 8192-byte Beast request-body limit.
          const auto payload = nlohmann::json::parse(request.body(), nullptr, false);
          if (payload.is_discarded()) {
            response->result(http::status::bad_request);
            response->body() = R"({"detail":"INVALID_JSON"})";
          } else {
            const auto preview = NativeMissionPreview::validate(payload);
            response->result(static_cast<http::status>(preview.http_status));
            response->body() = preview.body.dump();
          }
        }
      }
    } else if (request.method() == http::verb::post &&
               request.target() == "/api/native/mission/stop") {
      // Dedicated authenticated decision-side STOP; no positive action access.
      if (!sessions_->configured()) {
        response->result(http::status::service_unavailable);
        response->body() = R"({"detail":"NATIVE_SESSION_DB_NOT_CONFIGURED"})";
      } else {
        std::string token;
        auto auth = request.find(http::field::authorization);
        if (auth != request.end()) {
          auto v = auth->value().to_string();
          if (v.size() >= 7 && v.compare(0,7,"Bearer ") == 0)
            token = v.substr(7);
        }
        if (token.empty()) {
          auto t = request.find("X-Rover-Token");
          if (t != request.end()) token = t->value().to_string();
        }
        auto session = sessions_->verify(token);
        if (!session.authenticated) {
          response->result(http::status::unauthorized);
          response->body() = R"({"detail":"AUTHENTICATION_REQUIRED"})";
        } else {
          dyx::rpp::BackendMissionApiRequest command{};
          command.method = "POST";
          command.path = dyx::rpp::kBackendMissionStopPath;
          command.principal.authenticated = true;
          command.principal.operator_session_id = session.session_id;
          command.mission_run_id = "native_stop";
          command.sequence = 1;
          auto result = mailbox_.submit_verified(std::move(command), true);
          if (result != NativeCommandMailbox::SubmitResult::accepted) {
            response->result(http::status::service_unavailable);
            response->body() = R"({"detail":"STOP_ENQUEUE_REJECTED"})";
          } else {
            response->result(http::status::accepted);
            response->body() = R"({"state":"STOP_QUEUED_DECISION_ONLY","physical_stop_confirmed":false,"px4_stop_confirmed":false})";
          }
        }
      }
    } else if (
        request.method() == http::verb::get &&
        request.target() == "/api/native/mission/status") {
      // This endpoint is a localhost-only, read-only diagnostic.
      // It exposes no authenticated mission control or secrets.
      const auto snapshot = snapshot_->read();

      response->result(
          snapshot.first
              ? http::status::ok
              : http::status::service_unavailable);
      response->body() = snapshot.second.dump();
    } else {
      // No unauthenticated mission, E-stop release,
      // spray or GNSS configuration API is exposed.
      response->result(http::status::service_unavailable);
      response->body() = nlohmann::json{
          {"detail", "NATIVE_API_NOT_COMMISSIONED"},
          {"mission_commands_enabled", false},
          {"motor_commands_enabled", false},
          {"spray_commands_enabled", false}
      }.dump();
    }

    response->prepare_payload();

    stream_.expires_after(std::chrono::seconds(2));

    http::async_write(
        stream_,
        *response,
        [self = shared_from_this(), response](
            beast::error_code,
            std::size_t) {
          beast::error_code ignored;
          self->stream_.socket().shutdown(
              tcp::socket::shutdown_both, ignored);
          self->stream_.socket().close(ignored);
        });
  }

  beast::tcp_stream stream_;
  std::shared_ptr<NativeStatusSnapshot> snapshot_;
  std::shared_ptr<NativeSessionAuth> sessions_;
  NativeCommandMailbox& mailbox_;
  beast::flat_buffer buffer_;
  http::request_parser<http::string_body> parser_;
};

class NativeHttpServer final {
 public:
  NativeHttpServer(
      std::uint16_t port,
      std::shared_ptr<NativeStatusSnapshot> snapshot,
      NativeCommandMailbox& mailbox)
      : acceptor_(ioc_), snapshot_(std::move(snapshot)), mailbox_(mailbox),
        sessions_(std::make_shared<NativeSessionAuth>(
            std::getenv("DYX_CPP17_AUTH_DB") ?
            std::getenv("DYX_CPP17_AUTH_DB") : "")) {
    if (!snapshot_)
      throw std::invalid_argument("HTTP status snapshot required");
    if (port == 0)
      throw std::invalid_argument("invalid HTTP port");

    const tcp::endpoint endpoint(
        asio::ip::make_address("127.0.0.1"), port);

    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(
        asio::socket_base::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen(asio::socket_base::max_listen_connections);

    accept_next();

    worker_ = std::thread([this]() {
      ioc_.run();
    });
  }

  NativeHttpServer(const NativeHttpServer&) = delete;
  NativeHttpServer& operator=(const NativeHttpServer&) = delete;

  ~NativeHttpServer() {
    beast::error_code ignored;
    acceptor_.close(ignored);
    ioc_.stop();

    if (worker_.joinable())
      worker_.join();
  }

 private:
  void accept_next() {
    acceptor_.async_accept(
        [this](
            beast::error_code ec,
            tcp::socket socket) {
          if (!ec) {
            std::make_shared<HttpSession>(
                std::move(socket), snapshot_, sessions_, mailbox_)->start();
          }

          if (acceptor_.is_open())
            accept_next();
        });
  }

  std::shared_ptr<NativeStatusSnapshot> snapshot_;
  std::shared_ptr<NativeSessionAuth> sessions_;
  NativeCommandMailbox& mailbox_;
  asio::io_context ioc_{1};
  tcp::acceptor acceptor_;
  std::thread worker_;
};

}  // namespace dyx::http_backend
