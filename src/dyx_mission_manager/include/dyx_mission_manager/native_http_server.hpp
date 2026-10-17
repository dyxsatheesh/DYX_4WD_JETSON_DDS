#pragma once

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <nlohmann/json.hpp>
#include "dyx_mission_manager/native_session_auth.hpp"
#include "dyx_mission_manager/native_command_mailbox.hpp"
#include "dyx_mission_manager/native_mission_preview.hpp"
#include "dyx_mission_manager/native_upload_ingress.hpp"
#include "dyx_mission_manager/native_load_ingress.hpp"
#include "dyx_mission_manager/native_execution_preflight.hpp"
#include "dyx_mission_manager/native_mission_command_preflight.hpp"
#include "dyx_mission_manager/native_command_decision_audit.hpp"

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
      NativeCommandMailbox& mailbox,
      NativeUploadIngress& ingress,
      NativeLoadIngress& loads,
      NativeCommandDecisionAudit& decisions)
      : stream_(std::move(socket)),
        snapshot_(std::move(snapshot)),
        sessions_(std::move(sessions)), mailbox_(mailbox), ingress_(ingress), loads_(loads), decisions_(decisions) {}

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
  NativeVerifiedSession verify_session(
      const http::request<http::string_body>& request) const {
    if (!sessions_->configured()) return {};
    std::string token;
    const auto authorization = request.find(http::field::authorization);
    if (authorization != request.end()) {
      const std::string value = authorization->value().to_string();
      if (value.size() >= 7 && value.compare(0, 7, "Bearer ") == 0)
        token = value.substr(7);
    }
    if (token.empty()) {
      const auto alternate = request.find("X-Rover-Token");
      if (alternate != request.end()) token = alternate->value().to_string();
    }
    return sessions_->verify(token);
  }

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
          {"authenticated_upload_staging_supported", true},
          {"exclusive_upload_lease_required", true},
          {"px4_disarm_attestation_required", true},
          {"preview_coordinate_frame", "LOCAL_ENU_METRES"},
          {"live_android_api_compatible", false},
          {"mission_load_enabled", false},
          {"mission_commands_enabled", false},
          {"motor_commands_enabled", false},
          {"spray_commands_enabled", false},
          {"physical_command_emitted", false}
      }.dump();
    } else if (request.method() == http::verb::get &&
               request.target() == "/api/native/mission/preflight") {
      // An authenticated, read-only report. All evidence originates from
      // the ROS snapshot, never the HTTP request body.
      const auto session = verify_session(request);
      if (!session.authenticated) {
        response->result(http::status::unauthorized);
        response->body() = R"({"detail":"AUTHENTICATION_REQUIRED"})";
      } else {
        const auto snapshot = snapshot_->read();
        if (!snapshot.first) {
          response->result(http::status::service_unavailable);
          response->body() = nlohmann::json{
              {"state","BLOCKED"},
              {"detail", "ROS_SNAPSHOT_STALE_OR_UNAVAILABLE"},
              {"start_enabled",false},
              {"ready_for_physical_motion",false},
              {"decision_only",true}}.dump();
        } else {
          response->result(http::status::ok);
          response->body() = NativeExecutionPreflight::evaluate(snapshot.second).dump();
        }
      }
    } else if (request.method() == http::verb::post &&
               (request.target() == "/api/native/mission/start" ||
                request.target() == "/api/native/mission/pause" ||
                request.target() == "/api/native/mission/resume")) {
      // Batch 7B: only authenticated, schema-valid commands reach preflight.
      // NO command is submitted to the mailbox or the ROS mission owner.
      const auto session = verify_session(request);
      if (!session.authenticated) {
        response->result(http::status::unauthorized);
        response->body() = R"({"detail":"AUTHENTICATION_REQUIRED"})";
      } else {
        const auto payload = nlohmann::json::parse(request.body(), nullptr, false);
        const auto cmd = NativeMissionCommandPreflight::validate(payload);
        if (!cmd.ok) {
          response->result(http::status::unprocessable_entity);
          response->body() = R"({"detail":"INVALID_COMMAND_ENVELOPE"})";
        } else {
          // This upload lease proves identity only, NEVER permission to move.
          // Even a forged optimistic snapshot cannot authorize execution.
          const auto upload_status = ingress_.status_for(
              session.session_id, mailbox_.stopped_latched());
          const bool upload_session_matches =
              upload_status.value("state", std::string{}) != "CONTROL_LEASE_REQUIRED" &&
              upload_status.value("state", std::string{}) != "STOP_LATCHED";
          if (mailbox_.stopped_latched()) {
            response->result(http::status::locked);
            response->body() = R"({"detail":"STOP_LATCHED","decision_only":true})";
          } else if (!upload_session_matches) {
            response->result(http::status::forbidden);
            response->body() = R"({"detail":"OPERATOR_SESSION_BINDING_REQUIRED","decision_only":true})";
          } else if (!decisions_.reserve(session.session_id, cmd.run_id,
                                          cmd.sequence, mailbox_.stopped_latched())) {
            response->result(http::status::conflict);
            response->body() = R"({"detail":"COMMAND_SEQUENCE_OR_RUN_CONFLICT","decision_only":true})";
          } else {
            dyx::rpp::BackendMissionApiRequest q{};
            q.method = "POST";
            q.path = request.target() == "/api/native/mission/start"
                ? dyx::rpp::kBackendMissionStartPath
                : (request.target() == "/api/native/mission/pause"
                   ? dyx::rpp::kBackendMissionPausePath
                   : dyx::rpp::kBackendMissionResumePath);
            q.principal.authenticated = true;
            q.principal.can_read_mission = true;
            q.principal.owns_control_lease = false; // Critical: NEVER grant motion.
            q.principal.operator_session_id = session.session_id;
            q.mission_run_id = cmd.run_id;
            q.sequence = cmd.sequence;
            const auto submitted = mailbox_.submit_verified(std::move(q), true);
            if (submitted == NativeCommandMailbox::SubmitResult::accepted) {
              response->result(http::status::accepted);
              response->body() = R"({"state":"QUEUED_FOR_ROS_SAFETY_DENIAL","decision_only":true,"mission_commands_enabled":false,"motor_commands_enabled":false,"spray_commands_enabled":false})";
            } else {
              response->result(http::status::conflict);
              response->body() = R"({"detail":"COMMAND_MAILBOX_FULL_OR_STOPPED","decision_only":true})";
            }
          }
        }
      }
    } else if (request.method() == http::verb::get &&
               request.target() == "/api/native/mission/commands/status") {
      const auto session = verify_session(request);
      if (!session.authenticated) {
        response->result(http::status::unauthorized);
        response->body() = R"({"detail":"AUTHENTICATION_REQUIRED"})";
      } else {
        const auto result = decisions_.status(session.session_id,
                                               mailbox_.stopped_latched());
        response->result(result.value("state", std::string{}) ==
                         "COMMAND_SESSION_REQUIRED" ? http::status::forbidden
                                                    : http::status::ok);
        response->body() = result.dump();
      }
    } else if (request.method() == http::verb::post &&
               request.target() == "/api/native/mission/lease/acquire") {
      if (!sessions_->configured()) {
        response->result(http::status::service_unavailable);
        response->body() = R"({"detail":"NATIVE_SESSION_DB_NOT_CONFIGURED"})";
      } else {
        const auto session = verify_session(request);
        if (!session.authenticated) {
          response->result(http::status::unauthorized);
          response->body() = R"({"detail":"AUTHENTICATION_REQUIRED"})";
        } else {
          const auto status = ingress_.acquire(
              session.session_id, mailbox_.stopped_latched());
          if (status == NativeUploadIngress::LeaseResult::acquired ||
              status == NativeUploadIngress::LeaseResult::already_owned) {
            response->result(http::status::ok);
            response->body() = R"({"state":"UPLOAD_LEASE_ACQUIRED","scope":"DRAFT_UPLOAD_ONLY","motion_authorized":false})";
          } else {
            response->result(http::status::locked);
            response->body() = R"({"detail":"UPLOAD_LEASE_CONFLICT_OR_STOP_LATCHED"})";
          }
        }
      }
    } else if (request.method() == http::verb::post &&
               request.target() == "/api/native/mission/upload") {
      if (!sessions_->configured()) {
        response->result(http::status::service_unavailable);
        response->body() = R"({"detail":"NATIVE_SESSION_DB_NOT_CONFIGURED"})";
      } else {
        const auto session = verify_session(request);
        if (!session.authenticated) {
          response->result(http::status::unauthorized);
          response->body() = R"({"detail":"AUTHENTICATION_REQUIRED"})";
        } else {
          // The schema is identical to PREVIEW. JSON cannot supply disarm
          // attestation, control lease, timestamps or a server-side sequence.
          auto payload = nlohmann::json::parse(request.body(), nullptr, false);
          if (payload.is_discarded()) {
            response->result(http::status::bad_request);
            response->body() = R"({"detail":"INVALID_JSON"})";
          } else {
            const auto checked = NativeMissionPreview::validate(payload);
            if (checked.http_status != 200) {
              response->result(static_cast<http::status>(checked.http_status));
              response->body() = checked.body.dump();
            } else {
              const auto submitted = ingress_.enqueue(
                  session.session_id, std::move(payload),
                  mailbox_.stopped_latched());
              if (submitted == NativeUploadIngress::EnqueueResult::accepted) {
                response->result(http::status::accepted);
                response->body() = R"({"state":"UPLOAD_QUEUED_FOR_ROS","decision_only":true,"mission_loaded":false,"motor_commands_enabled":false})";
              } else {
                response->result(http::status::conflict);
                response->body() = R"({"detail":"UPLOAD_LEASE_REQUIRED_OR_INBOX_BUSY_OR_STOPPED"})";
              }
            }
          }
        }
      }
    } else if (request.method() == http::verb::get &&
               request.target() == "/api/native/mission/upload/status") {
      if (!sessions_->configured()) {
        response->result(http::status::service_unavailable);
        response->body() = R"({"detail":"NATIVE_SESSION_DB_NOT_CONFIGURED"})";
      } else {
        const auto session = verify_session(request);
        if (!session.authenticated) {
          response->result(http::status::unauthorized);
          response->body() = R"({"detail":"AUTHENTICATION_REQUIRED"})";
        } else {
          const auto body = ingress_.status_for(
              session.session_id, mailbox_.stopped_latched());
          if (body.value("state", std::string{}) == "CONTROL_LEASE_REQUIRED") {
            response->result(http::status::forbidden);
          } else {
            response->result(http::status::ok);
          }
          response->body() = body.dump();
        }
      }
    } else if (request.method() == http::verb::post &&
               request.target() == "/api/native/mission/load") {
      const auto session = verify_session(request);
      if (!session.authenticated) {
        response->result(http::status::unauthorized);
        response->body() = R"({"detail":"AUTHENTICATION_REQUIRED"})";
      } else {
        const auto data = nlohmann::json::parse(request.body(), nullptr, false);
        const auto validated = NativeLoadIngress::validate(data);
        if (!validated.ok) {
          response->result(http::status::unprocessable_entity);
          response->body() = R"({"detail":"INVALID_LOAD_ENVELOPE"})";
        } else if (!ingress_.staged_for(session.session_id, validated.run_id,
                                       validated.signature, validated.revision)) {
          response->result(http::status::conflict);
          response->body() = R"({"detail":"STAGED_DRAFT_OR_OPERATOR_MISMATCH"})";
        } else {
          const auto queued = loads_.enqueue(session.session_id, validated,
                                             mailbox_.stopped_latched());
          response->result(queued ? http::status::accepted : http::status::conflict);
          response->body() = queued ?
              R"({"state":"LOAD_QUEUED_FOR_ROS","decision_only":true,"mission_loaded":false,"motor_commands_enabled":false})" :
              R"({"detail":"LOAD_BUSY_OR_STOP_LATCHED"})";
        }
      }
    } else if (request.method() == http::verb::get &&
               request.target() == "/api/native/mission/load/status") {
      const auto session = verify_session(request);
      if (!session.authenticated) {
        response->result(http::status::unauthorized);
        response->body() = R"({"detail":"AUTHENTICATION_REQUIRED"})";
      } else {
        response->result(http::status::ok);
        response->body() = loads_.status_for(session.session_id,
                                             mailbox_.stopped_latched()).dump();
      }
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
          if (result == NativeCommandMailbox::SubmitResult::accepted) {
            ingress_.cancel_on_stop();
            loads_.cancel_on_stop();
            decisions_.stop();
          }
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
  NativeUploadIngress& ingress_;
  NativeLoadIngress& loads_;
  NativeCommandDecisionAudit& decisions_;
  beast::flat_buffer buffer_;
  http::request_parser<http::string_body> parser_;
};

class NativeHttpServer final {
 public:
  NativeHttpServer(
      std::uint16_t port,
      std::shared_ptr<NativeStatusSnapshot> snapshot,
      NativeCommandMailbox& mailbox,
      NativeUploadIngress& ingress,
      NativeLoadIngress& loads,
      NativeCommandDecisionAudit& decisions)
      : acceptor_(ioc_), snapshot_(std::move(snapshot)), mailbox_(mailbox),
        ingress_(ingress), loads_(loads), decisions_(decisions),
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
                std::move(socket), snapshot_, sessions_, mailbox_,
                ingress_, loads_, decisions_)->start();
          }

          if (acceptor_.is_open())
            accept_next();
        });
  }

  std::shared_ptr<NativeStatusSnapshot> snapshot_;
  std::shared_ptr<NativeSessionAuth> sessions_;
  NativeCommandMailbox& mailbox_;
  NativeUploadIngress& ingress_;
  NativeLoadIngress& loads_;
  NativeCommandDecisionAudit& decisions_;
  asio::io_context ioc_{1};
  tcp::acceptor acceptor_;
  std::thread worker_;
};

}  // namespace dyx::http_backend
