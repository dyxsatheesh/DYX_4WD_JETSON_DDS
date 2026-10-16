#pragma once

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <nlohmann/json.hpp>

#include <cstdint>
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

class HttpSession final
    : public std::enable_shared_from_this<HttpSession> {
 public:
  explicit HttpSession(tcp::socket socket)
      : stream_(std::move(socket)) {}

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
  beast::flat_buffer buffer_;
  http::request_parser<http::string_body> parser_;
};

class NativeHttpServer final {
 public:
  explicit NativeHttpServer(std::uint16_t port)
      : acceptor_(ioc_) {
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
                std::move(socket))->start();
          }

          if (acceptor_.is_open())
            accept_next();
        });
  }

  asio::io_context ioc_{1};
  tcp::acceptor acceptor_;
  std::thread worker_;
};

}  // namespace dyx::http_backend
