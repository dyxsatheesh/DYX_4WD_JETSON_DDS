// DYX native Drogon C++17 backend phase 1: parity-safe read-only HTTP.
// Production mission authority stays with mission_manager; no actuator route.
#include "dyx_mission_manager/native_session_auth.hpp"
#include "dyx_drogon_auth.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <drogon/drogon.h>
#include <nlohmann/json.hpp>
#include <sstream>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

using namespace std::chrono_literals;
using ResponseCallback=std::function<void(const drogon::HttpResponsePtr&)>;

class RoverBackend final : public rclcpp::Node {
public:
  RoverBackend():Node("rover_backend") {
    const char* db=std::getenv("DYX_CPP17_AUTH_DB");
    sessions_=std::make_shared<dyx::http_backend::NativeSessionAuth>(db?db:"");
    auth_store_=std::make_shared<dyx::drogon_auth::Store>(db?db:"");
    mission_sub_=create_subscription<std_msgs::msg::String>(
      "/mission_manager/status",rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr m){
        std::lock_guard<std::mutex> lock(mu_);
        latest_status_=m->data.substr(0,8192);
        last_status_at_=std::chrono::steady_clock::now();
        status_seen_=true;
      });
    status_pub_=create_publisher<std_msgs::msg::String>("/rover_backend/migration_status",10);
    timer_=create_wall_timer(100ms,[this] {
      std_msgs::msg::String msg;
      msg.data="DROGON_CPP17_BACKEND;read_only_http_ready=true;mission_commands_enabled=false;"
               "motor_commands_enabled=false;spray_commands_enabled=false;"
               "physical_command_emitted=false";
      status_pub_->publish(msg);
    });
    const auto port=declare_parameter<int>("readonly_http_port",18767);
    if(port<1024||port>65535||port==18766)
      throw std::invalid_argument("invalid read-only backend port");

    // All listeners bind loopback until API parity and authentication are commissioned.
    drogon::app().registerHandler("/api/native/health",
      [this](const drogon::HttpRequestPtr& req,ResponseCallback&& cb){
        if(req->method()!=drogon::Get) {send(cb,drogon::k405MethodNotAllowed,{{"detail","METHOD_NOT_ALLOWED"}});return;}
        send(cb,drogon::k200OK,{{"service","dyx_cpp17_rover_backend"},
          {"state","DROGON_READ_ONLY_MIGRATION"},
          {"authentication_configured",sessions_->configured()},
          {"mission_commands_enabled",false},{"motor_commands_enabled",false},
          {"spray_commands_enabled",false},{"physical_command_emitted",false}});
      },{drogon::Get,drogon::Post,drogon::Put,drogon::Delete});

    auto authenticated=[this](const drogon::HttpRequestPtr& req,ResponseCallback&& cb,bool session_info){
      if(req->method()!=drogon::Get) {
        send(cb,drogon::k405MethodNotAllowed,{{"detail","METHOD_NOT_ALLOWED"}});return;
      }
      if(!sessions_->configured()) {
        send(cb,drogon::k503ServiceUnavailable,{{"detail","NATIVE_SESSION_DB_NOT_CONFIGURED"}});return;
      }
      std::string token;
      const auto auth=req->getHeader("authorization");
      if(auth.rfind("Bearer ",0)==0)token=auth.substr(7);
      if(token.empty())token=req->getHeader("x-rover-token");
      const auto session=sessions_->verify(token);
      if(!session.authenticated){
        send(cb,drogon::k401Unauthorized,{{"detail","AUTHENTICATION_REQUIRED"}});return;
      }
      if(session_info){
        send(cb,drogon::k200OK,{{"authenticated",true},
          {"session",{{"session_id",session.session_id},{"username",session.username}}},
          {"decision_only",true},{"mission_commands_enabled",false}});
      }else{
        std::string snapshot;
        double age=-1.0;
        {
          std::lock_guard<std::mutex> lk(mu_);
          if(status_seen_){
            age=std::chrono::duration<double>(std::chrono::steady_clock::now()-last_status_at_).count();
            snapshot=latest_status_;
          }
        }
        if(age<0.0||age>0.5){
          send(cb,drogon::k503ServiceUnavailable,{{"detail","MISSION_OWNER_STATUS_STALE_OR_MISSING"},
            {"mission_commands_enabled",false},{"ready_for_physical_motion",false}});return;
        }
        send(cb,drogon::k200OK,{{"mission_status_observation",snapshot},
          {"observation_age_sec",age},{"mission_commands_enabled",false},
          {"ready_for_physical_motion",false},{"physical_command_emitted",false}});
      }
    };
    drogon::app().registerHandler("/api/native/auth/session",
      [authenticated](const drogon::HttpRequestPtr& r,ResponseCallback&& cb){authenticated(r,std::move(cb),true);},
      {drogon::Get,drogon::Post,drogon::Put,drogon::Delete});
    drogon::app().registerHandler("/api/native/mission/status",
      [authenticated](const drogon::HttpRequestPtr& r,ResponseCallback&& cb){authenticated(r,std::move(cb),false);},
      {drogon::Get,drogon::Post,drogon::Put,drogon::Delete});
    // Do NOT create positive command routes in this early migration phase.

    drogon::app().registerHandler(
        "/api/native/mission/start",
        [this](const drogon::HttpRequestPtr&,
               ResponseCallback&& cb) {
          send(cb, drogon::k503ServiceUnavailable,
               {{"detail", "COMMAND_IPC_NOT_COMMISSIONED"},
                {"mission_commands_enabled", false},
                {"physical_command_emitted", false}});
        },
        {drogon::Post});

    // Phase 2: Python-compatible REST authentication. SQLite user provisioning
    // remains an explicit administrative process; login never creates users.
    drogon::app().registerHandler("/api/auth/login",
      [this](const drogon::HttpRequestPtr& req,ResponseCallback&& cb){
        if(req->getBody().size()>8192){send(cb,drogon::k400BadRequest,{{"detail","REQUEST_TOO_LARGE"}});return;}
        const auto body=req->getJsonObject();
        if(!body || !body->isObject() || !body->isMember("username") ||
           !body->isMember("password") || !(*body)["username"].isString() ||
           !(*body)["password"].isString()) {
          send(cb,drogon::k400BadRequest,{{"detail","INVALID_LOGIN_REQUEST"}});return;
        }
        // Restrict to loopback while authentication and Android parity are in development.
        const auto ip=req->peerAddr().toIp();
        const auto result=auth_store_->login(
          (*body)["username"].asString(),(*body)["password"].asString(),
          ip,req->getHeader("user-agent"));
        if(!result.success){
          const auto code=static_cast<drogon::HttpStatusCode>(result.code);
          send(cb,code,{{"detail",result.detail}});return;
        }
        send(cb,drogon::k200OK,{{"success",true},{"token",result.token},
          {"token_type","Bearer"},{"session_id",result.session_id},
          {"expires_at_epoch",result.expires_at},
          {"user",{{"username",result.username}}},
          {"mission_commands_enabled",false}});
      },{drogon::Post});

    drogon::app().registerHandler("/api/auth/session",
      [this](const drogon::HttpRequestPtr& req,ResponseCallback&& cb){
        if(!sessions_->configured()){
          send(cb,drogon::k503ServiceUnavailable,{{"detail","NATIVE_SESSION_DB_NOT_CONFIGURED"}});return;
        }
        std::string token=get_token(req);
        auto session=sessions_->verify(token);
        if(!session.authenticated){send(cb,drogon::k401Unauthorized,{{"detail","AUTHENTICATION_REQUIRED"}});return;}
        send(cb,drogon::k200OK,{{"authenticated",true},
          {"session",{{"session_id",session.session_id},{"username",session.username}}},
          {"mission_commands_enabled",false}});
      },{drogon::Get});

    drogon::app().registerHandler("/api/auth/logout",
      [this](const drogon::HttpRequestPtr& req,ResponseCallback&& cb){
        const auto token=get_token(req);
        const auto verified=sessions_->verify(token);
        if(!verified.authenticated){send(cb,drogon::k401Unauthorized,{{"detail","AUTHENTICATION_REQUIRED"}});return;}
        if(!auth_store_->logout(token)){
          send(cb,drogon::k503ServiceUnavailable,{{"detail","SESSION_REVOCATION_FAILED"}});return;
        }
        send(cb,drogon::k200OK,{{"success",true},{"message","Logged out successfully."}});
      },{drogon::Post});

    drogon::app().setThreadNum(2).addListener("127.0.0.1",static_cast<uint16_t>(port));
    server_=std::thread([]{drogon::app().run();});

    RCLCPP_WARN(get_logger(),"Drogon C++17 read-only backend on loopback:%ld; motion disabled",port);
  }
  ~RoverBackend() override {
    drogon::app().quit();
    if(server_.joinable())server_.join();
  }
private:
  static std::string get_token(const drogon::HttpRequestPtr& req){
    const std::string header=req->getHeader("authorization");
    if(header.rfind("Bearer ",0)==0)return header.substr(7);
    return req->getHeader("x-rover-token");
  }
  static void send(const ResponseCallback& cb,drogon::HttpStatusCode status,const nlohmann::json& body){
    Json::Value payload;
    Json::CharReaderBuilder parser;
    std::string errors;
    std::istringstream stream(body.dump());
    if(!Json::parseFromStream(parser,stream,&payload,&errors))payload=Json::objectValue;
    auto response=drogon::HttpResponse::newHttpJsonResponse(payload);
    response->setStatusCode(status);
    cb(response);
  }
  std::shared_ptr<dyx::http_backend::NativeSessionAuth> sessions_;
  std::shared_ptr<dyx::drogon_auth::Store> auth_store_;
  std::mutex mu_;
  std::string latest_status_;
  std::chrono::steady_clock::time_point last_status_at_{};
  bool status_seen_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread server_;
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<RoverBackend>());}
  catch(const std::exception& e){
    RCLCPP_ERROR(rclcpp::get_logger("rover_backend"),"Drogon startup failure: %s",e.what());
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
