// DYX native C++17 NTRIP RTCM receiver. No serial/PX4/actuator publishers.
#include "rtcm3.hpp"
#include "serial_rtcm_sink.hpp"
#include "ntrip_protocol.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
namespace {
using Clock=std::chrono::steady_clock;
double seconds(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
struct Fd {int fd{-1};~Fd(){if(fd>=0)::close(fd);}Fd(const Fd&)=delete;Fd&operator=(const Fd&)=delete;Fd()=default;};
struct Tls {SSL_CTX* ctx{};SSL* ssl{};~Tls(){if(ssl){SSL_shutdown(ssl);SSL_free(ssl);}if(ctx)SSL_CTX_free(ctx);}Tls(const Tls&)=delete;Tls&operator=(const Tls&)=delete;Tls()=default;};
int connect_tcp(const std::string&host,int port){
  addrinfo hints{};hints.ai_family=AF_UNSPEC;hints.ai_socktype=SOCK_STREAM;
  addrinfo* raw=nullptr;const std::string service=std::to_string(port);
  if(getaddrinfo(host.c_str(),service.c_str(),&hints,&raw)!=0)throw std::runtime_error("NTRIP DNS failed");
  std::unique_ptr<addrinfo,decltype(&freeaddrinfo)> addresses(raw,freeaddrinfo);
  for(auto* ai=addresses.get();ai;ai=ai->ai_next){
    Fd fd;fd.fd=::socket(ai->ai_family,ai->ai_socktype|SOCK_CLOEXEC,ai->ai_protocol);
    if(fd.fd<0)continue;
    const int flags=fcntl(fd.fd,F_GETFL,0);if(flags<0||fcntl(fd.fd,F_SETFL,flags|O_NONBLOCK)<0)continue;
    int result=::connect(fd.fd,ai->ai_addr,ai->ai_addrlen);
    if(result<0&&errno!=EINPROGRESS)continue;
    pollfd pfd{fd.fd,POLLOUT,0};if(result<0&&::poll(&pfd,1,4000)<=0)continue;
    int error=0;socklen_t len=sizeof(error);
    if(getsockopt(fd.fd,SOL_SOCKET,SO_ERROR,&error,&len)!=0||error!=0)continue;
    // Blocking I/O with bounded send/receive (TLS handshake also bounded).
    if(fcntl(fd.fd,F_SETFL,flags)<0)continue;
    timeval tv{2,0};setsockopt(fd.fd,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv));
    setsockopt(fd.fd,SOL_SOCKET,SO_SNDTIMEO,&tv,sizeof(tv));
    int connected=fd.fd;fd.fd=-1;return connected;
  }
  throw std::runtime_error("NTRIP TCP connect timeout/failure");
}
}
class NtripObserver final: public rclcpp::Node {
public:
 NtripObserver():Node("rtk_correction_bridge"){
  enabled_=declare_parameter<bool>("receive_enabled",false);
  host_=declare_parameter<std::string>("caster_host","");
  mount_=declare_parameter<std::string>("mountpoint","");
  user_=declare_parameter<std::string>("username","");
  pass_=declare_parameter<std::string>("password","");
  port_=declare_parameter<int>("caster_port",2101);
  tls_required_=declare_parameter<bool>("tls_required",true);
  serial_enabled_=declare_parameter<bool>("serial_injection_enabled",false);
  serial_commissioned_=declare_parameter<bool>("serial_commissioned",false);
  serial_device_=declare_parameter<std::string>("serial_device","");
  serial_baud_=declare_parameter<int>("serial_baud",230400);
  if(serial_enabled_ && (!serial_commissioned_ || !enabled_ || serial_device_.empty()))
    throw std::invalid_argument("RTCM_SERIAL_REQUIRES_EXPLICIT_COMMISSIONING_AND_NTRIP");
  if(serial_enabled_)serial_=std::make_unique<dyx::rtcm::SerialRtcmSink>(serial_device_,serial_baud_);
  if(port_<1||port_>65535)throw std::invalid_argument("Invalid caster port");
  if(enabled_){(void)dyx::rtcm::request(host_,mount_,user_,pass_);worker_=std::thread([this]{loop();});}
  pub_=create_publisher<std_msgs::msg::String>("/rtk_correction_bridge/status",10);
  timer_=create_wall_timer(1s,[this]{
    std::lock_guard<std::mutex> lock(mu_);
    std_msgs::msg::String m;
    m.data="CXX17_NTRIP;receiving="+std::string(receiving_?"true":"false")+
      ";valid_rtcm_frames="+std::to_string(valid_)+
      ";crc_rejected="+std::to_string(bad_)+
      ";last_frame_age_sec="+(last_frame_>0?std::to_string(seconds()-last_frame_):std::string("unavailable"))+
      ";status="+state_+
      ";receiver_injection_enabled="+std::string(serial_enabled_?"true":"false")+
      ";serial_frames_written="+std::to_string(serial_frames_)+
      ";serial_write_faults="+std::to_string(serial_faults_)+
      ";rtk_fixed_verified=false;motion_authorized=false";
    pub_->publish(m);
  });
 }
 ~NtripObserver()override{stop_=true;if(worker_.joinable())worker_.join();}
private:
 void update(const std::string&s,bool rx){std::lock_guard<std::mutex>l(mu_);state_=s;receiving_=rx;}
 void record(const dyx::rtcm::Parser& parser,std::size_t new_frames){
   std::lock_guard<std::mutex>l(mu_);valid_+=new_frames;bad_=parser.counters().crc_bad;
   if(new_frames)last_frame_=seconds();
 }
 void session(){
  Fd sock;sock.fd=connect_tcp(host_,port_);
  Tls tls;
  if(tls_required_){
    tls.ctx=SSL_CTX_new(TLS_client_method());if(!tls.ctx)throw std::runtime_error("TLS_CTX_FAILED");
    SSL_CTX_set_min_proto_version(tls.ctx,TLS1_2_VERSION);
    if(SSL_CTX_set_default_verify_paths(tls.ctx)!=1)throw std::runtime_error("TLS_CA_STORE_FAILED");
    SSL_CTX_set_verify(tls.ctx,SSL_VERIFY_PEER,nullptr);
    tls.ssl=SSL_new(tls.ctx);if(!tls.ssl)throw std::runtime_error("TLS_NEW_FAILED");
    if(SSL_set_tlsext_host_name(tls.ssl,host_.c_str())!=1||SSL_set1_host(tls.ssl,host_.c_str())!=1)
      throw std::runtime_error("TLS_HOSTNAME_VALIDATION_SETUP_FAILED");
    SSL_set_fd(tls.ssl,sock.fd);
    if(SSL_connect(tls.ssl)!=1||SSL_get_verify_result(tls.ssl)!=X509_V_OK)
      throw std::runtime_error("NTRIP_TLS_CERTIFICATE_OR_HANDSHAKE_FAILED");
  }
  const auto send=[&](const std::string&s){
    std::size_t sent=0;
    while(sent<s.size()&&!stop_){
      const int n=tls_required_?SSL_write(tls.ssl,s.data()+sent,int(s.size()-sent)):
          int(::send(sock.fd,s.data()+sent,s.size()-sent,MSG_NOSIGNAL));
      if(n<=0)throw std::runtime_error("NTRIP_REQUEST_SEND_FAILED");sent+=std::size_t(n);
    }
  };
  send(dyx::rtcm::request(host_,mount_,user_,pass_));
  dyx::rtcm::Parser parser;
  std::vector<std::uint8_t> header;header.reserve(8192);
  bool ready=false;std::uint8_t chunk[2048];
  update("CONNECTED_WAITING_RTCM",true);
  while(!stop_){
    int n=tls_required_?SSL_read(tls.ssl,chunk,sizeof(chunk)):
      int(::recv(sock.fd,chunk,sizeof(chunk),0));
    if(n<=0)throw std::runtime_error("NTRIP_STREAM_CLOSED_OR_TIMED_OUT");
    std::size_t offset=0;
    if(!ready){
      std::size_t consumed=0;
      while(consumed<std::size_t(n)&&!ready){
        if(header.size()>=8192)throw std::runtime_error("NTRIP_HEADER_TOO_LARGE");
        header.push_back(chunk[consumed++]);
        if(header.size()>=4&&header[header.size()-4]=='\r'&&header[header.size()-3]=='\n'&&
           header[header.size()-2]=='\r'&&header.back()=='\n'){
          (void)dyx::rtcm::response_header(header);ready=true;
        }
      }
      offset=consumed;
    }
    if(ready&&offset<std::size_t(n)){
      const auto frames=parser.feed(chunk+offset,std::size_t(n)-offset,seconds());
      record(parser,frames.size());
      if(serial_){
        for(const auto &f:frames){
          if(!serial_->is_open() && !serial_->open()){
            {std::lock_guard<std::mutex> lock(mu_);++serial_faults_;}
            throw std::runtime_error("RTCM_SERIAL_OPEN_FAILED");
          }
          if(!serial_->write_frame(f.bytes)){
            {std::lock_guard<std::mutex> lock(mu_);++serial_faults_;}
            throw std::runtime_error("RTCM_SERIAL_WRITE_FAILED");
          }
          {std::lock_guard<std::mutex> lock(mu_);++serial_frames_;}
        }
      }
    }
  }
 }
 void loop(){
  while(!stop_){
    try{session();}catch(const std::exception&){update("DISCONNECTED_OR_RETRYING",false);}
    // Capped reconnect wait; do not busy-loop on invalid credentials.
    for(int i=0;i<30&&!stop_;++i)std::this_thread::sleep_for(100ms);
  }
  update("STOPPED",false);
 }
 std::atomic<bool> stop_{false};std::thread worker_;
 std::mutex mu_;std::string state_{"DISABLED"};bool receiving_{false},enabled_{false},tls_required_{true};
 std::string host_,mount_,user_,pass_,serial_device_;int port_{2101},serial_baud_{230400};
 bool serial_enabled_{false},serial_commissioned_{false};
 std::unique_ptr<dyx::rtcm::SerialRtcmSink> serial_;
 std::uint64_t serial_frames_{0},serial_faults_{0};
 std::uint64_t valid_{0},bad_{0};double last_frame_{-1};
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<NtripObserver>());}catch(const std::exception&e){RCLCPP_ERROR(rclcpp::get_logger("rtk_correction_bridge"),"Startup failed: %s",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
