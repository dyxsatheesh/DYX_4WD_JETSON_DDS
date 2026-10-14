#pragma once
// 2H4H: development-only HTTP/1.1 TCP listener. IPv4 loopback only, GET only.
// NEVER a production authentication/TLS ingress, DDS publisher or actuator.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>
#endif
#include "dyx_rpp_cpp17_core/backend_http_wire.hpp"
#include <atomic>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
namespace dyx::rpp {
namespace offline_net {
#ifdef _WIN32
using Socket = SOCKET;
constexpr Socket invalid_socket = INVALID_SOCKET;
inline void close_socket(Socket s) { if (s != invalid_socket) closesocket(s); }
struct Startup { Startup() { WSADATA d{}; if (WSAStartup(MAKEWORD(2, 2), &d)) throw std::runtime_error("Winsock startup failed"); } ~Startup(){ WSACleanup(); } };
#else
using Socket = int;
constexpr Socket invalid_socket = -1;
inline void close_socket(Socket s) { if (s != invalid_socket) ::close(s); }
struct Startup {};
#endif
inline void timeouts(Socket s) {
#ifdef _WIN32
  const DWORD ms = 1000;
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&ms), sizeof(ms));
  setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&ms), sizeof(ms));
#else
  timeval tv{}; tv.tv_sec = 1;
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
#endif
}
inline BackendHttpWireResponse error(int status, const char* code) {
  BackendHttpWireResponse r; r.status = status; r.body = http_json::error_json(code); return r;
}
struct ParsedGet { bool valid{false}; int status{400}; const char* reason{"MALFORMED_REQUEST"}; std::string path{}; };
inline ParsedGet parse_get(const std::string& text, std::uint16_t port) {
  ParsedGet bad;
  if (text.size() > 4096 || text.size() < 8 || text.compare(text.size()-4,4,"\r\n\r\n") != 0) return bad;
  for (unsigned char c: text)
    if ((c < 0x20 && c != '\r' && c != '\n' && c != '\t') || c == 0x7f) return bad;
  // CR/LF may appear only as canonical CRLF pairs (no bare control lines).
  for (std::size_t i=0;i<text.size();++i) {
    if (text[i]=='\r' && (i+1==text.size() || text[i+1]!='\n')) return bad;
    if (text[i]=='\n' && (i==0 || text[i-1]!='\r')) return bad;
  }
  const auto e = text.find("\r\n");
  if (e == std::string::npos) return bad;
  const auto first = text.substr(0,e);
  const auto a=first.find(' '), b=a==std::string::npos?a:first.find(' ',a+1);
  if (a==std::string::npos || b==std::string::npos || first.find(' ',b+1)!=std::string::npos || first.substr(b+1)!="HTTP/1.1") return bad;
  if (first.substr(0,a)!="GET") return {false,405,"READ_ONLY_METHOD_NOT_ALLOWED",{}};
  const auto path=first.substr(a+1,b-a-1);
  if (path.empty() || path[0]!='/' || path.size()>192 || path.find_first_of("?#%\\ \t\r\n")!=std::string::npos) return bad;
  bool saw_host=false;
  std::size_t pos=e+2;
  while (pos<text.size()) {
    const auto end=text.find("\r\n",pos);
    if (end==std::string::npos) return bad;
    if (end==pos) {
      if (end+2!=text.size() || !saw_host) return bad;
      return {true,200,"OK",path};
    }
    const auto line=text.substr(pos,end-pos);
    if (line[0]==' ' || line[0]=='\t') return bad;
    const auto colon=line.find(':');
    if (colon==std::string::npos || colon==0) return bad;
    std::string name=line.substr(0,colon);
    for (char& c:name) {
      if (!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='-')) return bad;
      if (c>='A'&&c<='Z') c=static_cast<char>(c+('a'-'A'));
    }
    std::string value=line.substr(colon+1);
    const auto begin=value.find_first_not_of(" \t");
    value = begin==std::string::npos ? "" : value.substr(begin);
    const auto last=value.find_last_not_of(" \t");
    if (last!=std::string::npos) value.erase(last+1);
    if (name=="content-length" || name=="transfer-encoding") return bad;
    if (name=="host") {
      if (saw_host) return bad;
      const auto suffix=":"+std::to_string(port);
      if (value!="localhost" && value!="127.0.0.1" && value!="localhost"+suffix && value!="127.0.0.1"+suffix) return bad;
      saw_host=true;
    }
    pos=end+2;
  }
  return bad;
}
inline void send_all(Socket c, const std::string& bytes) {
  std::size_t pos=0;
  while(pos<bytes.size()) {
    const auto left=bytes.size()-pos;
    const auto count=static_cast<int>(left>4096?4096:left);
    const int sent=::send(c,bytes.data()+pos,count,0);
    if(sent<=0) break;
    pos+=static_cast<std::size_t>(sent);
  }
}
} // namespace offline_net

// Underlying mission gateway must not be modified concurrently with listener.
// Fixed developer reader is injected by local test/application, NOT network.
class NativeLoopbackReadOnlyHttpServer final {
 public:
  explicit NativeLoopbackReadOnlyHttpServer(NativeBackendHttpWire& wire,
      BackendVerifiedPrincipal trusted_debug_reader={})
      : wire_(wire), reader_(std::move(trusted_debug_reader)) {
    reader_.owns_control_lease=false;
    if(!reader_.authenticated) reader_.can_read_mission=false;
  }
  NativeLoopbackReadOnlyHttpServer(const NativeLoopbackReadOnlyHttpServer&)=delete;
  NativeLoopbackReadOnlyHttpServer& operator=(const NativeLoopbackReadOnlyHttpServer&)=delete;
  ~NativeLoopbackReadOnlyHttpServer(){stop();}
  std::uint16_t start(std::uint16_t requested_port=0) {
    if(running_ || worker_.joinable()) throw std::logic_error("listener already running");
    const auto fd=::socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    if(fd==offline_net::invalid_socket) throw std::runtime_error("socket failed");
    sockaddr_in local{};
    local.sin_family=AF_INET;
    local.sin_addr.s_addr=htonl(INADDR_LOOPBACK); // NEVER INADDR_ANY
    local.sin_port=htons(requested_port);
    if(::bind(fd,reinterpret_cast<const sockaddr*>(&local),sizeof(local)) || ::listen(fd,4)) {
      offline_net::close_socket(fd); throw std::runtime_error("bind/listen failed");
    }
#ifdef _WIN32
    int len=sizeof(local);
#else
    socklen_t len=sizeof(local);
#endif
    if(::getsockname(fd,reinterpret_cast<sockaddr*>(&local),&len) ||
       local.sin_addr.s_addr!=htonl(INADDR_LOOPBACK)) {
      offline_net::close_socket(fd); throw std::runtime_error("loopback bind verification failed");
    }
    fd_=fd; port_=ntohs(local.sin_port); running_=true;
    try { worker_=std::thread([this]{run();}); }
    catch(...) { running_=false; offline_net::close_socket(fd_); fd_=offline_net::invalid_socket; port_=0; throw; }
    return port_;
  }
  void stop() noexcept {
    running_=false;
    if(worker_.joinable()) worker_.join();
    offline_net::close_socket(fd_);
    fd_=offline_net::invalid_socket;port_=0;
  }
  std::uint16_t port() const noexcept {return port_;}
 private:
  void run() noexcept {
    while(running_) {
      fd_set fds; FD_ZERO(&fds); FD_SET(fd_,&fds);
      timeval tv{}; tv.tv_usec=100000;
#ifdef _WIN32
      const int ready=::select(0,&fds,nullptr,nullptr,&tv);
#else
      const int ready=::select(fd_+1,&fds,nullptr,nullptr,&tv);
#endif
      if(ready<=0 || !running_) continue;
      sockaddr_in peer{};
#ifdef _WIN32
      int len=sizeof(peer);
#else
      socklen_t len=sizeof(peer);
#endif
      const auto client=::accept(fd_,reinterpret_cast<sockaddr*>(&peer),&len);
      if(client==offline_net::invalid_socket) continue;
      offline_net::timeouts(client);
      try {if(peer.sin_family==AF_INET && peer.sin_addr.s_addr==htonl(INADDR_LOOPBACK)) serve(client);}
      catch(...) {/* malformed clients cannot crash event loop */}
      offline_net::close_socket(client);
    }
  }
  void serve(offline_net::Socket client) {
    std::string header;
    bool complete=false, extra=false;
    while(header.size()<=4096 && running_) {
      char b[1024];const int n=::recv(client,b,sizeof(b),0);
      if(n<=0) break;
      header.append(b,static_cast<std::size_t>(n));
      const auto end=header.find("\r\n\r\n");
      if(end!=std::string::npos) {
        extra=end+4!=header.size(); header.resize(end+4); complete=true;break;
      }
    }
    BackendHttpWireResponse reply;
    if(!complete || header.size()>4096) reply=offline_net::error(413,"OVERSIZE_OR_INCOMPLETE_HEADERS");
    else if(extra) reply=offline_net::error(400,"REQUEST_BODY_NOT_ALLOWED");
    else {
      const auto parsed=offline_net::parse_get(header,port_);
      if(!parsed.valid) reply=offline_net::error(parsed.status,parsed.reason);
      else {
        BackendTrustedHttpRequest req;
        req.method="GET";req.path=parsed.path;req.principal=reader_;
        const auto now=std::chrono::steady_clock::now().time_since_epoch();
        req.host_received_sec=std::chrono::duration<double>(now).count();
        reply=wire_.handle(req); // wire: no physical actuation
      }
    }
    offline_net::send_all(client,reply.http11_bytes());
  }
  offline_net::Startup sockets_{};
  NativeBackendHttpWire& wire_;
  BackendVerifiedPrincipal reader_{};
  offline_net::Socket fd_{offline_net::invalid_socket};
  std::uint16_t port_{0};
  std::atomic<bool> running_{false};
  std::thread worker_{};
};
} // namespace dyx::rpp
