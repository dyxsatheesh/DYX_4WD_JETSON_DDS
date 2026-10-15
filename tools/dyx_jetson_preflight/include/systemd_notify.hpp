#pragma once
// Linux-only AF_UNIX notification: no network, ROS, DDS or hardware I/O.
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <cstddef>
#include <string>
#if defined(__linux__)
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif
namespace dyx::preflight {
inline bool notify_systemd(const char* msg) noexcept {
#if defined(__linux__)
  const char* p = std::getenv("NOTIFY_SOCKET");
  if(!p || !*p || std::strlen(p) >= sizeof(sockaddr_un::sun_path)) return false;
  sockaddr_un addr{};addr.sun_family=AF_UNIX;
  std::strncpy(addr.sun_path,p,sizeof(addr.sun_path)-1);
  const bool abstract = p[0]=='@';
  if(abstract) addr.sun_path[0]='\0';
  auto len = static_cast<socklen_t>(offsetof(sockaddr_un,sun_path)+std::strlen(p)+(abstract?0:1));
  int fd=::socket(AF_UNIX,SOCK_DGRAM | SOCK_CLOEXEC,0);
  if(fd<0) return false;
  const ssize_t sent=::sendto(fd,msg,std::strlen(msg),MSG_NOSIGNAL,
                             reinterpret_cast<const sockaddr*>(&addr),len);
  ::close(fd);
  return sent==static_cast<ssize_t>(std::strlen(msg));
#else
  (void)msg; return false;
#endif
}
inline bool watchdog_usecs(unsigned long long& microseconds) noexcept {
  const char* e=std::getenv("WATCHDOG_USEC");
  if(!e||!*e) return false;
  errno=0;char* end=nullptr;
  const auto v=std::strtoull(e,&end,10);
  if(errno || end==e || *end || v<1000000ULL || v>60000000ULL) return false;
#if defined(__linux__)
  const char* pid=std::getenv("WATCHDOG_PID");
  if(pid && *pid){
    errno=0;char* t=nullptr;const auto n=std::strtoul(pid,&t,10);
    if(errno||t==pid||*t||n!=static_cast<unsigned long>(::getpid())) return false;
  }
#endif
  microseconds=v; return true;
}
inline bool watchdog_requested() noexcept {
  const char* p=std::getenv("WATCHDOG_USEC");return p&&*p;
}
} // namespace dyx::preflight
