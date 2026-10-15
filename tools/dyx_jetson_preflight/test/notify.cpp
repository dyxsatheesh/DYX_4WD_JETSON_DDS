#include "systemd_notify.hpp"
#include <cassert>
#include <cstdlib>
#include <string>
#if defined(__linux__)
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstdio>
#endif
int main(){
  using namespace dyx::preflight;
  ::setenv("WATCHDOG_USEC","x",1);
  unsigned long long us=0;
  assert(watchdog_requested());assert(!watchdog_usecs(us));
  ::setenv("WATCHDOG_USEC","15000000",1);
  assert(watchdog_usecs(us)); assert(us==15000000ULL);
#if defined(__linux__)
  std::string path="/tmp/dyx_notify_"+std::to_string(getpid());
  int fd=socket(AF_UNIX,SOCK_DGRAM,0);assert(fd>=0);
  sockaddr_un addr{};addr.sun_family=AF_UNIX;
  assert(path.size()<sizeof(addr.sun_path));
  std::snprintf(addr.sun_path,sizeof(addr.sun_path),"%s",path.c_str());
  assert(bind(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))==0);
  ::setenv("NOTIFY_SOCKET",path.c_str(),1);
  assert(notify_systemd("READY=1"));
  char buffer[40]{};auto n=recv(fd,buffer,sizeof(buffer),0);
  assert(n==7 && std::string(buffer,static_cast<size_t>(n))=="READY=1");
  close(fd);unlink(path.c_str());
#endif
}
