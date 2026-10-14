#pragma once
// Real 127.0.0.1 TCP client; synthetic/unloaded mission; NO hardware outputs.
#include "dyx_rpp_cpp17_core/loopback_readonly_http_server.hpp"
#include <cassert>
#include <string>
using namespace dyx::rpp;

struct LoopbackFixture {
  NativeMissionCommandGateway gateway;
  NativeBackendMissionApi mission{gateway};
  NativeBackendLiveStatus live{gateway};
  NativeBackendHttpWire wire{mission,live};
  NativeLoopbackReadOnlyHttpServer listener{wire,{true,true,false,"SESSION_LOCAL_DEBUG"}};
};
inline std::string loopback_request(std::uint16_t port, const std::string& request) {
  const auto client=::socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
  assert(client!=offline_net::invalid_socket);
  offline_net::timeouts(client);
  sockaddr_in address{};address.sin_family=AF_INET;
  address.sin_port=htons(port);address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  assert(::connect(client,reinterpret_cast<const sockaddr*>(&address),sizeof(address))==0);
  std::size_t sent=0;
  while(sent<request.size()) {
    const int n=::send(client,request.data()+sent,static_cast<int>(request.size()-sent),0);
    assert(n>0);sent+=static_cast<std::size_t>(n);
  }
  std::string response;
  for(;;) {
    char buf[1024];const int n=::recv(client,buf,sizeof(buf),0);
    if(n<=0) break;
    response.append(buf,static_cast<std::size_t>(n));
    assert(response.size()<100000);
  }
  offline_net::close_socket(client);
  return response;
}
inline std::string loopback_get(std::uint16_t port,const std::string& path) {
  return loopback_request(port,"GET "+path+" HTTP/1.1\r\nHost: 127.0.0.1:"+
      std::to_string(port)+"\r\nConnection: close\r\n\r\n");
}
