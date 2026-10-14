#pragma once
#include "backend_loopback_fixture.hpp"
#include <chrono>
#include <thread>

inline offline_net::Socket h4k_connect(std::uint16_t port) {
  const auto client=::socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
  assert(client!=offline_net::invalid_socket);
  offline_net::timeouts(client);
  sockaddr_in addr{}; addr.sin_family=AF_INET;
  addr.sin_port=htons(port);addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  assert(::connect(client,reinterpret_cast<const sockaddr*>(&addr),sizeof(addr))==0);
  return client;
}
inline void h4k_send(offline_net::Socket client, const std::string& s) {
  std::size_t pos=0;
  while(pos<s.size()) {
#ifdef MSG_NOSIGNAL
    const int n=::send(client,s.data()+pos,static_cast<int>(s.size()-pos),MSG_NOSIGNAL);
#else
    const int n=::send(client,s.data()+pos,static_cast<int>(s.size()-pos),0);
#endif
    assert(n>0);pos+=static_cast<std::size_t>(n);
  }
}
inline std::string h4k_receive(offline_net::Socket client) {
  std::string result;
  for (;;) {
    char data[1024]; const int n=::recv(client,data,sizeof(data),0);
    if(n<=0) break;
    result.append(data,static_cast<std::size_t>(n));
    assert(result.size()<100000);
  }
  return result;
}
