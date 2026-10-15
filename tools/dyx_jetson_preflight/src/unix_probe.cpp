// Opt-in real AF_UNIX HTTP health probe, no TCP, DDS, motor or spray objects.
#include "safety.hpp"
#include "verified_http.hpp"
#include "replay_fence.hpp"
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/types.h>
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
namespace {
volatile std::sig_atomic_t stopped=0;
extern "C" void finish(int){stopped=1;}
bool safe_socket_parent(const std::string& path){
  if(path.size()<3||path[0]!='/'||path.size()>=sizeof(sockaddr_un::sun_path))return false;
  auto slash=path.rfind('/');if(slash==std::string::npos||slash==0)return false;
  struct stat st{};
  std::string parent=path.substr(0,slash);
  return lstat(parent.c_str(),&st)==0&&S_ISDIR(st.st_mode)&&
         st.st_uid==geteuid()&&(st.st_mode&0077)==0;
}
std::string reply(int code,const std::string& json){
  std::string reason=code==200?"OK":code==401?"Unauthorized":code==409?"Conflict":"Locked";
  return "HTTP/1.1 "+std::to_string(code)+" "+reason+"\r\nContent-Type: application/json\r\n"
         "Connection: close\r\nCache-Control: no-store\r\nContent-Length: "+
         std::to_string(json.size())+"\r\n\r\n"+json;
}
}
int main(int argc,char** argv){
  if(argc!=3||std::string(argv[1])!="--serve-unix"){
    std::cerr<<"Usage: dyx_jetson_unix_probe --serve-unix /secure/dir/probe.sock\n";return 64;}
  const char* keypath=getenv("DYX_HMAC_KEY_FILE");
  std::vector<unsigned char> secret;
  if(!keypath||!dyx::preflight::load_owned_secret(keypath,secret)||
     !safe_socket_parent(argv[2])){std::cerr<<"Bad secret/socket directory\n";return 77;}
  const char* dbpath=getenv("DYX_REPLAY_DB");
  if(!dbpath){std::cerr<<"DYX_REPLAY_DB is required (fail closed)\n";return 77;}
  dyx::preflight::PersistentReplayFence fence(dbpath);
  if(!fence.ready()){std::cerr<<"Cannot open owned SQLite replay fence\n";return 77;}
  const std::string path=argv[2];
  struct stat existing{};
  if(lstat(path.c_str(),&existing)==0||errno!=ENOENT){std::cerr<<"Socket path already exists\n";return 73;}
  int fd=socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);
  if(fd<0)return 70;
  sockaddr_un addr{};addr.sun_family=AF_UNIX;
  std::memcpy(addr.sun_path,path.c_str(),path.size()+1);
  mode_t prev=umask(0077);
  bool bound=bind(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))==0;
  umask(prev);
  if(!bound){close(fd);return 70;}
  if(chmod(path.c_str(),0600)!=0||listen(fd,4)!=0){close(fd);unlink(path.c_str());return 70;}
  std::signal(SIGTERM,finish);std::signal(SIGINT,finish);
  std::cout<<"DYX local diagnostic socket active (NO PX4/DDS output)\n"<<std::flush;
  while(!stopped){
    pollfd p{fd,POLLIN,0};int n=poll(&p,1,200);
    if(n<=0)continue;
    int c=accept4(fd,nullptr,nullptr,SOCK_CLOEXEC);
    if(c<0)continue;
    struct ucred peer{};socklen_t plen=sizeof(peer);
    bool peer_ok=getsockopt(c,SOL_SOCKET,SO_PEERCRED,&peer,&plen)==0&&
                 peer.uid==geteuid();
    timeval timeout{0,500000};setsockopt(c,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
    char buf[2049]{};ssize_t nr=recv(c,buf,2049,0);
    dyx::preflight::SignedRequest request;
    bool valid=peer_ok&&nr>0&&nr<=2048&&
        dyx::preflight::verify_request(std::string(buf,static_cast<size_t>(nr)),secret,request);
    const bool accepted=valid && fence.accept(request);
    const auto message=accepted?reply(200,dyx::preflight::InhibitState{}.json()):
       valid?reply(409,"{\"error\":\"REPLAY_OR_STORAGE_UNAVAILABLE\"}"):
             reply(401,"{\"error\":\"UNAUTHORIZED\"}");
    (void)send(c,message.data(),message.size(),MSG_NOSIGNAL);close(c);
  }
  close(fd);unlink(path.c_str());
  OPENSSL_cleanse(secret.data(),secret.size());return 0;
}
