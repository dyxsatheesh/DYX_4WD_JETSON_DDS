#pragma once
#include <cerrno>
#include <cstdint>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>
namespace dyx::rtcm {
class ReceiverAgeSerial final {
public:
 explicit ReceiverAgeSerial(std::string device,int baud):device_(std::move(device)),baud_(baud) {
  if(device_.rfind("/dev/",0)!=0||device_.size()>256||device_.find("..")!=std::string::npos||device_.find_first_of("\r\n\t")!=std::string::npos)
    throw std::invalid_argument("receiver_age_device must be an absolute /dev path");
  if(baud_!=115200&&baud_!=230400&&baud_!=460800)throw std::invalid_argument("Unsupported receiver age baud");
 }
 ~ReceiverAgeSerial(){close();}
 ReceiverAgeSerial(const ReceiverAgeSerial&)=delete;
 ReceiverAgeSerial&operator=(const ReceiverAgeSerial&)=delete;
 bool open_read_only(){
  if(fd_>=0)return true;
  int candidate=::open(device_.c_str(),O_RDONLY|O_NOCTTY|O_NONBLOCK|O_CLOEXEC|O_NOFOLLOW);
  if(candidate<0)return false;
  if(::flock(candidate,LOCK_EX|LOCK_NB)!=0){::close(candidate);return false;}
  termios t{};
  if(::tcgetattr(candidate,&t)!=0){::close(candidate);return false;}
  ::cfmakeraw(&t);
  const speed_t speed=baud_==115200?B115200:(baud_==230400?B230400:B460800);
  ::cfsetispeed(&t,speed);::cfsetospeed(&t,speed);
  t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CRTSCTS;
  if(::tcsetattr(candidate,TCSANOW,&t)!=0){::close(candidate);return false;}
  fd_=candidate;return true;
 }
 // Returns bytes read, zero for polling timeout, -1 for broken port.
 int read_some(std::uint8_t* data,std::size_t length,int timeout_ms=200){
  if(fd_<0||!data||length==0||length>2048)return -1;
  pollfd p{fd_,POLLIN,0};const int r=::poll(&p,1,timeout_ms);
  if(r==0)return 0;
  if(r<0)return errno==EINTR?0:-1;
  if(!(p.revents&POLLIN))return -1;
  const ssize_t n=::read(fd_,data,length);
  if(n<0&&(errno==EINTR||errno==EAGAIN))return 0;
  return n>0?static_cast<int>(n):-1;
 }
 void close()noexcept{if(fd_>=0){::flock(fd_,LOCK_UN);::close(fd_);fd_=-1;}}
private:std::string device_;int baud_,fd_{-1};
};
}
