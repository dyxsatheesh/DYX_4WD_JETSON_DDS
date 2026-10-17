#pragma once
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace dyx::rtcm {
class SerialRtcmSink {
public:
 explicit SerialRtcmSink(std::string device,int baud=230400,int write_timeout_ms=1000)
   :device_(std::move(device)),baud_(baud),timeout_ms_(write_timeout_ms){
   if(device_.rfind("/dev/",0)!=0||device_.size()>256||device_.find("..")!=std::string::npos ||
      device_.find_first_of("\r\n\t")!=std::string::npos||timeout_ms_<=0||timeout_ms_>5000)
     throw std::invalid_argument("Invalid RTCM serial device or write timeout");
   if(baud_!=115200&&baud_!=230400&&baud_!=460800)throw std::invalid_argument("Unsupported RTCM baud");
 }
 ~SerialRtcmSink(){close();}
 SerialRtcmSink(const SerialRtcmSink&)=delete;
 SerialRtcmSink&operator=(const SerialRtcmSink&)=delete;
 bool open(){
   if(fd_>=0)return true;
   int candidate=::open(device_.c_str(),O_WRONLY|O_NOCTTY|O_NONBLOCK|O_CLOEXEC|O_NOFOLLOW);
   if(candidate<0)return false;
   if(::flock(candidate,LOCK_EX|LOCK_NB)!=0){::close(candidate);return false;}
   termios t{};
   if(::tcgetattr(candidate,&t)!=0){::close(candidate);return false;}
   ::cfmakeraw(&t);
   const speed_t speed=baud_==115200?B115200:(baud_==230400?B230400:B460800);
   ::cfsetispeed(&t,speed);::cfsetospeed(&t,speed);
   t.c_cflag|=CLOCAL|CREAD;
   t.c_cflag&=~CRTSCTS;
   if(::tcsetattr(candidate,TCSANOW,&t)!=0){::close(candidate);return false;}
   fd_=candidate;
   return true;
 }
 void close() noexcept {if(fd_>=0){::flock(fd_,LOCK_UN);::close(fd_);fd_=-1;}}
 bool is_open()const{return fd_>=0;}
 std::uint64_t frames_written()const{return frames_;}
 std::uint64_t bytes_written()const{return bytes_;}
 // On failure NEVER replay the frame from byte zero after a partial write.
 bool write_frame(const std::vector<std::uint8_t>&frame){
   if(fd_<0||frame.size()<6||frame.size()>1029)return false;
   const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms_);
   std::size_t offset=0;
   while(offset<frame.size()){
     const auto remain=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-std::chrono::steady_clock::now()).count();
     if(remain<=0){close();return false;}
     pollfd p{fd_,POLLOUT,0};
     const int r=::poll(&p,1,static_cast<int>(remain));
     if(r<=0||!(p.revents&POLLOUT)){close();return false;}
     const ssize_t n=::write(fd_,frame.data()+offset,frame.size()-offset);
     if(n<0&&(errno==EINTR||errno==EAGAIN))continue;
     if(n<=0){close();return false;}
     offset+=static_cast<std::size_t>(n);
   }
   ++frames_;bytes_+=frame.size();return true;
 }
private:
 std::string device_;int baud_,timeout_ms_,fd_{-1};
 std::uint64_t frames_{0},bytes_{0};
};
} // namespace dyx::rtcm
