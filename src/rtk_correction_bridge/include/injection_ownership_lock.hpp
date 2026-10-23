#pragma once
// Process-wide cooperative RTCM writer lock, compatible with the archived
// Python injection_lock.py and legacy rover-rtk-injection.lock users.
// Acquired ONLY when serial injection is explicitly commissioned/enabled.
#include <cerrno>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <unistd.h>

namespace dyx::rtcm {
class InjectionOwnershipLock final {
 public:
  static constexpr const char *kPath="/run/lock/rover-rtk-injection.lock";
  InjectionOwnershipLock() {
    int fd=::open(kPath, O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW, 0640);
    if(fd<0) throw std::runtime_error("RTCM_INJECTION_LOCK_OPEN_FAILED");
    if(::flock(fd,LOCK_EX|LOCK_NB)!=0) {
      const int saved=errno;
      ::close(fd);
      if(saved==EAGAIN || saved==EWOULDBLOCK || saved==EACCES)
        throw std::runtime_error("RTCM_INJECTION_OWNERSHIP_CONFLICT");
      throw std::runtime_error("RTCM_INJECTION_LOCK_FAILURE");
    }
    fd_=fd;
  }
  ~InjectionOwnershipLock() noexcept {
    if(fd_>=0){::flock(fd_,LOCK_UN);::close(fd_);}
  }
  InjectionOwnershipLock(const InjectionOwnershipLock&)=delete;
  InjectionOwnershipLock& operator=(const InjectionOwnershipLock&)=delete;
  bool held() const noexcept {return fd_>=0;}
 private:
  int fd_{-1};
};
} // namespace dyx::rtcm
