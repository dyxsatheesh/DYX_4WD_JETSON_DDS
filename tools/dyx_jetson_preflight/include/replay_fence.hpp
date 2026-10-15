#pragma once
// H5X: transactionally persisted, single-process SQLite replay fence.
// No actuator writes. One owner per database with flock().
#include "verified_http.hpp"
#include <sqlite3.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
namespace dyx::preflight {
inline bool safe_owned_parent(const std::string& path){
  if(path.size()<3||path[0]!='/')return false;
  auto slash=path.rfind('/');if(slash==std::string::npos||slash==0)return false;
  const auto dir=path.substr(0,slash);
  struct stat st{};
  return lstat(dir.c_str(),&st)==0 && S_ISDIR(st.st_mode) &&
         st.st_uid==geteuid() && (st.st_mode&0077)==0;
}
inline bool safe_owned_regular(const std::string& path){
  struct stat st{};
  if(lstat(path.c_str(),&st)!=0)return errno==ENOENT;
  return S_ISREG(st.st_mode)&&st.st_uid==geteuid()&&(st.st_mode&0077)==0;
}
class PersistentReplayFence final {
public:
  explicit PersistentReplayFence(const std::string& path){
    if(!safe_owned_parent(path)||!safe_owned_regular(path) ||
       !safe_owned_regular(path+".owner"))return;
    owner_=open((path+".owner").c_str(),O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);
    if(owner_<0||flock(owner_,LOCK_EX|LOCK_NB)!=0)return;
    mode_t old=umask(0077);
    int rc=sqlite3_open_v2(path.c_str(),&db_,
                  SQLITE_OPEN_READWRITE|SQLITE_OPEN_CREATE|SQLITE_OPEN_FULLMUTEX,nullptr);
    umask(old);
    if(rc!=SQLITE_OK||!db_||!safe_owned_regular(path))return;
    sqlite3_busy_timeout(db_,1000);
    const char* init="PRAGMA journal_mode=DELETE;PRAGMA synchronous=FULL;"
                     "CREATE TABLE IF NOT EXISTS replay_fence ("
                     "session TEXT PRIMARY KEY NOT NULL,seq INTEGER NOT NULL CHECK(seq>0));";
    valid_=sqlite3_exec(db_,init,nullptr,nullptr,nullptr)==SQLITE_OK;
  }
  ~PersistentReplayFence(){if(db_)sqlite3_close_v2(db_);
    if(owner_>=0){flock(owner_,LOCK_UN);close(owner_);}}
  PersistentReplayFence(const PersistentReplayFence&)=delete;
  PersistentReplayFence& operator=(const PersistentReplayFence&)=delete;
  bool ready() const noexcept{return valid_;}
  bool accept(const SignedRequest& r){
    if(!valid_||!safe_id(r.session,8,128)||r.sequence==0||
       r.sequence>static_cast<std::uint64_t>(INT64_MAX))return false;
    if(sqlite3_exec(db_,"BEGIN IMMEDIATE",nullptr,nullptr,nullptr)!=SQLITE_OK){valid_=false;return false;}
    sqlite3_stmt* stmt=nullptr;
    const char* sql="INSERT INTO replay_fence(session,seq) VALUES(?1,?2) "
                    "ON CONFLICT(session) DO UPDATE SET seq=excluded.seq "
                    "WHERE replay_fence.seq < excluded.seq";
    bool changed=false;
    if(sqlite3_prepare_v2(db_,sql,-1,&stmt,nullptr)==SQLITE_OK){
      if(sqlite3_bind_text(stmt,1,r.session.data(),static_cast<int>(r.session.size()),SQLITE_TRANSIENT)==SQLITE_OK&&
         sqlite3_bind_int64(stmt,2,static_cast<sqlite3_int64>(r.sequence))==SQLITE_OK&&
         sqlite3_step(stmt)==SQLITE_DONE)changed=sqlite3_changes(db_)==1;
    }
    if(stmt)sqlite3_finalize(stmt);
    if(!changed){sqlite3_exec(db_,"ROLLBACK",nullptr,nullptr,nullptr);return false;}
    if(sqlite3_exec(db_,"COMMIT",nullptr,nullptr,nullptr)!=SQLITE_OK){
      (void)sqlite3_exec(db_,"ROLLBACK",nullptr,nullptr,nullptr);
      valid_=false;return false;
    }
    return true;
  }
private:
  sqlite3* db_{nullptr};int owner_{-1};bool valid_{false};
};
} // namespace dyx::preflight
