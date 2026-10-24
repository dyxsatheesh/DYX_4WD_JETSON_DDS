#pragma once
// DYX Phase 2: Drogon-only HTTP authentication against existing Python SQLite schema.
// No users are created here; no command or actuator authority is granted.
#include <argon2.h>
#include <openssl/rand.h>
#include <openssl/sha.h>
#include <sqlite3.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace dyx::drogon_auth {
struct Result {
  int code{503};
  std::string detail{"AUTH_DATABASE_UNAVAILABLE"};
  std::string token{};
  std::string session_id{};
  std::string username{};
  std::int64_t expires_at{0};
  bool success{false};
};
class Store {
 public:
  explicit Store(std::string db_path) : path_(std::move(db_path)) {}
  bool configured() const { return !path_.empty(); }
  Result login(std::string user, const std::string& pass,
               std::string ip, std::string agent) {
    Result out;
    if (!configured()) { out.detail="NATIVE_SESSION_DB_NOT_CONFIGURED"; return out; }
    if (user.empty() || user.size()>128 || pass.empty() || pass.size()>512) {
      out.code=401;out.detail="Invalid username or password.";return out;
    }
    trim_lower(user);
    if (user.empty()) { out.code=401;out.detail="Invalid username or password.";return out; }
    if (ip.empty()) ip="unknown";
    if (ip.size()>128) ip.resize(128);
    if (agent.empty()) agent="unknown";
    if (agent.size()>512) agent.resize(512);
    std::lock_guard<std::mutex> lock(mu_);
    auto db=open_db();
    if (!db) return out;
    if (!exec(db.get(),"BEGIN IMMEDIATE")) return out;
    const auto rollback=[&]{exec(db.get(),"ROLLBACK");};
    const auto now=static_cast<std::int64_t>(std::time(nullptr));
    if (now<=0) {rollback();return out;}
    // Conservative built-in fallback: 5 failures/5 min, 15 min lockout.
    // Do not silently relax the Python backend's production policy.
    constexpr std::int64_t window=300,lockout=900;
    constexpr int limit=5;
    int failures=0;
    std::int64_t latest=0;
    {
      auto q=prepare(db.get(),"SELECT COUNT(*),COALESCE(MAX(failed_at),0) FROM login_failures WHERE username=?1 AND client_ip=?2 AND failed_at>=?3");
      if (!q || !bind(q.get(),1,user) || !bind(q.get(),2,ip) || sqlite3_bind_int64(q.get(),3,now-window)!=SQLITE_OK) {rollback();return out;}
      if (sqlite3_step(q.get())!=SQLITE_ROW) {rollback();return out;}
      failures=sqlite3_column_int(q.get(),0);latest=sqlite3_column_int64(q.get(),1);
    }
    if (failures>=limit && now<latest+lockout) {
      rollback();out.code=429;out.detail="Too many failed login attempts.";return out;
    }
    if (failures>=limit) {
      auto q=prepare(db.get(),"DELETE FROM login_failures WHERE username=?1 AND client_ip=?2");
      if (!q || !bind(q.get(),1,user) || !bind(q.get(),2,ip) || sqlite3_step(q.get())!=SQLITE_DONE) {rollback();return out;}
    }
    sqlite3_int64 user_id=0;
    std::string stored_hash;
    bool active=false;
    {
      auto q=prepare(db.get(),"SELECT id,password_hash,active FROM users WHERE username=?1 LIMIT 1");
      if (!q || !bind(q.get(),1,user)) {rollback();return out;}
      const auto rc=sqlite3_step(q.get());
      if (rc==SQLITE_ROW) {
        user_id=sqlite3_column_int64(q.get(),0);
        auto h=sqlite3_column_text(q.get(),1);
        if (h) stored_hash=reinterpret_cast<const char*>(h);
        active=sqlite3_column_int(q.get(),2)==1;
      } else if (rc!=SQLITE_DONE) {rollback();return out;}
    }
    bool good=false;
    if(active && !stored_hash.empty() && stored_hash.size()<1024 && stored_hash.rfind("$argon2id$",0)==0)
      good=argon2id_verify(stored_hash.c_str(),pass.data(),pass.size())==ARGON2_OK;
    if(!good) {
      auto q=prepare(db.get(),"INSERT INTO login_failures(username,client_ip,failed_at) VALUES(?1,?2,?3)");
      if (!q || !bind(q.get(),1,user) || !bind(q.get(),2,ip) || sqlite3_bind_int64(q.get(),3,now)!=SQLITE_OK || sqlite3_step(q.get())!=SQLITE_DONE || !exec(db.get(),"COMMIT")) {rollback();return out;}
      out.code=401;out.detail="Invalid username or password.";return out;
    }
    std::array<unsigned char,48> random_token{};
    std::array<unsigned char,16> random_session{};
    if (RAND_bytes(random_token.data(),random_token.size())!=1 || RAND_bytes(random_session.data(),random_session.size())!=1) {rollback();return out;}
    const std::string token=hex(random_token.data(),random_token.size());
    const std::string session=hex(random_session.data(),random_session.size());
    std::array<unsigned char,SHA256_DIGEST_LENGTH> digest{};
    SHA256(reinterpret_cast<const unsigned char*>(token.data()),token.size(),digest.data());
    const std::string token_hash=hex(digest.data(),digest.size());
    // Default one-day expiry; configure to match settings.session_ttl_seconds.
    const auto ttl=session_ttl();
    const auto expires=now+ttl;
    {
      auto q=prepare(db.get(),"DELETE FROM login_failures WHERE username=?1 AND client_ip=?2");
      if (!q || !bind(q.get(),1,user) || !bind(q.get(),2,ip) || sqlite3_step(q.get())!=SQLITE_DONE) {rollback();return out;}
    }
    {
      auto q=prepare(db.get(),"INSERT INTO sessions(session_id,token_hash,user_id,created_at,expires_at,last_seen_at,revoked_at,client_ip,user_agent) VALUES(?1,?2,?3,?4,?5,?4,NULL,?6,?7)");
      if (!q || !bind(q.get(),1,session) || !bind(q.get(),2,token_hash) || sqlite3_bind_int64(q.get(),3,user_id)!=SQLITE_OK || sqlite3_bind_int64(q.get(),4,now)!=SQLITE_OK || sqlite3_bind_int64(q.get(),5,expires)!=SQLITE_OK || !bind(q.get(),6,ip) || !bind(q.get(),7,agent) || sqlite3_step(q.get())!=SQLITE_DONE) {rollback();return out;}
    }
    if(!exec(db.get(),"COMMIT")) {rollback();return out;}
    out.code=200;out.success=true;out.detail="OK";out.token=token;out.session_id=session;out.username=user;out.expires_at=expires;
    return out;
  }
  bool logout(const std::string& token) {
    if (!configured() || token.empty() || token.size()>512) return false;
    std::lock_guard<std::mutex> lock(mu_);
    auto db=open_db();if(!db)return false;
    std::array<unsigned char,SHA256_DIGEST_LENGTH> digest{};
    SHA256(reinterpret_cast<const unsigned char*>(token.data()),token.size(),digest.data());
    auto q=prepare(db.get(),"UPDATE sessions SET revoked_at=?1 WHERE token_hash=?2 AND revoked_at IS NULL AND expires_at>?1");
    if (!q)return false;
    const auto now=static_cast<std::int64_t>(std::time(nullptr));
    if(sqlite3_bind_int64(q.get(),1,now)!=SQLITE_OK || !bind(q.get(),2,hex(digest.data(),digest.size())) || sqlite3_step(q.get())!=SQLITE_DONE)return false;
    return sqlite3_changes(db.get())==1;
  }
 private:
  using Db=std::unique_ptr<sqlite3,decltype(&sqlite3_close)>;
  using Stmt=std::unique_ptr<sqlite3_stmt,decltype(&sqlite3_finalize)>;
  static void trim_lower(std::string& s) {
    const auto start=s.find_first_not_of(" \t\r\n");
    if(start==std::string::npos){s.clear();return;}
    const auto end=s.find_last_not_of(" \t\r\n");
    s=s.substr(start,end-start+1);
    std::transform(s.begin(),s.end(),s.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
  }
  static std::string hex(const unsigned char* p,std::size_t n) {
    constexpr char digits[]="0123456789abcdef";
    std::string out;out.reserve(n*2);
    for(std::size_t i=0;i<n;i++){out+=digits[p[i]>>4];out+=digits[p[i]&15];}
    return out;
  }
  static Stmt prepare(sqlite3* db,const char* sql) {
    sqlite3_stmt* raw=nullptr;
    if(sqlite3_prepare_v2(db,sql,-1,&raw,nullptr)!=SQLITE_OK) return Stmt(nullptr,sqlite3_finalize);
    return Stmt(raw,sqlite3_finalize);
  }
  static bool bind(sqlite3_stmt* q,int index,const std::string& v) {
    return sqlite3_bind_text(q,index,v.c_str(),static_cast<int>(v.size()),SQLITE_TRANSIENT)==SQLITE_OK;
  }
  static bool exec(sqlite3* db,const char* sql) {return sqlite3_exec(db,sql,nullptr,nullptr,nullptr)==SQLITE_OK;}
  Db open_db() const {
    sqlite3* raw=nullptr;
    if(sqlite3_open_v2(path_.c_str(),&raw,SQLITE_OPEN_READWRITE | SQLITE_OPEN_NOMUTEX,nullptr)!=SQLITE_OK) {
      if(raw)sqlite3_close(raw);
      return Db(nullptr,sqlite3_close);
    }
    sqlite3_busy_timeout(raw,250);
    return Db(raw,sqlite3_close);
  }
  static std::int64_t session_ttl() {
    const char* env=std::getenv("DYX_DROGON_SESSION_TTL_SECONDS");
    if(!env)return 86400;
    try {auto v=std::stoll(env);if(v>=60 && v<=604800)return v;}catch(...){}
    return 86400;
  }
  std::string path_;
  std::mutex mu_;
};
} // namespace dyx::drogon_auth
