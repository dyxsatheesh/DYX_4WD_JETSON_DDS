#pragma once
// Read-only validator for the EXISTING Python backend's SQLite sessions.
// Does not create users, mint tokens, change sessions or confer a control lease.
#include <sqlite3.h>
#include <openssl/sha.h>
#include <array>
#include <cctype>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
namespace dyx::http_backend {
struct NativeVerifiedSession {
  bool authenticated{false};
  std::string session_id{};
  std::string username{};
};
class NativeSessionAuth final {
 public:
  explicit NativeSessionAuth(std::string path):db_path_(std::move(path)){}
  bool configured() const {return !db_path_.empty();}
  NativeVerifiedSession verify(const std::string& token) const {
    NativeVerifiedSession out;
    if (db_path_.empty() || token.empty() || token.size()>512) return out;
    sqlite3* raw=nullptr;
    if (sqlite3_open_v2(db_path_.c_str(),&raw,SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX,nullptr)!=SQLITE_OK) {
      if (raw) sqlite3_close(raw);
      return out;
    }
    std::unique_ptr<sqlite3,decltype(&sqlite3_close)> db(raw,sqlite3_close);
    sqlite3_busy_timeout(db.get(),100);
    const char* sql = "SELECT s.session_id,u.username FROM sessions AS s JOIN users AS u ON u.id=s.user_id WHERE s.token_hash=?1 AND s.revoked_at IS NULL AND s.expires_at>?2 AND u.active=1 LIMIT 1";
    sqlite3_stmt* stmt_raw=nullptr;
    if(sqlite3_prepare_v2(db.get(),sql,-1,&stmt_raw,nullptr)!=SQLITE_OK)return out;
    std::unique_ptr<sqlite3_stmt,decltype(&sqlite3_finalize)> stmt(stmt_raw,sqlite3_finalize);
    std::array<unsigned char,SHA256_DIGEST_LENGTH> digest{};
    SHA256(reinterpret_cast<const unsigned char*>(token.data()),token.size(),digest.data());
    const char* digits="0123456789abcdef";
    std::string hex;hex.reserve(64);
    for(unsigned char b:digest){hex+=digits[b>>4];hex+=digits[b&0x0f];}
    const std::time_t now=std::time(nullptr);
    if(now<=0)return out;
    if(sqlite3_bind_text(stmt.get(),1,hex.c_str(),-1,SQLITE_TRANSIENT)!=SQLITE_OK ||
       sqlite3_bind_int64(stmt.get(),2,static_cast<sqlite3_int64>(now))!=SQLITE_OK)return out;
    if(sqlite3_step(stmt.get())==SQLITE_ROW){
      auto sid=sqlite3_column_text(stmt.get(),0);
      auto usr=sqlite3_column_text(stmt.get(),1);
      if(sid&&usr){out.authenticated=true;out.session_id=reinterpret_cast<const char*>(sid);out.username=reinterpret_cast<const char*>(usr);}
    }
    return out;
  }
 private:
  const std::string db_path_;
};
} // namespace dyx::http_backend
