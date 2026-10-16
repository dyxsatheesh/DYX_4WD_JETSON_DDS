#include "dyx_mission_manager/native_session_auth.hpp"
#include <sqlite3.h>
#include <openssl/sha.h>
#include <cassert>
#include <ctime>
#include <iostream>
#include <string>
#include <cstdio>
#include <unistd.h>
using dyx::http_backend::NativeSessionAuth;
int main() {
  const std::string path="/tmp/dyx_cpp17_auth_test_"+std::to_string(getpid())+".db";
  sqlite3* db=nullptr; assert(sqlite3_open(path.c_str(),&db)==SQLITE_OK);
  auto run=[&](const char* sql){char* error=nullptr;int rc=sqlite3_exec(db,sql,nullptr,nullptr,&error);if(error)sqlite3_free(error);assert(rc==SQLITE_OK);};
  run("CREATE TABLE users(id INTEGER PRIMARY KEY,username TEXT,active INTEGER);");
  run("CREATE TABLE sessions(session_id TEXT,token_hash TEXT,user_id INTEGER,expires_at INTEGER,revoked_at INTEGER);");
  run("INSERT INTO users VALUES(1,'satheesh',1);");
  run("INSERT INTO users VALUES(2,'disabled',0);");
  auto add=[&](std::string tok,const char* sid,int uid,long long expiry,const char* revoked){
    unsigned char digest[SHA256_DIGEST_LENGTH];SHA256((const unsigned char*)tok.data(),tok.size(),digest);
    const char* digits="0123456789abcdef";std::string hash;for(unsigned char b:digest){hash+=digits[b>>4];hash+=digits[b&15];}
    sqlite3_stmt* st=nullptr;
    assert(sqlite3_prepare_v2(db,"INSERT INTO sessions VALUES(?1,?2,?3,?4,?5)",-1,&st,nullptr)==SQLITE_OK);
    sqlite3_bind_text(st,1,sid,-1,SQLITE_TRANSIENT);sqlite3_bind_text(st,2,hash.c_str(),-1,SQLITE_TRANSIENT);
    sqlite3_bind_int(st,3,uid);sqlite3_bind_int64(st,4,expiry);
    if(revoked)sqlite3_bind_int(st,5,1);else sqlite3_bind_null(st,5);
    assert(sqlite3_step(st)==SQLITE_DONE);sqlite3_finalize(st);
  };
  const long long now=static_cast<long long>(std::time(nullptr));
  add("valid_tok","s1",1,now+10000,nullptr);
  add("expired_tok","s2",1,now-1,nullptr);
  add("revoked_tok","s3",1,now+10000,"yes");
  add("disabled_tok","s4",2,now+10000,nullptr);
  sqlite3_close(db);
  NativeSessionAuth auth(path);
  const auto good=auth.verify("valid_tok");
  assert(good.authenticated&&good.session_id=="s1"&&good.username=="satheesh");
  for(auto token:{"bad","expired_tok","revoked_tok","disabled_tok",""})assert(!auth.verify(token).authenticated);
  assert(!NativeSessionAuth("").verify("valid_tok").authenticated);
  std::remove(path.c_str());
  std::cout<<"PASS: Native shared SQLite auth, expiry, revocation and inactive user tests\n";
}
