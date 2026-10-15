#include "replay_fence.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <string>
namespace fs=std::filesystem;
int main(){
  const auto d=fs::temp_directory_path() / ("dyx_replay_"+std::to_string(getpid())+
       "_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  assert(fs::create_directory(d));assert(chmod(d.c_str(),0700)==0);
  const auto path=(d/"counter.sqlite").string();
  dyx::preflight::SignedRequest a{"operator_01","abcdef0123456789",1};
  {
    dyx::preflight::PersistentReplayFence db(path);assert(db.ready());
    assert(db.accept(a));assert(!db.accept(a));
    dyx::preflight::PersistentReplayFence competing(path);
    assert(!competing.ready());
    a.sequence=3;assert(db.accept(a));
    a.sequence=2;assert(!db.accept(a));
  }
  {
    dyx::preflight::PersistentReplayFence db(path);assert(db.ready());
    a.sequence=3;assert(!db.accept(a));
    a.sequence=4;assert(db.accept(a));
    a.session="operator_02";a.sequence=1;assert(db.accept(a));
  }
  fs::remove_all(d);
}
