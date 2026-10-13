#pragma once
// Phase 2H3A: C++17 FILE-BACKED write-ahead actuator transaction journal.
// Persist ON identity before the corresponding DDS request is transmitted.
// Restart always treats an unfinished ON/OFF as UNCERTAIN, never as success.
// OFF response alone cannot clear the lock: independent physical OFF proof
// plus correlated RELEASE acceptance are required. No hardware is driven.
// Single-process/single-writer only; production deployment MUST additionally
// enforce directory security, exclusive ownership, backup/recovery policy and
// confirm power-loss durability on the deployment filesystem.
#include "dyx_rpp_cpp17_core/px4_actuator_dds_intent.hpp"
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#if defined(_WIN32)
#include <io.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace dyx::rpp {

struct JournalActuatorLock {
  DdsActuatorOwner owner{};
  std::string identity{};
  std::uint64_t last_sequence{0};
  // Persisted unique incrementing OFF dispatch ticket per transaction.
  // Unlike FSM command_sequence this survives process restarts.
  std::uint64_t dispatch_generation{1};
  // False until a matching, accepted OFF transport response is observed.
  bool off_response_accepted{false};
};

class FileBackedActuatorJournal final {
 public:
  explicit FileBackedActuatorJournal(std::filesystem::path file) : file_(std::move(file)) {
    if (file_.empty() || file_.filename().empty())
      throw std::invalid_argument("actuator journal path required");
    load(); // Fail-closed on any corrupt/inaccessible existing file.
  }
  bool recovery_required() const { return active_.has_value(); }
  const std::optional<JournalActuatorLock> &active_lock() const { return active_; }
  bool identity_used(DdsActuatorOwner owner, const std::string &identity) const {
    return used_.count({static_cast<int>(owner),identity})!=0;
  }
  std::size_t used_count() const { return used_.size(); }

  // PRE-CONDITION for an ON dispatch. If this throws, NEVER transmit ON.
  void persist_before_on(const DdsActuatorIntent &command) {
    require_valid(command);
    if (command.action!=DdsActuatorAction::on || active_ ||
        identity_used(command.owner,command.identity) || used_.size()>=100000)
      throw std::logic_error("ON blocked by uncertain transaction or replay");
    auto next_used=used_;
    next_used.insert({static_cast<int>(command.owner),command.identity});
    const JournalActuatorLock next_active{command.owner,command.identity,
                                         command.local_sequence,1,false};
    persist(next_active,next_used);
    used_.swap(next_used);
    active_=next_active;
  }
  // OFF is allowed after an uncertain ON, including after process restart.
  // It must be journalled before transmission, and cannot change ownership.
  std::uint64_t persist_before_off(const DdsActuatorIntent &command) {
    require_valid(command);
    if (command.action!=DdsActuatorAction::off || !active_ ||
        active_->owner!=command.owner || active_->identity!=command.identity ||
        active_->dispatch_generation==std::numeric_limits<std::uint64_t>::max())
      throw std::logic_error("OFF does not match uncertain actuator owner");
    auto next=*active_;
    // The FSM sequence may restart at 1 after a power cycle. Correlation
    // therefore uses a separately durable journal dispatch generation.
    next.last_sequence=command.local_sequence;
    ++next.dispatch_generation;
    next.off_response_accepted=false;
    persist(next,used_);
    active_=next;
    return next.dispatch_generation;
  }
  // This receipt MUST come from a single-flight DDS service/future matched
  // to the exact request, NOT a raw VehicleCommandAck filtered by cmd=187.
  void record_off_response(const DdsActuatorIntent &command,
                           std::uint64_t dispatch_generation, bool accepted) {
    require_matching_off(command,dispatch_generation);
    if (!accepted) return; // an unsuccessful ACK does not prove OFF
    auto next=*active_;
    next.off_response_accepted=true;
    persist(next,used_);
    active_=next;
  }
  // Both PX4 acceptance and separately established physical-safe-OFF are
  // needed; no command ACK or arming event is itself physical OFF feedback.
  void complete_off(const DdsActuatorIntent &command,
                    std::uint64_t dispatch_generation, bool independently_off_verified) {
    require_matching_off(command,dispatch_generation);
    if (!active_->off_response_accepted || !independently_off_verified)
      throw std::logic_error("cannot clear journal without independent OFF proof");
    persist(std::nullopt,used_);
    active_.reset();
  }

 private:
  using ReplayKey=std::pair<int,std::string>;
  static void require_valid(const DdsActuatorIntent &cmd) {
    if (cmd.local_sequence==0 || cmd.identity.empty() || cmd.identity.size()>256 ||
        cmd.vehicle_command!=187 ||
        (cmd.owner!=DdsActuatorOwner::mission_spray &&
         cmd.owner!=DdsActuatorOwner::manual_spray &&
         cmd.owner!=DdsActuatorOwner::laser) ||
        (cmd.action!=DdsActuatorAction::on && cmd.action!=DdsActuatorAction::off) ||
        (cmd.owner==DdsActuatorOwner::laser && cmd.actuator_set!=2) ||
        (cmd.owner!=DdsActuatorOwner::laser && cmd.actuator_set!=1) ||
        !std::isfinite(cmd.normalized_value) ||
        cmd.normalized_value < -1.0 || cmd.normalized_value > 1.0 ||
        !std::isfinite(cmd.params[6]) || cmd.params[6]!=0.0)
      throw std::invalid_argument("invalid actuator journal command");
    for (std::size_t i=0;i<6;++i) {
      if (i==cmd.actuator_set-1) {
        if (!std::isfinite(cmd.params[i]) || cmd.params[i]!=cmd.normalized_value)
          throw std::invalid_argument("journal selected actuator value mismatch");
      } else if (!std::isnan(cmd.params[i]))
        throw std::invalid_argument("journal request would change unrelated actuator");
    }
  }
  void require_matching_off(const DdsActuatorIntent &cmd,
                            std::uint64_t dispatch_generation) const {
    require_valid(cmd);
    if (cmd.action!=DdsActuatorAction::off || !active_ ||
        cmd.owner!=active_->owner || cmd.identity!=active_->identity ||
        cmd.local_sequence!=active_->last_sequence ||
        dispatch_generation!=active_->dispatch_generation)
      throw std::logic_error("unmatched OFF transport response");
  }
  static std::string hex(const std::string &bytes) {
    static constexpr char alphabet[]="0123456789abcdef";
    std::string out;
    out.reserve(bytes.size()*2);
    for (unsigned char b:bytes) {
      out.push_back(alphabet[b>>4]);out.push_back(alphabet[b&15U]);
    }
    return out;
  }
  static std::string unhex(const std::string &encoded) {
    if (encoded.empty() || encoded.size()%2!=0 || encoded.size()>512)
      throw std::runtime_error("corrupt actuator journal identity");
    const auto nib=[](char c)->int {
      if (c>='0' && c<='9') return c-'0';
      if (c>='a' && c<='f') return c-'a'+10;
      throw std::runtime_error("corrupt journal hex");
    };
    std::string out;
    for (std::size_t i=0;i<encoded.size();i+=2)
      out.push_back(static_cast<char>((nib(encoded[i])<<4)|nib(encoded[i+1])));
    return out;
  }
  static DdsActuatorOwner decode_owner(int n) {
    if (n<1 || n>3) throw std::runtime_error("corrupt actuator journal owner");
    return static_cast<DdsActuatorOwner>(n);
  }
  void load() {
    if (!std::filesystem::exists(file_)) return;
    if (!std::filesystem::is_regular_file(file_) ||
        std::filesystem::file_size(file_)>2*1024*1024)
      throw std::runtime_error("actuator journal inaccessible or too large");
    std::ifstream in(file_,std::ios::binary);
    std::string header;
    if (!std::getline(in,header) || header!="DYX_ACTUATOR_WAL_V2")
      throw std::runtime_error("actuator journal header invalid: fail closed");
    std::optional<JournalActuatorLock> active;
    std::set<ReplayKey> used;
    std::string line;
    bool seen_end=false;
    while (std::getline(in,line)) {
      if (line=="END") { seen_end=true;break; }
      std::istringstream ss(line);
      std::string tag,extra;
      int owner=0;
      std::string code;
      if (!(ss>>tag>>owner>>code)) throw std::runtime_error("corrupt WAL row");
      auto decoded=unhex(code);
      auto act_owner=decode_owner(owner);
      if (tag=="USED") {
        if (ss>>extra || !used.insert({owner,decoded}).second)
          throw std::runtime_error("duplicate or invalid WAL USED row");
      } else if(tag=="ACTIVE") {
        std::uint64_t seq=0, generation=0;
        int ack=0;
        if (active || !(ss>>seq>>generation>>ack) || (ss>>extra) || seq==0 ||
            generation==0 || (ack!=0 && ack!=1))
          throw std::runtime_error("corrupt WAL ACTIVE row");
        active=JournalActuatorLock{act_owner,decoded,seq,generation,ack==1};
      } else throw std::runtime_error("unknown WAL row");
    }
    if (!seen_end || (in>>line) || (active && !used.count(
        {static_cast<int>(active->owner),active->identity})))
      throw std::runtime_error("truncated or inconsistent actuator WAL");
    active_=active;used_=std::move(used);
  }
  void persist(const std::optional<JournalActuatorLock> &active,
               const std::set<ReplayKey> &used) const {
    const auto parent=file_.has_parent_path()?file_.parent_path():std::filesystem::path(".");
    if (!std::filesystem::exists(parent) || !std::filesystem::is_directory(parent))
      throw std::runtime_error("actuator journal parent does not exist");
    const auto temp=file_.string()+".tmp";
    std::string body="DYX_ACTUATOR_WAL_V2\n";
    if (active) body+="ACTIVE "+std::to_string(static_cast<int>(active->owner))+" "+
        hex(active->identity)+" "+std::to_string(active->last_sequence)+" "+
        std::to_string(active->dispatch_generation)+" "+
        (active->off_response_accepted?"1":"0")+"\n";
    for (const auto &key:used)
      body+="USED "+std::to_string(key.first)+" "+hex(key.second)+"\n";
    body+="END\n";
    std::FILE *f=nullptr;
#if defined(_WIN32)
    if (fopen_s(&f,temp.c_str(),"wb")!=0) f=nullptr;
#else
    f=std::fopen(temp.c_str(),"wb");
#endif
    if (!f) throw std::runtime_error("cannot open actuator WAL temporary file");
    const bool written=std::fwrite(body.data(),1,body.size(),f)==body.size();
    const bool flushed=written && std::fflush(f)==0;
    bool synced=false;
    if (flushed) {
#if defined(_WIN32)
      synced=_commit(_fileno(f))==0;
#else
      synced=::fsync(fileno(f))==0;
#endif
    }
    const bool closed=std::fclose(f)==0;
    if (!written || !flushed || !synced || !closed)
      throw std::runtime_error("actuator WAL write/flush/sync failed: press forbidden");
    std::filesystem::rename(temp,file_);
#if !defined(_WIN32)
    // Sync parent directory metadata so the new journal name survives crash.
    int dirfd=::open(parent.c_str(),O_RDONLY|O_DIRECTORY);
    if (dirfd<0) throw std::runtime_error("cannot sync actuator WAL directory");
    const bool dir_synced=::fsync(dirfd)==0;
    ::close(dirfd);
    if (!dir_synced) throw std::runtime_error("actuator WAL directory sync failed");
#endif
  }
  std::filesystem::path file_;
  std::optional<JournalActuatorLock> active_{};
  std::set<ReplayKey> used_{};
};
} // namespace dyx::rpp
