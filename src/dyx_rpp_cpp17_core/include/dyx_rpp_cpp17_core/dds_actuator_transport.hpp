#pragma once
// Phase 2H3B: C++17 DDS actuator transport BOUNDARY (NON-ACTUATING).
// A caller may publish only a returned DispatchTicket using an independently
// audited, hardware-gated ROS2 adapter. The core itself has NO ROS2 API.
// PX4 VehicleCommandAck has no per-request sequence/transaction field:
// command==187 alone NEVER authorizes a positive receipt or WAL completion.
#include "dyx_rpp_cpp17_core/actuator_write_ahead_journal.hpp"
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace dyx::rpp {

struct VehicleCommandWire {
  std::uint64_t timestamp_us{0};
  float param1{},param2{},param3{},param4{};
  double param5{},param6{};
  float param7{};
  std::uint32_t command{187};
  std::uint8_t target_system{1},target_component{1};
  std::uint8_t source_system{1};
  std::uint16_t source_component{191};
  std::uint8_t confirmation{0};
  bool from_external{true};
};
inline VehicleCommandWire px4_wire_command(const DdsActuatorIntent &intent,
     std::uint64_t px4_timestamp_us, std::uint8_t target_system=1,
     std::uint8_t target_component=1, std::uint8_t source_system=1,
     std::uint16_t source_component=191) {
  if (intent.vehicle_command!=187 || intent.local_sequence==0 ||
      intent.identity.empty() || intent.identity.size()>256 ||
      (intent.owner!=DdsActuatorOwner::mission_spray &&
       intent.owner!=DdsActuatorOwner::manual_spray &&
       intent.owner!=DdsActuatorOwner::laser) ||
      (intent.action!=DdsActuatorAction::on &&
       intent.action!=DdsActuatorAction::off) ||
      !std::isfinite(intent.normalized_value) ||
      px4_timestamp_us==0 ||
      target_system==0 || source_system==0 || source_component==0 ||
      !std::isfinite(intent.params[6]) || intent.params[6]!=0 ||
      intent.actuator_set<1 || intent.actuator_set>2 ||
      (intent.owner==DdsActuatorOwner::laser && intent.actuator_set!=2) ||
      (intent.owner!=DdsActuatorOwner::laser && intent.actuator_set!=1))
    throw std::invalid_argument("unsafe PX4 DDS command metadata");
  for (std::size_t i=0;i<6;++i) {
    if (i == intent.actuator_set-1) {
      if (!std::isfinite(intent.params[i]) ||
          intent.params[i]!=intent.normalized_value ||
          intent.params[i]<-1.0 || intent.params[i]>1.0)
        throw std::invalid_argument("invalid selected actuator set");
    } else if (!std::isnan(intent.params[i])) {
      throw std::invalid_argument("DDS intent would modify unrelated outputs");
    }
  }
  VehicleCommandWire w;
  w.timestamp_us=px4_timestamp_us;
  w.param1=static_cast<float>(intent.params[0]);
  w.param2=static_cast<float>(intent.params[1]);
  w.param3=static_cast<float>(intent.params[2]);
  w.param4=static_cast<float>(intent.params[3]);
  w.param5=intent.params[4];w.param6=intent.params[5];
  w.param7=0.0f;
  w.target_system=target_system;w.target_component=target_component;
  w.source_system=source_system;w.source_component=source_component;
  return w;
}

struct DdsDispatchTicket {
  DdsActuatorIntent intent{};
  std::uint64_t durable_generation{0};
  double sent_at_sec{0};
  // An immutable descriptor; does not mean ROS2 actually published.
};
struct ObservedPx4Ack {
  std::uint32_t command{0};
  std::uint8_t result{0};
  std::uint8_t target_system{0};
  std::uint16_t target_component{0};
  bool from_external{false};
  std::uint64_t px4_timestamp_us{0};
};
// Receipt must be produced by a separately authenticated request-correlating
// transport, NOT synthesized from VehicleCommandAck alone.
struct UniquelyCorrelatedReceipt {
  DdsActuatorOwner owner{DdsActuatorOwner::mission_spray};
  DdsActuatorAction action{DdsActuatorAction::off};
  std::uint64_t local_sequence{0};
  std::uint64_t durable_generation{0};
  std::string identity{};
  double received_at_sec{0};
  bool accepted{false};
  bool correlation_proven{false};
};
enum class TransportState { idle, awaiting_receipt, positive_receipt,
                             negative_receipt, timed_out,
                             awaiting_physical_off_proof };
struct TransportResult {
  TransportState state{TransportState::idle};
  bool changed{false};
  bool publish_authorized{false}; // NEVER true just because a PX4 ACK arrived
  std::string reason{"IDLE"};
};
class NativeDdsActuatorTransport final {
 public:
  explicit NativeDdsActuatorTransport(FileBackedActuatorJournal &wal,
                                      double timeout_sec=1.0)
      : wal_(wal), timeout_sec_(timeout_sec) {
    if (!std::isfinite(timeout_sec_) || timeout_sec_<=0)
      throw std::invalid_argument("DDS response timeout must be positive");
  }
  // Fail-closed: caller MUST publish only after this method returns a ticket;
  // recording the ON write-ahead lock is mandatory and synchronous.
  std::optional<DdsDispatchTicket> begin_on(const DdsActuatorIntent &intent,
      double now, bool link_healthy, bool operator_authorized,
      bool physical_off_proven, bool exclusive_actuator_owner) {
    check_time(now);
    if (intent.action!=DdsActuatorAction::on || !link_healthy ||
        !operator_authorized || !physical_off_proven ||
        !exclusive_actuator_owner || pending_ || wal_.recovery_required() ||
        wal_.identity_used(intent.owner,intent.identity))
      return std::nullopt;
    wal_.persist_before_on(intent); // may throw: must NOT publish on failure
    pending_=DdsDispatchTicket{intent,wal_.active_lock()->dispatch_generation,now};
    state_=TransportState::awaiting_receipt;
    return pending_;
  }
  // OFF can preempt a pending ON (e.g. E-stop). Old ON receipts are then
  // invalidated; this is a priority RELEASE, never a second PRESS.
  std::optional<DdsDispatchTicket> begin_off(const DdsActuatorIntent &intent,
      double now, bool link_healthy) {
    check_time(now);
    if (intent.action!=DdsActuatorAction::off || !link_healthy ||
        !wal_.recovery_required()) return std::nullopt;
    if (pending_ && pending_->intent.action==DdsActuatorAction::off &&
        state_==TransportState::awaiting_receipt &&
        now-pending_->sent_at_sec < timeout_sec_) return std::nullopt;
    const auto generation=wal_.persist_before_off(intent); // durable first
    pending_=DdsDispatchTicket{intent,generation,now};
    state_=TransportState::awaiting_receipt;
    return pending_;
  }
  // PX4 ACK has no dispatch identity. Even ACCEPTED cannot prove which
  // same-command request was accepted, nor physical actuator state.
  TransportResult observe_unkeyed_px4_ack(const ObservedPx4Ack &ack) const {
    if (ack.command!=187 || !pending_) return {state_,false,false,"UNRELATED_ACK"};
    return {state_,false,false,"UNKEYED_ACK_OBSERVATION_ONLY"};
  }
  TransportResult confirm_unique_receipt(const UniquelyCorrelatedReceipt &r,
                                         double now) {
    check_time(now);
    if (!pending_ || !r.correlation_proven || !std::isfinite(r.received_at_sec) ||
        r.received_at_sec<pending_->sent_at_sec || r.received_at_sec>now ||
        r.owner!=pending_->intent.owner ||
        r.action!=pending_->intent.action ||
        r.local_sequence!=pending_->intent.local_sequence ||
        r.durable_generation!=pending_->durable_generation ||
        r.identity!=pending_->intent.identity ||
        state_!=TransportState::awaiting_receipt)
      return {state_,false,false,"RECEIPT_NOT_UNIQUELY_MATCHED"};
    if (now-pending_->sent_at_sec >= timeout_sec_) {
      state_=TransportState::timed_out;
      return {state_,true,false,"LATE_RESPONSE_UNTRUSTED"};
    }
    if (!r.accepted) {
      state_=TransportState::negative_receipt;
      return {state_,true,false,"COMMAND_REJECTED_RELEASE_REQUIRED"};
    }
    if (pending_->intent.action==DdsActuatorAction::off) {
      wal_.record_off_response(pending_->intent,pending_->durable_generation,true);
      state_=TransportState::awaiting_physical_off_proof;
      return {state_,true,false,"OFF_ACCEPTED_NOT_PHYSICALLY_VERIFIED"};
    }
    state_=TransportState::positive_receipt;
    return {state_,true,false,"ON_RESPONSE_NOT_PHYSICAL_PROOF"};
  }
  TransportResult prove_physical_off(const DdsDispatchTicket &ticket,
                                    bool externally_verified_off) {
    if (!pending_ || ticket.intent.action!=DdsActuatorAction::off ||
        pending_->durable_generation!=ticket.durable_generation ||
        pending_->intent.local_sequence!=ticket.intent.local_sequence ||
        pending_->intent.identity!=ticket.intent.identity ||
        pending_->intent.owner!=ticket.intent.owner ||
        state_!=TransportState::awaiting_physical_off_proof ||
        !externally_verified_off)
      return {state_,false,false,"OFF_NOT_INDEPENDENTLY_VERIFIED"};
    wal_.complete_off(ticket.intent,ticket.durable_generation,true);
    pending_.reset();state_=TransportState::idle;
    return {state_,true,false,"OFF_VERIFIED_JOURNAL_CLEARED"};
  }
  TransportResult poll(double now) {
    check_time(now);
    if (pending_ && state_==TransportState::awaiting_receipt &&
        now-pending_->sent_at_sec>=timeout_sec_) {
      state_=TransportState::timed_out;
      return {state_,true,false,"RECEIPT_TIMEOUT_RELEASE_REQUIRED"};
    }
    return {state_,false,false,"NO_CHANGE"};
  }
  const std::optional<DdsDispatchTicket> &pending() const {return pending_;}
  TransportState state() const {return state_;}
 private:
  void check_time(double now) {
    if (!std::isfinite(now) || now<0 || (last_time_ && now<*last_time_))
      throw std::logic_error("DDS transport clock went backwards: block dispatch");
    last_time_=now;
  }
  FileBackedActuatorJournal &wal_;
  double timeout_sec_{1.0};
  std::optional<double> last_time_{};
  std::optional<DdsDispatchTicket> pending_{};
  TransportState state_{TransportState::idle};
};
} // namespace dyx::rpp
