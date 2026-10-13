#pragma once
// Phase 2G2B: pure C++17 mission snapshot integrity & staging (non-actuating).
// Signature byte protocol matches existing prepared-path v1: SHA-256 over
// NAVIGATION + BE float64(x,y)*, MARKINGS + BE float64(x,y)*,
// TYPES + uint8(point_type)*, INDICES + BE signed int32(marking_index)*.
// A digest validates content consistency, not the identity or authorization
// of its publisher; higher-level mission/session authorization is required.

#include "dyx_rpp_cpp17_core/path_index.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dyx::rpp {
namespace snapshot_detail {
class Sha256 final {
 public:
  Sha256() = default;
  void update(const std::uint8_t *data, std::size_t size) {
    if (size && !data) throw std::invalid_argument("SHA256 null data");
    if (size > (std::numeric_limits<std::uint64_t>::max() / 8) - length_) {
      throw std::length_error("SHA256 message too large");
    }
    length_ += static_cast<std::uint64_t>(size);
    for (std::size_t i = 0; i < size; ++i) {
      buffer_[used_++] = data[i];
      if (used_ == 64) {
        compress();
        used_ = 0;
      }
    }
  }
  void text(const char *s) {
    update(reinterpret_cast<const std::uint8_t *>(s), std::strlen(s));
  }
  void byte(std::uint8_t b) { update(&b, 1); }
  void u32be(std::uint32_t v) {
    for (int s = 24; s >= 0; s -= 8) byte(static_cast<std::uint8_t>(v >> s));
  }
  void u64be(std::uint64_t v) {
    for (int s = 56; s >= 0; s -= 8) byte(static_cast<std::uint8_t>(v >> s));
  }
  std::string hex() const {
    Sha256 tmp = *this;
    const std::uint64_t bits = tmp.length_ * 8;
    tmp.byte(0x80);
    while (tmp.used_ != 56) tmp.byte(0);
    tmp.u64be(bits);
    static constexpr char alphabet[] = "0123456789abcdef";
    std::string result;
    result.reserve(64);
    for (std::uint32_t word : tmp.state_) {
      for (int s = 28; s >= 0; s -= 4)
        result.push_back(alphabet[(word >> s) & 15U]);
    }
    return result;
  }
 private:
  static std::uint32_t rotate(std::uint32_t x, int n) { return (x >> n) | (x << (32-n)); }
  void compress() {
    std::array<std::uint32_t, 64> w{};
    for (std::size_t i=0; i<16; ++i) {
      w[i] = (static_cast<std::uint32_t>(buffer_[4*i]) << 24) |
             (static_cast<std::uint32_t>(buffer_[4*i+1]) << 16) |
             (static_cast<std::uint32_t>(buffer_[4*i+2]) << 8) |
              static_cast<std::uint32_t>(buffer_[4*i+3]);
    }
    for (std::size_t i=16; i<64; ++i) {
      const auto a = rotate(w[i-15],7) ^ rotate(w[i-15],18) ^ (w[i-15] >> 3);
      const auto b = rotate(w[i-2],17) ^ rotate(w[i-2],19) ^ (w[i-2] >> 10);
      w[i] = w[i-16] + a + w[i-7] + b;
    }
    auto a=state_[0], b=state_[1], c=state_[2], d=state_[3];
    auto e=state_[4], f=state_[5], g=state_[6], h=state_[7];
    for (std::size_t i=0; i<64; ++i) {
      const auto s1 = rotate(e,6) ^ rotate(e,11) ^ rotate(e,25);
      const auto ch = (e & f) ^ (~e & g);
      const auto t1 = h + s1 + ch + constants_[i] + w[i];
      const auto s0 = rotate(a,2) ^ rotate(a,13) ^ rotate(a,22);
      const auto maj = (a & b) ^ (a & c) ^ (b & c);
      const auto t2 = s0 + maj;
      h=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
    }
    state_[0]+=a; state_[1]+=b; state_[2]+=c; state_[3]+=d;
    state_[4]+=e; state_[5]+=f; state_[6]+=g; state_[7]+=h;
  }
  inline static constexpr std::array<std::uint32_t,64> constants_{{
    0x428a2f98U,0x71374491U,0xb5c0fbcfU,0xe9b5dba5U,0x3956c25bU,0x59f111f1U,0x923f82a4U,0xab1c5ed5U,
    0xd807aa98U,0x12835b01U,0x243185beU,0x550c7dc3U,0x72be5d74U,0x80deb1feU,0x9bdc06a7U,0xc19bf174U,
    0xe49b69c1U,0xefbe4786U,0x0fc19dc6U,0x240ca1ccU,0x2de92c6fU,0x4a7484aaU,0x5cb0a9dcU,0x76f988daU,
    0x983e5152U,0xa831c66dU,0xb00327c8U,0xbf597fc7U,0xc6e00bf3U,0xd5a79147U,0x06ca6351U,0x14292967U,
    0x27b70a85U,0x2e1b2138U,0x4d2c6dfcU,0x53380d13U,0x650a7354U,0x766a0abbU,0x81c2c92eU,0x92722c85U,
    0xa2bfe8a1U,0xa81a664bU,0xc24b8b70U,0xc76c51a3U,0xd192e819U,0xd6990624U,0xf40e3585U,0x106aa070U,
    0x19a4c116U,0x1e376c08U,0x2748774cU,0x34b0bcb5U,0x391c0cb3U,0x4ed8aa4aU,0x5b9cca4fU,0x682e6ff3U,
    0x748f82eeU,0x78a5636fU,0x84c87814U,0x8cc70208U,0x90befffaU,0xa4506cebU,0xbef9a3f7U,0xc67178f2U
  }};
  std::array<std::uint32_t,8> state_{{
    0x6a09e667U,0xbb67ae85U,0x3c6ef372U,0xa54ff53aU,
    0x510e527fU,0x9b05688cU,0x1f83d9abU,0x5be0cd19U
  }};
  std::array<std::uint8_t,64> buffer_{};
  std::size_t used_{0};
  std::uint64_t length_{0};
};
inline void add_double(Sha256 &sha, double x) {
  static_assert(sizeof(double) == sizeof(std::uint64_t), "IEEE-754 64-bit double required");
  static_assert(std::numeric_limits<double>::is_iec559, "IEEE-754 required");
  if (!std::isfinite(x)) throw std::invalid_argument("nonfinite path coordinate");
  std::uint64_t bits{};
  std::memcpy(&bits,&x,sizeof(bits));
  sha.u64be(bits);
}
} // namespace snapshot_detail

struct PreparedPathSnapshot {
  std::vector<Point2D> navigation_points;
  std::vector<Point2D> marking_points;
  std::vector<int> point_types;
  std::vector<int> marking_indices;
  std::string signature;
  std::vector<RawPathPoint> raw_navigation() const {
    std::vector<RawPathPoint> result;
    result.reserve(navigation_points.size());
    for (std::size_t i=0;i<navigation_points.size();++i)
      result.push_back({navigation_points[i],point_types[i],marking_indices[i]});
    return result;
  }
};

inline bool valid_path_signature(const std::string &value) {
  if (value.size()!=64) return false;
  for (char c : value)
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  return true;
}

inline void validate_snapshot_content(const PreparedPathSnapshot &s,
    std::size_t max_navigation_points=200000, std::size_t max_markings=10000) {
  const auto n=s.navigation_points.size(), m=s.marking_points.size();
  if (n < 2 || m < 2 || n > max_navigation_points || m > max_markings ||
      n < m || s.point_types.size()!=n || s.marking_indices.size()!=n)
    throw std::invalid_argument("invalid prepared path dimensions");
  std::size_t seen=0;
  for (std::size_t i=0;i<n;++i) {
    const auto &p=s.navigation_points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
      throw std::invalid_argument("nonfinite navigation coordinate");
    const int type=s.point_types[i];
    const int mark=s.marking_indices[i];
    if (type==point_marking) {
      if (seen>=m || mark!=static_cast<int>(seen) ||
          p.x!=s.marking_points[seen].x || p.y!=s.marking_points[seen].y)
        throw std::invalid_argument("marking order or coordinate mismatch");
      ++seen;
    } else if ((type!=point_pass_through && type!=point_dummy_alignment) || mark!=-1) {
      throw std::invalid_argument("invalid navigation point metadata");
    }
  }
  for (const auto &p:s.marking_points)
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
      throw std::invalid_argument("nonfinite marking coordinate");
  if (seen!=m || s.point_types.front()!=point_marking ||
      s.point_types.back()!=point_marking)
    throw std::invalid_argument("missing first, final or interior marking");
}

inline std::string compute_path_signature(const PreparedPathSnapshot &s) {
  validate_snapshot_content(s);
  snapshot_detail::Sha256 sha;
  sha.text("NAVIGATION");
  for (const auto &p : s.navigation_points) {
    snapshot_detail::add_double(sha,p.x);
    snapshot_detail::add_double(sha,p.y);
  }
  sha.text("MARKINGS");
  for (const auto &p : s.marking_points) {
    snapshot_detail::add_double(sha,p.x);
    snapshot_detail::add_double(sha,p.y);
  }
  sha.text("TYPES");
  for (int type : s.point_types) sha.byte(static_cast<std::uint8_t>(type));
  sha.text("INDICES");
  static_assert(sizeof(int)==4, "32-bit int required for signature compatibility");
  for (int index : s.marking_indices)
    sha.u32be(static_cast<std::uint32_t>(static_cast<std::int32_t>(index)));
  return sha.hex();
}

inline PreparedPathSnapshot signed_snapshot(const std::vector<RawPathPoint> &nav,
    const std::vector<Point2D> &markings) {
  PreparedPathSnapshot s;
  s.navigation_points.reserve(nav.size());
  s.point_types.reserve(nav.size());
  s.marking_indices.reserve(nav.size());
  for (const auto &p:nav) {
    s.navigation_points.push_back(p.point);
    s.point_types.push_back(p.point_type);
    s.marking_indices.push_back(p.marking_index);
  }
  s.marking_points=markings;
  s.signature=compute_path_signature(s);
  return s;
}

enum class SnapshotInstallStatus {
  not_ready, incomplete, invalid_signature, invalid_content,
  signature_mismatch, duplicate_install, installed
};
inline const char *snapshot_install_status_name(SnapshotInstallStatus s) {
  switch(s) {
    case SnapshotInstallStatus::not_ready:return "NOT_READY";
    case SnapshotInstallStatus::incomplete:return "INCOMPLETE";
    case SnapshotInstallStatus::invalid_signature:return "INVALID_SIGNATURE";
    case SnapshotInstallStatus::invalid_content:return "INVALID_CONTENT";
    case SnapshotInstallStatus::signature_mismatch:return "SIGNATURE_MISMATCH";
    case SnapshotInstallStatus::duplicate_install:return "DUPLICATE_INSTALL";
    case SnapshotInstallStatus::installed:return "INSTALLED";
  }
  return "INVALID_STATUS";
}
struct SnapshotInstallResult {
  SnapshotInstallStatus status{SnapshotInstallStatus::not_ready};
  std::optional<PreparedPathSnapshot> snapshot{};
  bool committed() const { return status==SnapshotInstallStatus::installed && snapshot.has_value(); }
};

// DDS topic arrival order is arbitrary. Keep each source independently until
// a READY flag, four complete sources, and a matching signature coexist.
// This holder never arms a vehicle or installs data into the live rover.
class PendingPreparedSnapshot final {
 public:
  void set_navigation(std::vector<Point2D> x) { navigation_=std::move(x); }
  void set_markings(std::vector<Point2D> x) { markings_=std::move(x); }
  void set_point_types(std::vector<int> x) { types_=std::move(x); }
  void set_marking_indices(std::vector<int> x) { indices_=std::move(x); }
  void set_signature(std::string x) { signature_=std::move(x); }
  void set_ready(bool ready) { ready_=ready; }
  void invalidate_readiness() { ready_=false; }
  void clear_navigation() { navigation_.reset(); }
  void clear_markings() { markings_.reset(); }
  void clear_point_types() { types_.reset(); }
  void clear_marking_indices() { indices_.reset(); }
  void clear_signature() { signature_.reset(); }
  // Only at an explicitly authorized mission-boundary reset.
  void clear_all() {
    navigation_.reset(); markings_.reset(); types_.reset();
    indices_.reset(); signature_.reset(); ready_=false;
    installed_signature_.reset();
  }
  SnapshotInstallResult try_install() {
    if (!ready_) return {SnapshotInstallStatus::not_ready,{}};
    if (!navigation_ || !markings_ || !types_ || !indices_ || !signature_)
      return {SnapshotInstallStatus::incomplete,{}};
    if (!valid_path_signature(*signature_))
      return {SnapshotInstallStatus::invalid_signature,{}};
    PreparedPathSnapshot s{*navigation_,*markings_,*types_,*indices_,*signature_};
    std::string computed;
    try { computed=compute_path_signature(s); }
    catch (const std::exception &) { return {SnapshotInstallStatus::invalid_content,{}}; }
    if (computed!=s.signature) return {SnapshotInstallStatus::signature_mismatch,{}};
    if (installed_signature_ && *installed_signature_==s.signature)
      return {SnapshotInstallStatus::duplicate_install,{}};
    installed_signature_=s.signature;
    ready_=false;
    return {SnapshotInstallStatus::installed,std::move(s)};
  }
 private:
  std::optional<std::vector<Point2D>> navigation_{};
  std::optional<std::vector<Point2D>> markings_{};
  std::optional<std::vector<int>> types_{};
  std::optional<std::vector<int>> indices_{};
  std::optional<std::string> signature_{};
  std::optional<std::string> installed_signature_{};
  bool ready_{false};
};
} // namespace dyx::rpp
