#include "dyx_px4_dds_bridge/safety_freshness.hpp"

#include <cassert>
#include <chrono>
#include <limits>
#include <iostream>

int main() {
  using namespace std::chrono;
  using dyx::bridge::safety_evidence_fresh;

  const auto now = steady_clock::now();

  assert(!safety_evidence_fresh(
      false, now, now, 0.30));

  assert(safety_evidence_fresh(
      true, now, now, 0.30));

  assert(safety_evidence_fresh(
      true, now - milliseconds(100), now, 0.30));

  assert(safety_evidence_fresh(
      true, now - milliseconds(300), now, 0.30));

  assert(!safety_evidence_fresh(
      true, now - milliseconds(301), now, 0.30));

  assert(!safety_evidence_fresh(
      true, now - seconds(1), now, 0.30));

  assert(!safety_evidence_fresh(
      true, now + milliseconds(1), now, 0.30));

  assert(!safety_evidence_fresh(
      true, now, now, 0.0));

  assert(!safety_evidence_fresh(
      true, now, now, -1.0));

  assert(!safety_evidence_fresh(
      true, now, now, 0.60));

  assert(!safety_evidence_fresh(
      true, now, now,
      std::numeric_limits<double>::quiet_NaN()));

  std::cout
      << "PASS: 11 DDS safety freshness checks\n";
}
