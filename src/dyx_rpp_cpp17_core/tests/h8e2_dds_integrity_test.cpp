#include "dyx_rpp_cpp17_core/dds_safety_telemetry.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>

using namespace dyx::rpp;

static int passed = 0;

static void check(bool ok, const char* name) {
    if (!ok) {
        std::cerr << "FAIL: " << name << '\n';
        std::exit(1);
    }
    ++passed;
    std::cout << "PASS: " << name << '\n';
}

static DdsBatteryFrame battery(
    std::uint64_t stamp, double received, double voltage = 25.0) {
    DdsBatteryFrame f{};
    f.timestamp_us = stamp;
    f.received_sec = received;
    f.voltage_v = voltage;
    f.connected = true;
    f.cell_count = 8;
    return f;
}

static DdsGpsFrame gps(
    std::uint64_t stamp, double received, std::uint32_t device) {
    DdsGpsFrame f{};
    f.gps_timestamp_sample_us = stamp;
    f.received_sec = received;
    f.device_id = device;
    f.latitude_deg = 12.9;
    f.longitude_deg = 79.1;
    f.fix_type = 6;
    return f;
}

int main() {
    {
        NativeDdsSafetyTelemetry t;
        check(t.observe_battery(battery(100, 1.0)) ==
                  DdsTelemetryDecision::accepted,
              "Initial battery sample accepted");

        check(t.observe_battery(battery(100, 2.0)) ==
                  DdsTelemetryDecision::duplicate,
              "Repeated battery timestamp identified as duplicate");

        const auto s = t.snapshot(3.0, true, true, false);
        check(!s.battery_sample_fresh,
              "Duplicate sample cannot refresh battery freshness");

        check(!t.integrity_fault_latched(),
              "Harmless duplicate does not latch integrity fault");
    }

    {
        NativeDdsSafetyTelemetry t;
        t.observe_battery(battery(200, 1.0));

        check(t.observe_battery(battery(100, 2.0)) ==
                  DdsTelemetryDecision::px4_epoch_changed,
              "Backward PX4 timestamp detects epoch change");

        check(t.integrity_fault_latched(),
              "PX4 epoch-change fault remains latched");

        check(t.observe_battery(battery(300, 3.0)) ==
                  DdsTelemetryDecision::fault_latched,
              "Fresh data cannot automatically clear epoch fault");

        check(!t.begin_new_disarmed_px4_epoch(false, true),
              "Unauthorized epoch reset rejected");

        check(!t.begin_new_disarmed_px4_epoch(true, false),
              "Armed epoch reset rejected");

        check(t.begin_new_disarmed_px4_epoch(true, true),
              "Explicit authorized disarmed reset accepted");

        check(!t.integrity_fault_latched(),
              "Authorized reset clears integrity latch");

        const auto empty = t.snapshot(4.0, true, true, false);
        check(!empty.battery_sample_fresh,
              "Reset clears old telemetry evidence");

        check(t.observe_battery(battery(1, 5.0)) ==
                  DdsTelemetryDecision::accepted,
              "Fresh epoch can accept new PX4 timestamps");
    }

    {
        NativeDdsSafetyTelemetry t;
        check(t.observe_battery(battery(
                  100, 1.0,
                  std::numeric_limits<double>::quiet_NaN())) ==
                  DdsTelemetryDecision::invalid_data,
              "Invalid battery voltage rejected");

        check(t.integrity_fault_latched(),
              "Invalid battery data latches integrity fault");
    }

    {
        NativeDdsSafetyTelemetry t;
        check(t.observe_gps(gps(100, 1.0, 101)) ==
                  DdsTelemetryDecision::accepted,
              "Initial GNSS receiver selected");

        check(t.observe_gps(gps(200, 2.0, 202)) ==
                  DdsTelemetryDecision::invalid_data,
              "Unexpected GNSS receiver switch rejected");

        check(t.integrity_fault_latched(),
              "GNSS source switch latches integrity fault");
    }

    {
        NativeDdsSafetyTelemetry t;
        check(t.observe_battery(battery(100, 5.0)) ==
                  DdsTelemetryDecision::accepted,
              "Reference host receipt accepted");

        check(t.observe_battery(battery(200, 4.0)) ==
                  DdsTelemetryDecision::clock_regression,
              "Backward host receive clock rejected");

        check(t.integrity_fault_latched(),
              "Host clock regression latches integrity fault");
    }

    std::cout
        << "ALL H8E2 DDS INTEGRITY TESTS PASSED ("
        << passed << " checks)\n";
    return 0;
}
