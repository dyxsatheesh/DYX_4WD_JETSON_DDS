cmake_minimum_required(VERSION 3.16)
project(dyx_rpp_cpp17_native_cpp_only LANGUAGES CXX)

# No Python interpreter, Python scripts, pybind, or ROS 2 installation required.
# This is the offline native C++17 smoke lane; the existing reference parity
# lane is intentionally separate and will be retired at final C++ cutover.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
enable_testing()
# Keep assertion-based legacy smoke checks active in Release builds.
# /DNDEBUG or -DNDEBUG would otherwise silently disable their core checks.

set(DYX_CPP_ONLY_SMOKE_TESTS
    path_index_native_test
    guidance_mock_smoke
    speed_regulator_native_smoke
    tracking_stability_native_smoke
    tracking_metrics_native_smoke
    stationary_detector_native_smoke
    terminal_stop_native_smoke
    pivot_motion_native_smoke
    forward_yaw_rate_native_smoke
    shadow_geometry_native_smoke
    mission_leg_preview_native_smoke
    mission_leg_preview_safety_native_smoke
    mission_sequence_native_smoke
    mission_sequence_safety_native_smoke
    raw_survey_accuracy_native_smoke
    raw_survey_safety_native_smoke
    raw_survey_mission_link_native_smoke
    trajectory_builder_native_smoke
    trajectory_builder_rules_native_smoke
    trajectory_builder_safety_native_smoke
    mission_snapshot_signature_native_smoke
    mission_snapshot_staging_native_smoke
    mission_snapshot_safety_native_smoke
    mission_runtime_install_native_smoke
    mission_runtime_install_safety_native_smoke
    safety_arbiter_rtk_native_smoke
    safety_arbiter_battery_native_smoke
    safety_arbiter_interlocks_native_smoke
    safety_mission_float_native_smoke
    safety_mission_fault_native_smoke
    safety_mission_hold_native_smoke
    spray_transaction_cycle_native_smoke
    spray_transaction_fault_native_smoke
    spray_transaction_restart_native_smoke
    spray_mission_link_native_smoke
    manual_spray_cycle_native_smoke
    manual_spray_safety_native_smoke
    manual_spray_recovery_native_smoke
    laser_steady_native_smoke
    laser_blink_native_smoke
    laser_safety_native_smoke
    dds_actuator_payload_native_smoke
    actuator_journal_restart_native_smoke
    actuator_journal_safety_native_smoke
    dds_transport_wire_native_smoke
    dds_transport_correlated_native_smoke
    dds_transport_timeout_native_smoke
    dds_transport_restart_native_smoke
    dds_monitor_observe_native_smoke
    dds_monitor_safety_native_smoke
    dds_monitor_config_native_smoke
    dds_safety_telemetry_nominal_native_smoke
    dds_safety_telemetry_stale_native_smoke
    dds_safety_telemetry_reboot_native_smoke
    dds_safety_telemetry_fault_native_smoke
    dds_mission_runtime_nominal_native_smoke
    dds_mission_runtime_float_native_smoke
    dds_mission_runtime_fault_native_smoke
    dds_mission_runtime_epoch_native_smoke
    mission_gateway_nominal_native_smoke
    mission_gateway_authority_native_smoke
    mission_gateway_safety_native_smoke
    mission_gateway_epoch_native_smoke
    backend_api_routes_native_smoke
    backend_api_authority_native_smoke
    backend_api_commands_native_smoke
    backend_api_fault_native_smoke
    backend_upload_nominal_native_smoke
    backend_upload_update_native_smoke
    backend_upload_authority_native_smoke
    backend_upload_load_safety_native_smoke
    backend_live_nominal_native_smoke
    backend_live_stale_native_smoke
    backend_live_authority_native_smoke
    backend_live_sources_native_smoke
    backend_http_live_native_smoke
    backend_http_commands_native_smoke
    backend_http_security_native_smoke
    backend_http_json_native_smoke
    backend_http_stale_sse_native_smoke
    backend_loopback_get_native_smoke
    backend_loopback_commands_native_smoke
    backend_loopback_security_native_smoke
    backend_loopback_auth_restart_native_smoke
    backend_ingress_lease_native_smoke
    backend_ingress_expiry_native_smoke
    backend_ingress_epoch_native_smoke
    backend_ingress_wire_native_smoke
    backend_lifecycle_nominal_native_smoke
    backend_lifecycle_timeout_native_smoke
    backend_lifecycle_capacity_native_smoke
    backend_lifecycle_android_native_smoke
    backend_loopback_deadline_native_smoke
    backend_loopback_incomplete_native_smoke
    backend_loopback_header_limit_native_smoke
    backend_loopback_stop_native_smoke
    backend_stream_nominal_native_smoke
    backend_stream_reconnect_native_smoke
    backend_stream_authority_native_smoke
    backend_stream_epoch_native_smoke
    backend_android_nominal_native_smoke
    backend_android_reconnect_native_smoke
    backend_android_ordering_native_smoke
    backend_android_safety_native_smoke
    backend_android_controls_nominal_native_smoke
    backend_android_controls_safety_native_smoke
    backend_android_controls_timeout_native_smoke
    backend_android_controls_epoch_native_smoke
    backend_android_editor_nominal_native_smoke
    backend_android_editor_safety_native_smoke
    backend_android_editor_timeout_native_smoke
    backend_android_editor_epoch_native_smoke
    backend_upload_http_nominal_native_smoke
    backend_upload_http_security_native_smoke
    backend_upload_http_parser_native_smoke
    backend_upload_http_safety_native_smoke
    backend_android_draft_codec_roundtrip_native_smoke
    backend_android_draft_codec_security_native_smoke
    backend_android_draft_codec_ordering_native_smoke
    backend_android_draft_codec_tamper_native_smoke
    backend_android_sync_nominal_native_smoke
    backend_android_sync_security_native_smoke
    backend_android_sync_recovery_native_smoke
    backend_android_sync_timeout_native_smoke
    backend_android_coordinator_nominal_native_smoke
    backend_android_coordinator_security_native_smoke
    backend_android_coordinator_timeout_native_smoke
    backend_android_coordinator_epoch_native_smoke
    backend_draft_ingress_lab_nominal_native_smoke
    backend_draft_ingress_lab_security_native_smoke
    backend_draft_ingress_lab_replay_native_smoke
    backend_draft_ingress_lab_recovery_native_smoke
    backend_draft_framing_nominal_native_smoke
    backend_draft_framing_smuggling_native_smoke
    backend_draft_framing_binding_native_smoke
    backend_draft_framing_recovery_native_smoke
    backend_draft_boundary_nominal_native_smoke
    backend_draft_boundary_spoof_native_smoke
    backend_draft_boundary_authority_native_smoke
    backend_draft_boundary_recovery_native_smoke
    backend_draft_restart_nominal_native_smoke
    backend_draft_restart_replay_native_smoke
    backend_draft_restart_store_fault_native_smoke
    backend_draft_restart_recovery_native_smoke)

# Phase 2H5L: real C++17 gateway/backend quarantine bypass regression tests.
add_executable(backend_recovery_quarantine_native_smoke
    ../backend_recovery_quarantine_native_smoke.cpp)
target_include_directories(backend_recovery_quarantine_native_smoke PRIVATE ../../include)
if(MSVC)
  target_compile_options(backend_recovery_quarantine_native_smoke PRIVATE /W4 /UNDEBUG)
else()
  target_compile_options(backend_recovery_quarantine_native_smoke PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
endif()
foreach(mode IN ITEMS legacy upload alternate load actions stop epoch active sticky)
  add_test(NAME backend_recovery_quarantine_${mode}_native_smoke
      COMMAND "$<TARGET_FILE:backend_recovery_quarantine_native_smoke>" ${mode})
endforeach()

foreach(test_name IN LISTS DYX_CPP_ONLY_SMOKE_TESTS)
  add_executable(${test_name} ../${test_name}.cpp)
  target_include_directories(${test_name} PRIVATE ../../include)
  if(MSVC)
    target_compile_options(${test_name} PRIVATE /W4 /UNDEBUG)
  else()
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
  endif()
  add_test(NAME ${test_name} COMMAND "$<TARGET_FILE:${test_name}>")
endforeach()

# The offline listener has real loopback sockets. Windows requires ws2_32.
find_package(Threads REQUIRED)
foreach(test_name IN ITEMS backend_loopback_get_native_smoke
                           backend_loopback_commands_native_smoke
                           backend_loopback_security_native_smoke
                           backend_loopback_auth_restart_native_smoke
    backend_ingress_lease_native_smoke
    backend_ingress_expiry_native_smoke
    backend_ingress_epoch_native_smoke
    backend_ingress_wire_native_smoke
    backend_loopback_deadline_native_smoke
    backend_loopback_incomplete_native_smoke
    backend_loopback_header_limit_native_smoke
    backend_loopback_stop_native_smoke)
  target_link_libraries(${test_name} PRIVATE Threads::Threads)
  if(WIN32)
    target_link_libraries(${test_name} PRIVATE ws2_32)
  endif()
endforeach()
add_executable(backend_loopback_readonly_demo ../../tools/loopback_readonly_demo.cpp)
target_include_directories(backend_loopback_readonly_demo PRIVATE ../../include)
target_link_libraries(backend_loopback_readonly_demo PRIVATE Threads::Threads)
if(WIN32)
  target_link_libraries(backend_loopback_readonly_demo PRIVATE ws2_32)
endif()
