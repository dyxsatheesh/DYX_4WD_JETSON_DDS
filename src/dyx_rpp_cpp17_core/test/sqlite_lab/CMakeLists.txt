cmake_minimum_required(VERSION 3.16)
project(dyx_rpp_cpp17_sqlite_replay_lab LANGUAGES CXX)
# OPTIONAL native Linux/Jetson lane, separate from 144-test Windows C++ core.
# On Ubuntu: sudo apt-get install libsqlite3-dev
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(SQLite3 REQUIRED)
find_package(Threads REQUIRED)
enable_testing()
add_executable(draft_sqlite_persistence_native_smoke ../draft_sqlite_persistence_native_smoke.cpp)
target_include_directories(draft_sqlite_persistence_native_smoke PRIVATE ../../include)
target_link_libraries(draft_sqlite_persistence_native_smoke PRIVATE SQLite::SQLite3 Threads::Threads)
if(MSVC)
  target_compile_options(draft_sqlite_persistence_native_smoke PRIVATE /W4 /UNDEBUG)
else()
  target_compile_options(draft_sqlite_persistence_native_smoke PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
endif()
foreach(mode IN ITEMS reopen conflict faults recovery)
  add_test(NAME draft_sqlite_${mode}_native_smoke COMMAND "$<TARGET_FILE:draft_sqlite_persistence_native_smoke>" ${mode})
endforeach()

# Phase 2H4Y: test the actual H4W source interface against the real H4X SQLite
# implementation (no Android ingress, network, DDS, PX4 or actuation).
add_executable(draft_sqlite_restart_fence_native_smoke ../draft_sqlite_restart_fence_native_smoke.cpp)
target_include_directories(draft_sqlite_restart_fence_native_smoke PRIVATE ../../include)
target_link_libraries(draft_sqlite_restart_fence_native_smoke PRIVATE SQLite::SQLite3 Threads::Threads)
if(MSVC)
  target_compile_options(draft_sqlite_restart_fence_native_smoke PRIVATE /W4 /UNDEBUG)
else()
  target_compile_options(draft_sqlite_restart_fence_native_smoke PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
endif()
foreach(mode IN ITEMS reopen binding fault concurrent)
  add_test(NAME draft_sqlite_fence_${mode}_native_smoke
      COMMAND "$<TARGET_FILE:draft_sqlite_restart_fence_native_smoke>" ${mode})
endforeach()

# Phase 2H4Z: separate atomic request/replay-fence journal prototype.
# No production ingress; additional optional SQLite-only tests.
add_executable(draft_sqlite_mutation_journal_native_smoke
    ../draft_sqlite_mutation_journal_native_smoke.cpp)
target_include_directories(draft_sqlite_mutation_journal_native_smoke PRIVATE ../../include)
target_link_libraries(draft_sqlite_mutation_journal_native_smoke
    PRIVATE SQLite::SQLite3 Threads::Threads)
if(MSVC)
  target_compile_options(draft_sqlite_mutation_journal_native_smoke PRIVATE /W4 /UNDEBUG)
else()
  target_compile_options(draft_sqlite_mutation_journal_native_smoke
      PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
endif()
foreach(mode IN ITEMS nominal recovery faults concurrent)
  add_test(NAME draft_sqlite_journal_${mode}_native_smoke
      COMMAND "$<TARGET_FILE:draft_sqlite_mutation_journal_native_smoke>" ${mode})
endforeach()

# Phase 2H5A: compile the real (but entirely offline) mission draft backend
# with the SQLite journal. No Jetson, DDS, HTTPS or actuator operations.
add_executable(draft_sqlite_journaled_backend_native_smoke
    ../draft_sqlite_journaled_backend_native_smoke.cpp)
target_include_directories(draft_sqlite_journaled_backend_native_smoke
    PRIVATE ../../include)
target_link_libraries(draft_sqlite_journaled_backend_native_smoke
    PRIVATE SQLite::SQLite3 Threads::Threads)
if(MSVC)
  target_compile_options(draft_sqlite_journaled_backend_native_smoke PRIVATE /W4 /UNDEBUG)
else()
  target_compile_options(draft_sqlite_journaled_backend_native_smoke
      PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
endif()
foreach(mode IN ITEMS nominal binding reopen pending)
  add_test(NAME draft_sqlite_backend_${mode}_native_smoke
      COMMAND "$<TARGET_FILE:draft_sqlite_journaled_backend_native_smoke>" ${mode})
endforeach()

# Phase 2H5B: strictly read-only lost-reply reconciliation, no restore/START.
add_executable(draft_sqlite_reconciliation_read_native_smoke
    ../draft_sqlite_reconciliation_read_native_smoke.cpp)
target_include_directories(draft_sqlite_reconciliation_read_native_smoke PRIVATE ../../include)
target_link_libraries(draft_sqlite_reconciliation_read_native_smoke PRIVATE SQLite::SQLite3 Threads::Threads)
if(MSVC)
  target_compile_options(draft_sqlite_reconciliation_read_native_smoke PRIVATE /W4 /UNDEBUG)
else()
  target_compile_options(draft_sqlite_reconciliation_read_native_smoke PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
endif()
foreach(mode IN ITEMS preview pending denied binding)
  add_test(NAME draft_sqlite_read_${mode}_native_smoke
      COMMAND "$<TARGET_FILE:draft_sqlite_reconciliation_read_native_smoke>" ${mode})
endforeach()

# Phase 2H5C: read-only persisted H5A snapshot consistency audits. No restore.
add_executable(draft_sqlite_snapshot_audit_native_smoke
    ../draft_sqlite_snapshot_audit_native_smoke.cpp)
target_include_directories(draft_sqlite_snapshot_audit_native_smoke PRIVATE ../../include)
target_link_libraries(draft_sqlite_snapshot_audit_native_smoke
    PRIVATE SQLite::SQLite3 Threads::Threads)
if(MSVC)
  target_compile_options(draft_sqlite_snapshot_audit_native_smoke PRIVATE /W4 /UNDEBUG)
else()
  target_compile_options(draft_sqlite_snapshot_audit_native_smoke
      PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
endif()
foreach(mode IN ITEMS preview ready tamper security)
  add_test(NAME draft_sqlite_audit_${mode}_native_smoke
      COMMAND "$<TARGET_FILE:draft_sqlite_snapshot_audit_native_smoke>" ${mode})
endforeach()

# Phase 2H5D: full saved mutation-history read-only screening. No recovery.
add_executable(draft_sqlite_history_screen_native_smoke
    ../draft_sqlite_history_screen_native_smoke.cpp)
target_include_directories(draft_sqlite_history_screen_native_smoke PRIVATE ../../include)
target_link_libraries(draft_sqlite_history_screen_native_smoke
    PRIVATE SQLite::SQLite3 Threads::Threads)
if(MSVC)
  target_compile_options(draft_sqlite_history_screen_native_smoke PRIVATE /W4 /UNDEBUG)
else()
  target_compile_options(draft_sqlite_history_screen_native_smoke
      PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
endif()
foreach(mode IN ITEMS nominal ready pending tamper security)
  add_test(NAME draft_sqlite_history_${mode}_native_smoke
      COMMAND "$<TARGET_FILE:draft_sqlite_history_screen_native_smoke>" ${mode})
endforeach()

# Phase 2H5E: sealed PREVIEW-only checkpoint for offline operator review.
# Does NOT restore backend, unquarantine writers or authorize vehicle motion.
add_executable(draft_sqlite_preview_checkpoint_native_smoke
    ../draft_sqlite_preview_checkpoint_native_smoke.cpp)
target_include_directories(draft_sqlite_preview_checkpoint_native_smoke PRIVATE ../../include)
target_link_libraries(draft_sqlite_preview_checkpoint_native_smoke
    PRIVATE SQLite::SQLite3 Threads::Threads)
if(MSVC)
  target_compile_options(draft_sqlite_preview_checkpoint_native_smoke PRIVATE /W4 /UNDEBUG)
else()
  target_compile_options(draft_sqlite_preview_checkpoint_native_smoke
      PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
endif()
foreach(mode IN ITEMS nominal reopen stale ready pending tamper)
  add_test(NAME draft_sqlite_checkpoint_${mode}_native_smoke
      COMMAND "$<TARGET_FILE:draft_sqlite_preview_checkpoint_native_smoke>" ${mode})
endforeach()

# Phase 2H5F: offline PREVIEW recovery preflight. Read-only, no restore.
add_executable(draft_sqlite_recovery_preflight_native_smoke
    ../draft_sqlite_recovery_preflight_native_smoke.cpp)
target_include_directories(draft_sqlite_recovery_preflight_native_smoke PRIVATE ../../include)
target_link_libraries(draft_sqlite_recovery_preflight_native_smoke
    PRIVATE SQLite::SQLite3 Threads::Threads)
if(MSVC)
  target_compile_options(draft_sqlite_recovery_preflight_native_smoke PRIVATE /W4 /UNDEBUG)
else()
  target_compile_options(draft_sqlite_recovery_preflight_native_smoke
      PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
endif()
foreach(mode IN ITEMS nominal reopen authorization stale pending ready no_checkpoint)
  add_test(NAME draft_sqlite_preflight_${mode}_native_smoke
      COMMAND "$<TARGET_FILE:draft_sqlite_recovery_preflight_native_smoke>" ${mode})
endforeach()

# Phase 2H5G: isolated memory-only PREVIEW read facade after restart.
# Refuses every command path. Never restores a live mission backend or DDS.
add_executable(draft_sqlite_recovered_preview_native_smoke
    ../draft_sqlite_recovered_preview_native_smoke.cpp)
target_include_directories(draft_sqlite_recovered_preview_native_smoke PRIVATE ../../include)
target_link_libraries(draft_sqlite_recovered_preview_native_smoke
    PRIVATE SQLite::SQLite3 Threads::Threads)
if(MSVC)
  target_compile_options(draft_sqlite_recovered_preview_native_smoke PRIVATE /W4 /UNDEBUG)
else()
  target_compile_options(draft_sqlite_recovered_preview_native_smoke
      PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
endif()
foreach(mode IN ITEMS nominal restart requests manual stale pending ready missing)
  add_test(NAME draft_sqlite_recovered_${mode}_native_smoke
      COMMAND "$<TARGET_FILE:draft_sqlite_recovered_preview_native_smoke>" ${mode})
endforeach()
