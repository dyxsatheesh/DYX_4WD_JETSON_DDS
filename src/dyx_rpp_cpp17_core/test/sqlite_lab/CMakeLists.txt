
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
