# Windows: Existing 160 C++ tests, plus the cross-platform inhibited executable.
$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root
cmake -S src/dyx_rpp_cpp17_core/test/native_cpp_only -B build/dyx_cpp_only -G 'Visual Studio 17 2022' -A x64
if ($LASTEXITCODE -ne 0) { throw 'Windows native CMake configure failed' }
cmake --build build/dyx_cpp_only --config Release
if ($LASTEXITCODE -ne 0) { throw 'Windows native build failed' }
ctest --test-dir build/dyx_cpp_only -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'Windows native tests failed' }
cmake -S tools/dyx_jetson_preflight -B build/dyx_preflight_win -G 'Visual Studio 17 2022' -A x64
if ($LASTEXITCODE -ne 0) { throw 'Preflight configure failed' }
cmake --build build/dyx_preflight_win --config Release
if ($LASTEXITCODE -ne 0) { throw 'Preflight build failed' }
ctest --test-dir build/dyx_preflight_win -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'Preflight tests failed' }
