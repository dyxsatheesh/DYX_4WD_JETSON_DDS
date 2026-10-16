# H6E2A - Windows PowerShell 5.1+, native C++17 offline only.
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$build = Join-Path $repo 'build\dyx_h6e2_gate'
cmake -S (Join-Path $repo 'tools\dyx_h6e2_gate') -B $build -G 'Visual Studio 17 2022' -A x64
if ($LASTEXITCODE -ne 0) { throw 'H6E2A Windows CMake configure failed' }
cmake --build $build --config Release
if ($LASTEXITCODE -ne 0) { throw 'H6E2A Windows native build failed' }
ctest --test-dir $build -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'H6E2A Windows tests failed' }
Write-Host 'H6E2A OFFLINE 3/3 TESTS PASSED; NO NETWORK, ROS2 OR ANDROID CERTIFICATION'
