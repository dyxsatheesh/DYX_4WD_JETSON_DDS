# PowerShell 5.1+: native C++17 offline tests, no ROS/motor processes.
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$build = Join-Path $repo 'build\dyx_h6c_gate'
cmake -S (Join-Path $repo 'tools\dyx_h6c_gate') -B $build -G 'Visual Studio 17 2022' -A x64
if ($LASTEXITCODE -ne 0) { throw 'H6C CMake configuration failed' }
cmake --build $build --config Release
if ($LASTEXITCODE -ne 0) { throw 'H6C C++17 compilation failed' }
ctest --test-dir $build -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'H6C offline regression failed' }
Write-Host 'H6C OFFLINE VALIDATED; ROS2 COLCON AND PX4 NOT CERTIFIED'
