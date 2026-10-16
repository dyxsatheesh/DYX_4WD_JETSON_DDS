# Windows PowerShell 5.1; no ROS 2 or motor access.
$ErrorActionPreference='Stop'
$repo=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$build=Join-Path $repo 'build\dyx_h6d_gate'
cmake -S (Join-Path $repo 'tools\dyx_h6d_gate') -B $build -G 'Visual Studio 17 2022' -A x64
if($LASTEXITCODE -ne 0){throw 'H6D configure failed'}
cmake --build $build --config Release
if($LASTEXITCODE -ne 0){throw 'H6D build failed'}
ctest --test-dir $build -C Release --output-on-failure
if($LASTEXITCODE -ne 0){throw 'H6D C++17 tests failed'}
Write-Host 'H6D OFFLINE TESTS PASSED; ROS2/PX4/HARDWARE NOT CERTIFIED'
