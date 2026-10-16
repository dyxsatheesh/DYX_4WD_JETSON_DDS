$ErrorActionPreference='Stop'
$root=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
Set-Location $root
cmake -S tools/dyx_h6b_gate -B build/dyx_h6b_gate -G 'Visual Studio 17 2022' -A x64
if($LASTEXITCODE -ne 0){throw 'H6B configure failed'}
cmake --build build/dyx_h6b_gate --config Release
if($LASTEXITCODE -ne 0){throw 'H6B build failed'}
ctest --test-dir build/dyx_h6b_gate -C Release --output-on-failure
if($LASTEXITCODE -ne 0){throw 'H6B tests failed'}
Write-Host 'H6B OFFLINE PREVIEW GATE PASSED; ROS 2 COLCON/HARDWARE NOT CERTIFIED'
