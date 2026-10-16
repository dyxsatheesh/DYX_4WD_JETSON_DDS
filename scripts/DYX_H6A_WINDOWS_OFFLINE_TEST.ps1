# Offline C++17 gate; no ROS2, Jetson, PX4, motors or spray commands.
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$build = Join-Path $repo 'build\dyx_h6a_offline'
cmake -S (Join-Path $repo 'tools\dyx_h6a_gate') -B $build -G 'Visual Studio 17 2022' -A x64
if ($LASTEXITCODE -ne 0) { throw 'H6A configure failed' }
cmake --build $build --config Release
if ($LASTEXITCODE -ne 0) { throw 'H6A build failed' }
ctest --test-dir $build -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'H6A test suite failed' }
& (Join-Path $build 'Release\dyx_h6a_gate.exe') $repo
if ($LASTEXITCODE -ne 0) { throw 'H6A live repository source gate failed' }
Write-Warning 'Offline boundary only. No production or motor certification.'
