# Windows PowerShell 5.1+; audit only; never starts ROS, motors or spray.
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$build = Join-Path $repo 'build\dyx_h6e1_api_audit'
cmake -S (Join-Path $repo 'tools\dyx_h6e_api_parity') -B $build -G 'Visual Studio 17 2022' -A x64
if ($LASTEXITCODE -ne 0) { throw 'H6E1 configure failed' }
cmake --build $build --config Release
if ($LASTEXITCODE -ne 0) { throw 'H6E1 compile failed' }
ctest --test-dir $build -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'H6E1 offline tests failed' }
& (Join-Path $build 'Release\dyx_h6e_api_audit.exe') --root $repo --json (Join-Path $build 'dyx_h6e1_api_parity.json')
if ($LASTEXITCODE -ne 0) { throw 'H6E1 inventory scan failed' }
Write-Host "H6E1 route inventory at $build\dyx_h6e1_api_parity.json"
Write-Warning 'NOT CERTIFIED; legacy route parity and live Android compatibility are NOT complete.'
