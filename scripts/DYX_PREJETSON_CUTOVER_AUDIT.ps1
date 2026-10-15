# Windows VS2022 / MSVC: offline observer-only, not a deployment script.
$ErrorActionPreference = 'Stop'
$repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$build = Join-Path $repo 'build\dyx_cutover_audit'
cmake -S (Join-Path $repo 'tools\dyx_prejetson_cutover') -B $build -G 'Visual Studio 17 2022' -A x64
if ($LASTEXITCODE -ne 0) { throw 'Cutover audit configure failed' }
cmake --build $build --config Release
if ($LASTEXITCODE -ne 0) { throw 'Cutover audit compile failed' }
ctest --test-dir $build -C Release --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'Cutover audit tests failed' }
$exe = Join-Path $build 'Release\dyx_cutover_audit.exe'
$report = Join-Path $build 'dyx_cutover_status.json'
& $exe --root $repo --json $report
if ($LASTEXITCODE -ne 0) { throw 'Cutover audit failed' }
Write-Host "Report: $report"
Write-Warning 'This reports remaining blockers. It does not certify production cutover.'
