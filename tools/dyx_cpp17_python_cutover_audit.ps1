# Read-only project-owned Python inventory and final cutover gate.
# Run with -Enforce only when ALL C++17 replacement modules are implemented.
# Uses PowerShell 5.1+; never deletes files and never invokes Python.
param([switch]$Enforce)
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path.TrimEnd('\', '/')
$ignore = '^(\.git|build|install|log|\.venv|venv|\.tox|\.pytest_cache|__pycache__)/'

$records = @(
  Get-ChildItem -LiteralPath $root -Recurse -File -Filter '*.py' |
    ForEach-Object {
      $rel = $_.FullName.Substring($root.Length).TrimStart('\', '/').Replace('\', '/')
      if ($rel -notmatch $ignore) { $rel }
    }
)
$source = @($records | Where-Object { $_ -like 'src/*' })
$byPackage = @($source | ForEach-Object {
  $chunks = $_.Split('/')
  if ($chunks.Length -ge 2) { $chunks[1] }
} | Group-Object | Sort-Object Count -Descending)

Write-Host ('Repository: ' + $root)
Write-Host ('Project Python source count: ' + $records.Count)
Write-Host ('Python files inside src/: ' + $source.Count)
Write-Host 'Python files by src package:'
foreach ($group in $byPackage) {
  Write-Host ('  {0,-28} {1,4}' -f $group.Name, $group.Count)
}

# New DYX C++ runtime dependencies must not import MAVROS.
$newRuntime = @(
  Get-ChildItem -LiteralPath (Join-Path $root 'src') -Recurse -File |
    Where-Object {
      $rel = $_.FullName.Substring($root.Length).TrimStart('\', '/').Replace('\', '/')
      $rel -match '^src/dyx_[^/]+/' -and ($rel -match '\.(cpp|hpp|h|xml|cmake)$' -or $rel -match '/CMakeLists\.txt$')
    }
)
$badMavros = @($newRuntime | Select-String -Pattern '(?i)(#\s*include\s*[<\"]mavros|mavros_msgs/|<\s*(?:depend|build_depend|exec_depend|build_export_depend)\s*>\s*mavros|find_package\s*\(\s*mavros|ament_target_dependencies\([^)]*mavros)' -List | ForEach-Object { $_.Path })
if ($badMavros.Count) {
  Write-Warning 'MAVROS references detected inside the new DYX C++ runtime files:'
  $badMavros | ForEach-Object { Write-Host ('  ' + $_) }
}

if ($records.Count -eq 0 -and $badMavros.Count -eq 0) {
  Write-Host 'PASS: no project Python files or MAVROS references in new DYX C++ runtime.'
  exit 0
}
Write-Warning 'CUTOVER NOT READY: Python removal or C++ runtime cleanup still pending.'
if ($Enforce) { exit 2 }
exit 0
