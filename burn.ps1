# burn.ps1 - one-click compile + upload for arduino sketches
#
# Usage:
#   .\burn.ps1 Voice_Test
#   .\burn.ps1 Voice_Test COM7
#
# Reads FQBN from <sketch>/sketch.yaml (extracts the fqbn: line).
# Uses arduino-cli compile -b $fqbn (NOT --profile) so global core +
# global libraries are used (no isolated profile cache).

param(
  [Parameter(Mandatory=$true)][string]$Sketch,
  [string]$Port = ""
)

$ErrorActionPreference = "Stop"

# Auto-refresh PATH from registry (covers winget installs from prior sessions)
$env:Path = [System.Environment]::GetEnvironmentVariable("Path","Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path","User")

# Check arduino-cli
if (-not (Get-Command arduino-cli -ErrorAction SilentlyContinue)) {
  Write-Host "[ERROR] arduino-cli not found in PATH (even after refresh)" -ForegroundColor Red
  Write-Host "Install: winget install ArduinoSA.CLI"
  exit 1
}

# Sketch folder
$sketchPath = Join-Path $PSScriptRoot $Sketch
if (-not (Test-Path $sketchPath)) {
  Write-Host "[ERROR] sketch not found: $sketchPath" -ForegroundColor Red
  Write-Host "Available sketches:"
  Get-ChildItem $PSScriptRoot -Directory | Where-Object {
    Test-Path (Join-Path $_.FullName "$($_.Name).ino")
  } | ForEach-Object { Write-Host "  $($_.Name)" }
  exit 1
}

# Extract FQBN from sketch.yaml
$sketchYaml = Join-Path $sketchPath "sketch.yaml"
if (-not (Test-Path $sketchYaml)) {
  Write-Host "[ERROR] no sketch.yaml in $sketchPath" -ForegroundColor Red
  Write-Host "Cannot determine FQBN. Create sketch.yaml with 'fqbn: ...' line."
  exit 1
}
$yamlContent = Get-Content $sketchYaml -Raw
if ($yamlContent -match "fqbn:\s*(\S+)") {
  $fqbn = $Matches[1]
} else {
  Write-Host "[ERROR] no fqbn: line found in $sketchYaml" -ForegroundColor Red
  exit 1
}

# Auto-detect COM port
if (-not $Port) {
  Write-Host "[*] Detecting connected boards..." -ForegroundColor Cyan
  $boardList = arduino-cli board list --format json | ConvertFrom-Json
  $ports = if ($boardList.detected_ports) { $boardList.detected_ports } else { $boardList }
  $candidates = @($ports | Where-Object { $_.port.address -match "COM" })

  if ($candidates.Count -eq 0) {
    Write-Host "[ERROR] No COM board detected. Check:" -ForegroundColor Red
    Write-Host "  1. USB cable plugged in?"
    Write-Host "  2. Board powered?"
    Write-Host "  3. Driver installed?"
    exit 1
  } elseif ($candidates.Count -eq 1) {
    $Port = $candidates[0].port.address
    Write-Host "[+] Auto-detected port: $Port" -ForegroundColor Green
  } else {
    Write-Host "[!] Multiple boards detected, specify port:" -ForegroundColor Yellow
    foreach ($c in $candidates) {
      $name = "unknown"
      if ($c.matching_boards) { $name = $c.matching_boards[0].name }
      Write-Host "  $($c.port.address) - $name"
    }
    Write-Host "Usage: .\burn.ps1 $Sketch COMx"
    exit 1
  }
}

# Compile + upload (uses global core + global libraries, no profile)
Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  Compile + Upload $Sketch -> $Port" -ForegroundColor Cyan
Write-Host "  FQBN: $fqbn" -ForegroundColor DarkGray
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

$startTime = Get-Date
arduino-cli compile -b $fqbn -u -p $Port $sketchPath
$exitCode = $LASTEXITCODE
$duration = (Get-Date) - $startTime

Write-Host ""
if ($exitCode -eq 0) {
  Write-Host "[OK] Done in $($duration.TotalSeconds.ToString('0.0'))s" -ForegroundColor Green
} else {
  Write-Host "[FAIL] exit $exitCode after $($duration.TotalSeconds.ToString('0.0'))s" -ForegroundColor Red
}

exit $exitCode
