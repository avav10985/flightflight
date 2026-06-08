# mon.ps1 - open serial monitor on auto-detected ESP32 board
# Usage:  .\mon.ps1           (auto-detect COM port, 115200 baud)
#         .\mon.ps1 COM7      (specify port)
#         .\mon.ps1 COM7 9600 (specify port + baudrate)

param(
  [string]$Port = "",
  [int]$Baud = 115200
)

if (-not $Port) {
  $boardList = arduino-cli board list --format json | ConvertFrom-Json
  $ports = if ($boardList.detected_ports) { $boardList.detected_ports } else { $boardList }
  $candidates = @($ports | Where-Object { $_.port.address -match "COM" })
  if ($candidates.Count -eq 0) {
    Write-Host "[ERROR] No COM board detected" -ForegroundColor Red
    exit 1
  } elseif ($candidates.Count -eq 1) {
    $Port = $candidates[0].port.address
  } else {
    Write-Host "Multiple boards detected, specify port:" -ForegroundColor Yellow
    foreach ($c in $candidates) { Write-Host "  $($c.port.address)" }
    exit 1
  }
}

Write-Host "[*] Monitoring $Port @ $Baud (Ctrl+C to exit)" -ForegroundColor Cyan
arduino-cli monitor -p $Port -c "baudrate=$Baud"
