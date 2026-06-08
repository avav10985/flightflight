# burn.ps1 - 一鍵編譯 + 上傳 arduino sketch
#
# 用法:
#   .\burn.ps1 Voice_Test                  ← 自動偵測 COM port
#   .\burn.ps1 Voice_Test COM7             ← 指定 COM port
#   .\burn.ps1 Drone_FC_Full COM8
#
# 設定來自每個 sketch 資料夾的 sketch.yaml(已預設好):
#   ESP32-S3-N16R8(手把)→ sketch.yaml profile esp32s3_n16r8
#   LOLIN D32 ESP32(飛機)→ sketch.yaml profile lolin_d32
#
# 需要先裝 arduino-cli:
#   choco install arduino-cli         (Windows + Chocolatey)
#   或 https://arduino.github.io/arduino-cli/

param(
  [Parameter(Mandatory=$true)][string]$Sketch,
  [string]$Port = ""
)

# 確認 arduino-cli 在 PATH
if (-not (Get-Command arduino-cli -ErrorAction SilentlyContinue)) {
  Write-Host "❌ 找不到 arduino-cli。安裝方法:" -ForegroundColor Red
  Write-Host "    1. choco install arduino-cli  (Chocolatey)"
  Write-Host "    2. 或 https://arduino.github.io/arduino-cli/latest/installation/"
  exit 1
}

# 確認 sketch 資料夾存在
$sketchPath = Join-Path $PSScriptRoot $Sketch
if (-not (Test-Path $sketchPath)) {
  Write-Host "❌ 找不到 sketch:$sketchPath" -ForegroundColor Red
  Write-Host "可用的 sketch:" -ForegroundColor Yellow
  Get-ChildItem $PSScriptRoot -Directory | Where-Object {
    Test-Path (Join-Path $_.FullName "$($_.Name).ino")
  } | ForEach-Object { Write-Host "  $($_.Name)" }
  exit 1
}

# 自動偵測 COM port(如果使用者沒指定)
if (-not $Port) {
  Write-Host "🔍 偵測連接的板..." -ForegroundColor Cyan
  $boardList = arduino-cli board list --format json | ConvertFrom-Json
  $candidates = $boardList | Where-Object { $_.port.address -match "COM" }

  if ($candidates.Count -eq 0) {
    Write-Host "❌ 沒偵測到任何 COM 板。檢查:" -ForegroundColor Red
    Write-Host "    1. USB 線插好了嗎?"
    Write-Host "    2. 板上電了嗎?"
    Write-Host "    3. 驅動程式裝了嗎?(CP210x / CH340)"
    exit 1
  } elseif ($candidates.Count -eq 1) {
    $Port = $candidates[0].port.address
    Write-Host "✅ 自動偵測 port:$Port" -ForegroundColor Green
  } else {
    Write-Host "⚠️ 偵測到多個板,請指定 port:" -ForegroundColor Yellow
    foreach ($c in $candidates) {
      $name = if ($c.matching_boards) { $c.matching_boards[0].name } else { "未識別" }
      Write-Host "  $($c.port.address) — $name"
    }
    Write-Host "用法:.\burn.ps1 $Sketch COMx"
    exit 1
  }
}

# 編譯 + 上傳
Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  編譯 + 上傳 $Sketch → $Port" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

$startTime = Get-Date
arduino-cli compile -u -p $Port $sketchPath
$exitCode = $LASTEXITCODE
$duration = (Get-Date) - $startTime

Write-Host ""
if ($exitCode -eq 0) {
  Write-Host "✅ 完成!花了 $($duration.TotalSeconds.ToString('0.0')) 秒" -ForegroundColor Green
} else {
  Write-Host "❌ 失敗(exit $exitCode),花了 $($duration.TotalSeconds.ToString('0.0')) 秒" -ForegroundColor Red
}

exit $exitCode
