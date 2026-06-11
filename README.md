# 四軸無人機飛行控制系統(flightflight)

從零自製的四軸無人機完整系統 —— 不使用現成飛控板與遙控器,飛行控制、無線地面站、AI 語音控制全部自行設計實作。

**📖 完整技術報告(GitHub Pages):** https://avav10985.github.io/flightflight/

## 系統組成

| 節點 | 硬體 | 功能 |
|---|---|---|
| **飛行控制器** | LOLIN D32(ESP32-WROOM-32) | 互補濾波姿態估計、三軸 PID 自穩、X 型混控、GPS 導航、失聯保護 |
| **地面站(手把)** | ESP32-S3 N16R8 + 2.8" TFT | 雙搖桿 + 9 飛行模式、50 Hz nRF24 雙向通訊、線上 PID 調參、遠端校準、語音 PTT |
| **影像節點** | ESP32-CAM | WiFi 第一人稱即時影像,固定 IP + mDNS |

## 主要功能

- **姿態自穩飛行** — 自寫飛控:MPU-6050 + 互補濾波(0.98 / 0.02)+ 角度環 PID + X 型四馬達混控
- **雙向無線鏈路** — nRF24L01+ CH 88:指令 12 B 下行 @ 50 Hz,遙測 20 B 以 ACK payload 回傳(姿態 / 高度 / 航向 / GPS)
- **線上調參與遠端校準** — 手把選單即時調 PID、觸發 IMU 快速 / 完整校準,結果寫入 NVS 斷電保存
- **AI 語音控制(Mode 10)** — 按住肩鈕說「起飛」:I²S 錄音 → 雲端 Whisper 中文辨識 → LLM 解析為 JSON 指令,約 3 秒
- **多層飛行安全** — 開機鎖定、可飛模式白名單、油門武裝條件、1 秒失聯自動斷電
- **懸停水平自學習** — 穩定懸停時自動吸收水平基準殘餘誤差,每次飛行讓下次起飛更準

## 倉庫結構

```
Drone_FC_Full/        飛行控制器主程式(LOLIN D32)
Ground_TX_ESP32/      地面站主程式(ESP32-S3)
ESP32CAM_Flight/      FPV 影像節點
Mode10_Test/          語音控制管線獨立驗證
WiFi_Min_Test/ 等     28 支單元測試程式(每個子系統一支)
docs/                 技術報告網站(GitHub Pages)
*.md                  接線總表、程式說明等設計文件
make_*.py             設計圖產生腳本
```

## 編譯需求

- Arduino IDE + ESP32 board package(arduino-esp32 3.x)
- 函式庫:RF24、LovyanGFX、MPU6050、TinyGPSPlus 等(詳見各程式開頭註解)
- 含 WiFi 功能之程式需自行建立 `secrets.h`(複製同目錄 `secrets.h.example` 填入憑證;真實憑證已由 `.gitignore` 排除)

## 安全注意事項

- 手把採雙 LDO 分軌供電:**USB 與電池嚴禁同時連接**(燒錄時斷電池、飛行時斷 USB)
- 飛行前務必於 mode 00 完成 IMU 校準;首飛建議空曠戶外、低油門測試

## 開發狀態

可飛行(手動自穩 mode 01)。GPS 導航、語音控制接入飛行鏈路、氣壓定高、雷射避障(VL53L0X × 4 硬體已安裝)開發中 —— 詳見[技術報告](https://avav10985.github.io/flightflight/)之未來工作章節。
