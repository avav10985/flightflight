# Retro-Go 全主機模擬器 — flightflight 手把 target

把手把變成多主機掌機:**NES + Game Boy + Game Boy Color + Sega Master System + Game Gear**,
附 ROM 選單與存檔。這是 [Retro-Go](https://github.com/ducalex/retro-go) 的自訂硬體 target。

> NES 已有獨立的 `NES_Test`(arduino-nofrendo)可玩;Retro-Go 是「全家桶」升級版,
> 一次涵蓋全部主機,但需要 ESP-IDF 工具鏈(非 Arduino)。

## 這個資料夾是什麼

`config.h` / `env.py` / `sdkconfig` 三個檔 = Retro-Go 的一個「target」(描述我們手把硬體)。
本資料夾是**備份**;實際建置時要放回 clone 下來的 retro-go 專案裡。

腳位全部沿用手把現有接線(同 Ground_TX_ESP32 / NES_Test),**不用改線**:

| 功能 | 接線 |
|---|---|
| 顯示 ILI9341 | SPI2:CLK38 / MOSI39 / MISO40 / CS48 / DC21 / RST16 |
| SD 卡 | SPI3:CLK15 / MOSI17 / MISO47 / CS0 |
| 喇叭 MAX98357A | I2S:BCK11 / WS12 / DATA13 / SD(enable)18 |
| 上下 | 右搖桿 Y(GPIO4 / ADC1_CH3) |
| 左右 | 左搖桿 X(GPIO2 / ADC1_CH1) |
| A | 右肩鈕 GPIO9 |
| MENU(選單/存檔) | 左肩鈕 GPIO8 |
| B / START / SELECT | 按鍵叢 GPIO7(+ / OK / 返回) |

## 安裝與建置步驟

### 1. 裝 ESP-IDF(一次性,~2GB)

Retro-Go 需要 ESP-IDF 4.4~5.3。Windows 最簡單用官方離線安裝程式:
- 下載 https://dl.espressif.com/dl/esp-idf/ 的 **ESP-IDF v5.1 Offline Installer**
- 安裝時勾 ESP32-S3 支援,裝完桌面會有「ESP-IDF 5.1 PowerShell」捷徑

### 2. clone retro-go + 放入 target

```powershell
cd C:\Users\<你>\Documents
git clone https://github.com/ducalex/retro-go.git
# 把本資料夾複製成 retro-go 的 target:
xcopy /E /I "D:\無人機\flightflight\Retro_Go_flightflight" "retro-go\components\retro-go\targets\flightflight"
```

接著在 `retro-go\components\retro-go\config.h` 的 target 清單裡加一行
(找到 `RG_TARGET_ESP32_S3_DEVKIT` 那兩行,後面插入):

```c
#elif defined(RG_TARGET_FLIGHTFLIGHT)
#include "targets/flightflight/config.h"
```

### 3. 建置 + 燒錄(一鍵)

開「ESP-IDF PowerShell」→ cd 到 retro-go →

```powershell
python rg_tool.py install --target=flightflight --port=COM7
```

(COM7 換成你的埠;`install` = 編譯所有模擬器 app + 打包成單一映像 + 全燒進去)

### 4. 放 ROM 上 SD 卡

SD 卡建資料夾分主機放(Retro-Go 慣例):
```
/roms/nes/*.nes
/roms/gb/*.gb        /roms/gbc/*.gbc
/roms/sms/*.sms      /roms/gg/*.gg
```
⚠ 版權:demo 用 homebrew 免費遊戲。

接電池開機 → Retro-Go 選單列出各主機 → 選遊戲。MENU 鍵(左肩)叫出存檔 / 讀檔 / 離開。

## 待調(實機才知道)

- **畫面方向 / 顏色**:config.h 的 `0x36` MADCTL byte(現 0x68 橫向)。顛倒或藍橘反就調這個。
- **搖桿閾值**:ADC map 的 3072 / 1024 門檻,太靈敏或不夠靈敏就調。
- **按鍵叢範圍**:600 / 1550 / 2300 / 3300 沿用 Ground_TX,理論上一致。
- 編譯第一次很久(把多個模擬器都編一遍),正常。
