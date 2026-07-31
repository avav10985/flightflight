#pragma once
#include <stdint.h>
// ============================================================
// 地面站 V2-A 硬體設定:腳位 / feature 旗標 / NRF24 常數 / 方向反轉
// (OOP 重構版,內容從 Ground_TX_ESP32.ino 原封搬來,含歷程註解)
//
// 開發板:ESP32S3 Dev Module;USB CDC On Boot=Enabled;
//          Flash=16MB;PSRAM=OPI PSRAM;Partition=16M with OTA。
// 硬體接線見 手把接線總表_V2.md。
// ============================================================

// ---- 腳位（V2-A 已定案,見 手把接線總表_V2.md）----
// 2026-06-04 GPIO 2 ↔ 10 對調:左搖桿訊號集中右側、右搖桿集中左側
// 2026-06-07 試燒實測:使用者只接了油門(GPIO 1)+ 右搖桿 Y(GPIO 4)
//   GPIO 2(左 X)、GPIO 10(右 X) 兩條線還沒接
//   把 J_PITCH 改到 GPIO 4(有訊號),J_ROLL 留 GPIO 10(以後接 右 X 才動)
#define J_THROTTLE   1    // 左搖桿 Y(油門,拆彈簧)— ✅ 已接
#define J_LEFT_X     2    // 左搖桿 X(可選 yaw)— ❌ 還沒接
#define J_PITCH      4    // 右搖桿 Y(俯仰)— ✅ 已接(從 GPIO 10 對調過來)
#define J_ROLL      10    // 右搖桿 X(翻滾)— ❌ 還沒接(從 GPIO 4 對調過來)
#define SW_A         5
#define SW_B         6
#define MENU_BTN     7
#define SHOULDER_L   8
#define SHOULDER_R   9

// TFT LED:MSP2806 的 LED 腳沒內接 VCC,**直接外接 B 板 3V3 軌常亮**(不走 GPIO/PWM)
// GPIO 15 因此回歸「真正釋出」可用腳
#define PIN_NRF_CE   41
#define PIN_NRF_CSN  42
#define PIN_SPI_SCK  38
#define PIN_SPI_MOSI 39
#define PIN_SPI_MISO 40
// SD 卡:獨立第二組 SPI(SPI3/HSPI),跟 NRF24/TFT 的 bus 完全分開。
// 2026-06-12 實測:SD 模組 level shifter 在共用 bus 上不放開 MISO,
// 連新電源架構也救不了 → 搬到專用腳位,讓它佔自己的線。
// MISO 曾誤選 GPIO 3(接線總表標「避開」,實測也失敗),改用 47;
// CS 回歸接線總表原始設計 GPIO 0(BOOT 腳輸出閒置 HIGH 安全,R11 上拉)
#define PIN_SD_CS    0   // SD CS(原始設計,R11 10k 上拉)
#define PIN_SD_SCK  15   // SD 專用 SCK(真正釋出腳)
#define PIN_SD_MOSI 17   // SD 專用 MOSI(真正釋出腳)
#define PIN_SD_MISO 47   // SD 專用 MISO(原 CS 腳讓出來)

// ====== V2-B 跟其他 feature 旗標(2026-06-06)======
// 硬體 / API key 齊全才改 1,**程式編譯時不會占用太多 flash**
#define ENABLE_PC_MODE11        0   // Mode 11:PC 透過 USB Serial 控制飛機(透明橋接)
#define ENABLE_VOICE_MODE10     1   // Mode 10:語音控制(需 secrets.h + WiFi 熱點,2026-06-12 開)
#define ENABLE_MUSIC            1   // 任意 mode 都可放音樂(需 SD + MAX98357A,2026-06-12 開)
#define ENABLE_VIDEO_MODE12     1   // Mode 12:媒體模式,SD /video/*.mjp 影片播放
                                    // (需 ENABLE_MUSIC=1 共用 I²S + JPEGDEC 函式庫)
#define ENABLE_WIFI_SCAN_MODE22 1   // Mode 22:WiFi 環境掃描(被動,列附近 AP 訊號/通道)
#define ENABLE_PERSISTENT_MENU  0   // TFT 下方常駐選單(肩鍵 L 長按 1 秒進入)
#define ENABLE_SD               1   // SD 卡模組:1 = 啟用(2026-06-11 新電源架構後重測。
                                    // 2026-06-09 曾因模組 level shifter 搶 SPI bus 拖爆 NRF24 停用;
                                    // 若 chip connected 變 NO 或飛機連不上,改回 0)
#define PIN_I2S_BCLK   11           // V2-B 語音模組腳位(共用 INMP441 + MAX98357A)
#define PIN_I2S_WS     12
#define PIN_I2S_DOUT   13           // → MAX98357A DIN
#define PIN_I2S_DIN    14           // INMP441 SD →
#define PIN_AMP_SD     18           // MAX98357A SD(shutdown):LOW=休眠靜音、HIGH=啟用
                                    // 預設拉低消除「沙沙」雜訊,Mode 10 / 音樂 要播放時設 HIGH

// ---- 方向反轉（測試後不對就改）----
const bool REV_THROTTLE = true;
const bool REV_PITCH    = true;
const bool REV_ROLL     = true;
// yaw 方向若相反，把 SHOULDER_L / SHOULDER_R 兩腳對調即可

// ---- NRF24 ----
// 2026-06-07 換新 pipe + channel(跟 Drone_FC_Full PIPE_IN 必須一致)
const uint64_t PIPE_OUT    = 0x4E6F9C2D5BLL;
const uint8_t  NRF_CHANNEL = 88;

#if ENABLE_VIDEO_MODE12 && !ENABLE_MUSIC
#error "ENABLE_VIDEO_MODE12 需要 ENABLE_MUSIC=1(共用 I2S TX 通道)"
#endif
#if ENABLE_VOICE_MODE10 && !ENABLE_MUSIC
#error "ENABLE_VOICE_MODE10 需要 ENABLE_MUSIC=1(I2S 全雙工共用時脈)"
#endif
