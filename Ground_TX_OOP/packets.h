#pragma once
#include <stdint.h>
// ============================================================
// nRF24 封包契約(手把 ⇄ 飛機),**兩線的介面,最重要**
//
// 這個檔是「平台無關」的:不含任何 Arduino/ESP32 相依,
// 新手把(Pi Zero 2W,Linux + RF24 函式庫)可直接拿去用。
// 改任何欄位 = 改契約,FC 與手把必須成對重燒,並告知新手把主線。
// ============================================================

// 手把 → 飛機
// mode 兩位數編碼,9 個值:00/01/02/10/11/12/20/21/22
//   00=安全/校準、01=手動、02=GPS、10=語音、11=PC、12/20/21/22=預留
struct Signal {
  uint8_t throttle, pitch, roll, yaw;
  uint8_t mode;        // SW_A×10 + SW_B(各 0/1/2),例:01=A上+B中
  uint8_t flags;       // bit0=完整校準觸發
  uint8_t paramID;     // 0=無、1=Kp、2=Ki、3=Kd、4=Kp_y、5=Ki_y
  float   paramVal;
};

// 飛機 → 手把(NRF24 ACK Payload,**必須跟飛機端 Telemetry struct 完全一致**)
// 2026-06-09 移除 battery_mV(使用者沒裝),改放 altitude + heading
struct Telemetry {
  int16_t  roll;        // 角度 ×10
  int16_t  pitch;       // 角度 ×10
  int16_t  yawRate;     // 角速度 ×10°/s
  int16_t  altitude_dm; // 相對高度 ×10(decimeter,範圍 ±3276.7m)
  int16_t  heading;     // 機頭方位 ×10(0~3599,0=北 900=東 1800=南 2700=西)
  uint8_t  status;      // 多 bit:見下面 STATUS_*
  uint8_t  satCount;    // GPS 衛星數
  int32_t  lat_e7;      // 緯度 ×10^7
  int32_t  lon_e7;      // 經度 ×10^7
  // 2026-06-18 飛機回傳目前 PID(各 ×1000),開機同步顯示 + 存檔確認
  int16_t  kp_rp_m;     // Kp_rp × 1000
  int16_t  ki_rp_m;     // Ki_rp × 1000
  int16_t  kd_rp_m;     // Kd_rp × 1000
  int16_t  kp_y_m;      // Kp_y  × 1000
  int16_t  ki_y_m;      // Ki_y  × 1000
};   // 30 bytes(對齊 32 = ACK 上限,**必須跟飛機端一起重燒**)

// status 位元定義(跟飛機端一致)
#define STATUS_ARMED         0x01   // bit0:已 armed
#define STATUS_CALIBRATING   0x02   // bit1:正在校準
#define STATUS_CAL_FAILED    0x04   // bit2:上次校準失敗
#define STATUS_GPS_FIX       0x08   // bit3:GPS 有效定位

// ---- 模式名稱(中文化,2026-06-06)----
// 兩位數 mode 編碼,值為 0/1/2/10/11/12/20/21/22(9 個離散值)
inline const char* getModeName(uint8_t m) {
  switch (m) {
    case 0:  return "安全校準";   // 安全 / 校準 / 設定 hub
    case 1:  return "手動自穩";   // 手動自穩(目前唯一可飛)
    case 2:  return "GPS導航";    // GPS 自動到目標 + 避障 + 降落
    case 10: return "語音控制";   // 喚醒詞「啟動」
    case 11: return "電腦控制";   // PC 透過 USB 控制
    case 12: case 20:
    case 21: case 22:    return "保留";    // 預留
    default:             return "未知";    // 無效值
  }
}

// 哪些 mode 已實作(影響 TFT「MODE NOT IMPLEMENTED」提示)。
// 目前只有 00(安全/設定畫面)跟 01(手動飛行)有完整 UI 跟邏輯。
// 將來 02 / 10 / 11 加進來後也要更新這個 list。
inline bool modeImplemented(uint8_t m) { return m == 0 || m == 1; }
