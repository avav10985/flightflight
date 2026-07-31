#pragma once
#include "config.h"
#if ENABLE_PERSISTENT_MENU
#include <Arduino.h>
// ============================================================
// 常駐選單(任何 mode TFT 下方 30px 區域常駐顯示)— feature 骨架,未實作
//
// 開啟:#define ENABLE_PERSISTENT_MENU 1
// 進入:肩鍵 L 長按 1 秒
// 操作:+ - 切換項目,OK 進入該功能,返回 退出
// 項目:音樂、亮度(預留,MSP2806 LED 直連 3V3 無法軟控)、系統資訊
//
// 設計上跟 mode 0 的 PID 選單分開:那是「設定」,常駐選單是「附屬功能」
// ============================================================
enum PMItem { PM_MUSIC, PM_BRIGHTNESS, PM_INFO, PM_COUNT };
// inline:header 全域,多個 .cpp include 也不會 multiple definition
inline const char* PM_NAMES[PM_COUNT] = { "音樂", "亮度", "系統資訊" };

inline bool          pmActive       = false;
inline int           pmCursor       = 0;
inline unsigned long pmShoulderDown = 0;   // 用來偵測長按

inline void persistentMenuUpdate() {
  bool shldL = (digitalRead(SHOULDER_L) == LOW);
  if (shldL) {
    if (pmShoulderDown == 0) pmShoulderDown = millis();
    else if (millis() - pmShoulderDown > 1000 && !pmActive) {
      pmActive = true;
      // TODO:暫停其他 UI 重畫,畫常駐選單
    }
  } else {
    pmShoulderDown = 0;
  }
  // pmActive 時 + - 改 cursor,OK 進入該功能,返回 退出
  // TODO
}

inline void persistentMenuDraw() {
  // 在 TFT 底部畫常駐選單
  // TODO:fillRect 區域 + 畫 PM_NAMES[pmCursor]
}
#endif  // ENABLE_PERSISTENT_MENU
