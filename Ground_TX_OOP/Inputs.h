#pragma once
#include <Arduino.h>
#include "config.h"
#include "packets.h"
// ============================================================
// Inputs:搖桿 / 三段開關 / 選單鈕 / 肩鍵
// 搖桿映射與死區邏輯是平台無關的(centerMap / 死區常數),
// 新手把換 ADC 來源時只要改 analogRead 那幾行。
// ============================================================

// ---- 選單鈕 ----
enum { BTN_NONE, BTN_PLUS, BTN_MINUS, BTN_OK, BTN_BACK };

class Inputs {
public:
  void begin() {
    pinMode(SHOULDER_L, INPUT_PULLUP);
    pinMode(SHOULDER_R, INPUT_PULLUP);
    analogReadResolution(12);
  }

  // 兩位數 mode 編碼:A 十位 + B 個位
  uint8_t readModeSwitches() { return readSwitch3(SW_A) * 10 + readSwitch3(SW_B); }

  // 按鈕單一邊緣偵測:只在「新按下」那一圈回傳 BTN_*,其餘回 BTN_NONE
  int menuBtnEdge() {
    int btn = readMenuBtn();
    bool edge = (btn != lastBtn_ && btn != BTN_NONE);
    lastBtn_ = btn;
    return edge ? btn : BTN_NONE;
  }

  bool shoulderL() { return digitalRead(SHOULDER_L) == LOW; }

  // 讀搖桿/肩鍵填 Signal(含死區與 mode 0 進場校準旗標)。
  // 回傳 true = 這一圈剛切進 mode 0(呼叫端要顯示「進場自動校準」提示)。
  bool fillSignal(Signal& s, uint8_t mode);

  // 選單/媒體清單捲動用的 pitch 位置(單次 analogRead,跟 data.pitch 同方向)
  int pitchPos() { return centerMap(analogRead(J_PITCH), REV_PITCH); }

  static int centerMap(int raw, bool reverse) {
    raw = constrain(raw, 0, 4095);
    int out = map(raw, 0, 4095, 0, 255);
    return reverse ? 255 - out : out;
  }

private:
  // ESP32 ADC 單次讀會跳幾十,多次取樣平均壓雜訊(12 次約 0.6ms,50Hz 迴圈吃得下)
  static int analogReadAvg(int pin) {
    long sum = 0;
    for (int i = 0; i < 12; i++) sum += analogRead(pin);
    return (int)(sum / 12);
  }

  static uint8_t readSwitch3(int pin) {
    int v = analogRead(pin);
    // 2026-06-06 試燒發現實體開關上下與 ADC 讀值相反,把回傳值對調
    // 撥到上 = ADC 低電位 → 回傳 0;撥到下 = ADC 高電位 → 回傳 2
    if (v > 2730) return 2;   // 下
    if (v < 1365) return 0;   // 上
    return 1;                  // 中(開路分壓中點)
  }

  // 2026-06-07 試燒實測:+ 3810~3850、− 2640~2690、OK 1910~1940、返回 1160~1200
  // 門檻取中點,邊緣最穩(舊門檻邊緣太靠近實測值,瞬間切換時容易誤觸)
  static int readMenuBtn() {
    int v = analogRead(MENU_BTN);
    if (v > 3300) return BTN_PLUS;
    if (v > 2300) return BTN_MINUS;
    if (v > 1550) return BTN_OK;
    if (v > 600)  return BTN_BACK;
    return BTN_NONE;
  }

  int           lastBtn_      = BTN_NONE;
  uint8_t       lastMode_     = 255;
  unsigned long calHoldUntil_ = 0;
};
