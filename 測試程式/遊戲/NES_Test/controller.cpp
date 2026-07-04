// NES 控制器 — flightflight 手把 V2-A 自訂版
//
// 十字鍵:右搖桿 ADC(注意:手把搖桿軸向反相 — 推上 ADC 高、推右 ADC 低)
// A / B :右肩鈕 / 左肩鈕(digital, 內部上拉, 按下 = LOW)
// Start / Select:按鍵叢電阻階梯(單一 ADC 腳,閾值同 Ground_TX readMenuBtn)
//
// 回傳格式(同 arduino-nofrendo 範例,低態有效再整體反相):
//   bit0=up 1=down 2=left 3=right 4=select 5=start 6=a 7=b 8=x 9=y

#include <Arduino.h>
#include "hw_config.h"

extern "C" void controller_init()
{
  pinMode(PIN_JOY_UD, INPUT);
  pinMode(PIN_JOY_LR, INPUT);
  pinMode(PIN_BTN_A, INPUT_PULLUP);
  pinMode(PIN_BTN_B, INPUT_PULLUP);
  pinMode(PIN_BTN_LADDER, INPUT);
}

extern "C" uint32_t controller_read_input()
{
  // 1 = 沒按(低態有效)
  uint32_t u = 1, d = 1, l = 1, r = 1, s = 1, t = 1, a = 1, b = 1;

  // 搖桿:手把軸向反相(推上 = ADC 高、推右 = ADC 低)
  int joyY = analogRead(PIN_JOY_UD);
  int joyX = analogRead(PIN_JOY_LR);
  if (joyY > 3072)      u = 0;   // 推上
  else if (joyY < 1024) d = 0;   // 推下
  if (joyX < 1024)      r = 0;   // 推右(反相:右 = 低)
  else if (joyX > 3072) l = 0;   // 推左

  // A = 右肩鈕;B = 按鍵叢「+」鍵(左肩太難按,2026-06-12 改)
  // 左肩保留當備用 B(兩個都能用)
  a = digitalRead(PIN_BTN_A);
  b = digitalRead(PIN_BTN_B);

  // 按鍵叢階梯(閾值同 Ground_TX):>3300 加 / >2300 減 / >1550 OK / >600 返回
  int v = analogRead(PIN_BTN_LADDER);
  if (v > 3300)                  b = 0;   // 「+」 → B(主力)
  else if (v > 1550 && v <= 2300) t = 0;  // OK  → Start
  else if (v > 600 && v <= 1550)  s = 0;  // 返回 → Select

  return 0xFFFFFFFF ^ ((!u << 0) | (!d << 1) | (!l << 2) | (!r << 3) |
                       (!s << 4) | (!t << 5) | (!a << 6) | (!b << 7));
}
