#pragma once
#include "config.h"
#if ENABLE_PC_MODE11
#include <Arduino.h>
#include "packets.h"
// ============================================================
// Mode 11:PC 控制(USB CDC 透明橋接)— feature 骨架,還沒接進 loop
//
// 開啟方法:#define ENABLE_PC_MODE11 1
//
// 使用流程:
// 1. PC 用 USB 接手把(會佔用 Serial 0,無法看 debug log)
// 2. 撥到 mode = 11(SW_A 中 + SW_B 中),手把進入 PC 橋接模式
// 3. PC 端 app 透過 USB Serial 送 13-byte PcCommand,手把轉成 Signal 送 NRF24
// 4. NRF24 回的 telemetry 透過 USB Serial 回 PC
//
// PC 端 app 建議:Python + pyserial + pygame.joystick + opencv(相機影像)
// 詳細實作等使用者要做時再細談
// ============================================================
struct PcCommand {
  uint8_t startByte;    // 必須 0xAA
  uint8_t throttle, pitch, roll, yaw;
  uint8_t mode;         // PC 可強制覆蓋手把開關位置
  uint8_t flags;
  uint8_t paramID;
  float   paramVal;
  uint8_t checksum;     // sum of bytes 1~11 取低 8 bit
};

inline void pcBridgeUpdate(Signal& sig, const Telemetry& tele) {
  // 從 USB Serial 讀 PC 指令,覆蓋 sig
  if (Serial.available() >= (int)sizeof(PcCommand)) {
    PcCommand cmd;
    Serial.readBytes((uint8_t*)&cmd, sizeof(cmd));
    if (cmd.startByte != 0xAA) return;
    // TODO:checksum 驗證
    sig.throttle = cmd.throttle;
    sig.pitch    = cmd.pitch;
    sig.roll     = cmd.roll;
    sig.yaw      = cmd.yaw;
    sig.mode     = cmd.mode;
    sig.flags    = cmd.flags;
    sig.paramID  = cmd.paramID;
    sig.paramVal = cmd.paramVal;
  }
  // 把當前 telemetry 推給 PC(每次都送,讓 PC 端隨時有最新狀態)
  Serial.write((const uint8_t*)&tele, sizeof(Telemetry));
}
#endif  // ENABLE_PC_MODE11
