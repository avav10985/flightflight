#pragma once
#include <Arduino.h>
#include <RF24.h>
#include "packets.h"
// ============================================================
// FlightLink:nRF24 飛控鏈路(送 Signal、收 Telemetry ACK payload)
//
// 平台無關核心:RF24 函式庫在 Raspberry Pi(Linux)有同一套 API,
// 新手把(Pi Zero 2W)可直接沿用這個類別 + packets.h。
// (millis() 在 RF24 的 Linux 相容層也有提供)
// ============================================================
class FlightLink {
public:
  FlightLink(uint8_t cePin, uint8_t csnPin, uint64_t pipeOut, uint8_t channel)
    : radio_(cePin, csnPin), pipe_(pipeOut), channel_(channel) {}

  // radio 初始化(SPI 要先 begin)。回傳 radio.begin() 結果;
  // 就算失敗也照原版把參數設完(debug 畫面還是會顯示 FAIL)。
  bool begin();

  bool chipConnected() { return radio_.isChipConnected(); }

  // 送一包 Signal;write 成功且有 ACK payload 就順手收 Telemetry。
  // teleOK 邏輯與原版一致:write 失敗 → false;成功但沒 ACK → 維持原值。
  bool send(const Signal& sig);

  const Telemetry& telemetry() const { return tele_; }
  bool teleOK() const { return teleOK_; }
  unsigned long lastTeleMs() const { return lastTeleMs_; }

  // 最近 maxAgeMs 內收過遙測 = 飛機真的在線
  // (校準結果/PID 存檔確認不能在飛機沒開時誤報完成)
  bool linkFresh(unsigned long maxAgeMs = 1000) const {
    return millis() - lastTeleMs_ < maxAgeMs;
  }

  // TX 統計(debug 用,.ino 每秒印一次後自行歸零)
  uint32_t txOk = 0, txFail = 0;

private:
  RF24          radio_;
  uint64_t      pipe_;
  uint8_t       channel_;
  Telemetry     tele_{};
  bool          teleOK_     = false;
  unsigned long lastTeleMs_ = 0;
};
