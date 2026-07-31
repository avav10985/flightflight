#pragma once
#include "config.h"
#if ENABLE_WIFI_SCAN_MODE22
#include <Arduino.h>
#include "LGFX_S3.h"
// ============================================================
// WifiScanScreen:Mode 22 WiFi 環境掃描(被動診斷,2026-06-14)
//
// 列出附近 WiFi AP:名稱 / 訊號強度 / 通道 / 加密。純被動(只「聽」
// 不發射),合法。用 ESP32-S3 內建 WiFi,不用加任何模組。
// 操作:OK = 重新掃描、+/- = 捲動清單、模式開關撥走 = 離開
// ============================================================
class WifiScanScreen {
public:
  WifiScanScreen(LGFX& tft) : tft_(tft) {}
  void enter();               // 進 mode 22(呼叫端先 musicStop)
  void exit();
  void loop(int btnEdge);
private:
  void drawHeader();
  void drawList();
  void doScan();

  LGFX& tft_;
  int  count_  = 0;
  int  cursor_ = 0;
  bool busy_   = false;
};
#endif  // ENABLE_WIFI_SCAN_MODE22
