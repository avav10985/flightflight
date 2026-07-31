#include "WifiScan.h"
#if ENABLE_WIFI_SCAN_MODE22
#include <WiFi.h>
#include "Inputs.h"   // BTN_* 列舉

#define SCAN_MAX_SHOW 32

void WifiScanScreen::drawHeader() {
  tft_.fillScreen(TFT_BLACK);
  tft_.fillRect(0, 0, 240, 32, TFT_DARKCYAN);
  tft_.setFont(&fonts::efontTW_24);
  tft_.setTextColor(TFT_WHITE, TFT_DARKCYAN);
  tft_.setCursor(18, 4);
  tft_.print("WiFi 環境掃描");
  tft_.setFont(&fonts::efontTW_14);
  tft_.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft_.setCursor(5, 300);
  tft_.print("OK:重掃  +/-:捲動  切模式:離開");
}

void WifiScanScreen::drawList() {
  const int VIS = 8, rowH = 30;
  tft_.fillRect(0, 36, 240, 260, TFT_BLACK);
  if (busy_) {
    tft_.setFont(&fonts::efontTW_24);
    tft_.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft_.setCursor(10, 130);
    tft_.print("掃描中...");
    return;
  }
  if (count_ <= 0) {
    tft_.setFont(&fonts::efontTW_24);
    tft_.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft_.setCursor(10, 130);
    tft_.print("找不到 WiFi");
    return;
  }
  if (cursor_ > count_ - 1) cursor_ = count_ - 1;
  int top = cursor_ - (cursor_ % VIS);   // 簡單分頁
  for (int r = 0; r < VIS && top + r < count_; r++) {
    int i = top + r;
    int y = 38 + r * rowH;
    bool sel = (i == cursor_);
    int rssi = WiFi.RSSI(i);
    // 訊號強度顏色:強綠 / 中黃 / 弱紅
    uint16_t sigColor = (rssi > -60) ? TFT_GREEN : (rssi > -75) ? TFT_YELLOW : TFT_RED;
    bool enc = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    if (sel) tft_.fillRect(0, y, 240, rowH, TFT_NAVY);
    tft_.setFont(&fonts::efontTW_16);
    // 第一行:鎖頭 + SSID(截斷)
    tft_.setTextColor(sel ? TFT_WHITE : TFT_LIGHTGREY, sel ? TFT_NAVY : TFT_BLACK);
    tft_.setCursor(6, y + 1);
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0) ssid = "(隱藏)";
    if (ssid.length() > 13) ssid = ssid.substring(0, 13);
    tft_.printf("%s%s", enc ? "*" : " ", ssid.c_str());
    // 第二行:通道 + RSSI(上色)
    tft_.setTextColor(sigColor, sel ? TFT_NAVY : TFT_BLACK);
    tft_.setCursor(150, y + 1);
    tft_.printf("CH%d", WiFi.channel(i));
    tft_.setCursor(150, y + 14);
    tft_.printf("%ddBm", rssi);
  }
  // 右下角計數
  tft_.setFont(&fonts::efontTW_14);
  tft_.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft_.setCursor(195, 282);
  tft_.printf("%d/%d", cursor_ + 1, count_);
}

void WifiScanScreen::doScan() {
  busy_ = true;
  drawList();
  WiFi.scanDelete();
  count_ = WiFi.scanNetworks(false, true);   // 同步掃,含隱藏
  if (count_ > SCAN_MAX_SHOW) count_ = SCAN_MAX_SHOW;
  cursor_ = 0;
  busy_ = false;
  drawList();
}

void WifiScanScreen::enter() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  drawHeader();
  doScan();
}

void WifiScanScreen::exit() {
  WiFi.scanDelete();
  // WiFi 留著(Mode 10 語音還要用),不關
}

void WifiScanScreen::loop(int btnEdge) {
  if (btnEdge == BTN_OK) { doScan(); return; }
  if (count_ > 0) {
    if (btnEdge == BTN_PLUS  && cursor_ < count_ - 1) { cursor_++; drawList(); }
    if (btnEdge == BTN_MINUS && cursor_ > 0)          { cursor_--; drawList(); }
  }
}
#endif  // ENABLE_WIFI_SCAN_MODE22
