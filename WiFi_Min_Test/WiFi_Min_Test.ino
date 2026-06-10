// 最簡 WiFi 測試 — 沒 BOD、沒 setTxPower、沒任何花俏
// 想驗證是不是先前那些設定害的

#include <WiFi.h>
#include "secrets.h"

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("BOOT");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("AFTER BEGIN");
}

void loop() {
  Serial.printf("status=%d rssi=%d\n", WiFi.status(), WiFi.RSSI());
  delay(1000);
}
