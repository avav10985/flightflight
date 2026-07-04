// 最簡 AP 測試 — 不連別人,自己當 AP 廣播。
// 手機 WiFi 列表看得到「ESP32_TEST」→ 晶片正常 WiFi 也正常
// 看不到 → 晶片本身或 WiFi 子系統有問題

#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("BOOT");

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP("ESP32_TEST", "12345678");
  Serial.printf("softAP ret=%d, IP=%s\n", ok, WiFi.softAPIP().toString().c_str());
}

void loop() {
  Serial.printf("clients=%d\n", WiFi.softAPgetStationNum());
  delay(2000);
}
