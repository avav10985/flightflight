// ============================================================
// WiFi_Min_Test — 最小 WiFi 連線測試
//
// 目的:把 ESP32-S3 從手把模組全部拔下、只接 USB,確認能不能單純連 WiFi。
// 排除外掛硬體 / 其他程式干擾的可能。
//
// 操作:
//   1. ESP32-S3 從手把拆下,只接 USB
//   2. 複製 secrets.h.example 成 secrets.h,填 WIFI_SSID / WIFI_PASS
//   3. 燒這支 sketch
//   4. Serial Monitor 看
// ============================================================

#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "secrets.h"

void setup() {
  // 暫時關 BOD
  REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_INT_ENA);
  REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(500);

  Serial.println("\n=== WiFi_Min_Test ===");
  Serial.printf("SDK: %s\n", ESP.getSdkVersion());
  Serial.printf("Free heap: %u\n", ESP.getFreeHeap());
  Serial.printf("PSRAM size: %u\n", ESP.getPsramSize());
  Serial.printf("Flash: %u\n", ESP.getFlashChipSize());
  Serial.flush();

  delay(200);
  Serial.printf("[1] WiFi.mode(STA) 前 ...\n");  Serial.flush();
  WiFi.mode(WIFI_STA);
  Serial.printf("[2] WiFi.mode(STA) 完成\n");    Serial.flush();
  delay(100);

  Serial.printf("[3] WiFi.begin(%s) 前 ...\n", WIFI_SSID); Serial.flush();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[4] WiFi.begin() 已返回,等連線 ...\n"); Serial.flush();

  // begin 返回後再降功率(避免 begin 前呼叫某些 SDK 版本會炸)
  WiFi.setTxPower(WIFI_POWER_2dBm);
  Serial.printf("[5] setTxPower(2dBm) 完成\n");  Serial.flush();

  unsigned long wt0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wt0 < 20000) {
    delay(500);
    Serial.printf(". status=%d\n", WiFi.status()); Serial.flush();
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[+] WiFi OK,IP=%s,RSSI=%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.printf("[X] WiFi 失敗,狀態碼 %d\n", WiFi.status());
  }
}

void loop() {
  // 每秒印一行心跳,看晶片有沒有真的在跑
  static uint32_t cnt = 0;
  Serial.printf("[heartbeat %lu] WiFi=%d RSSI=%d heap=%u\n",
                cnt++, WiFi.status(), WiFi.RSSI(), ESP.getFreeHeap());
  Serial.flush();
  delay(1000);
}
