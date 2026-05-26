// ============================================================
// GY-NEO6MV2 GPS 獨立測試程式
// MCU: LOLIN D32 (ESP32)
//
// 用途：驗證 GPS 接線、模組有沒有壞、能否定位
//
// 接線：
//   GPS 模組         LOLIN D32
//   ──────────       ──────────
//   VCC ────────────→ 3V 或 USB(5V)
//   GND ────────────→ GND
//   TX  ────────────→ GPIO 16 (RX2)   ← GPS 講話 ESP32 聽
//   RX  ────────────→ GPIO 17 (TX2)   ← ESP32 設定 GPS 用
//
// ⚠️ 重要：GPS 必須在「室外開闊處」才能收到衛星！
//   - 室內 / 窗邊 / 大樓間 → 通常收不到，這正常
//   - 第一次冷啟動 30 秒 ~ 5 分鐘才有 fix
//   - 模組上的紅 LED 開始閃爍 = 有定位（沒閃 = 還沒）
// ============================================================

#include <TinyGPSPlus.h>

#define PIN_GPS_RX  16   // ESP32 接收 GPS 講的話
#define PIN_GPS_TX  17   // ESP32 設定 GPS 用
#define GPS_BAUD    9600 // NEO-6M 預設

TinyGPSPlus gps;

unsigned long startTime;
unsigned long charsReceived = 0;
unsigned long lastDbgTime = 0;
unsigned long lastRawDumpEnd = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("  GY-NEO6MV2 GPS 測試程式");
  Serial.println("========================================");
  Serial.printf("UART2: RX=GPIO%d, TX=GPIO%d, %d baud\n",
                PIN_GPS_RX, PIN_GPS_TX, GPS_BAUD);

  Serial2.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  startTime = millis();
  lastRawDumpEnd = millis() + 10000;   // 前 10 秒印原始 NMEA

  Serial.println("\n----- 階段 1：前 10 秒原始 NMEA 輸出 -----");
  Serial.println("應該看到一堆 $GPGGA、$GPRMC 開頭的字串");
  Serial.println("沒有 → 接線錯 / TX RX 對調 / 模組壞");
  Serial.println();
}

void loop() {
  // ----- 階段 1：前 10 秒印原始字元 -----
  while (Serial2.available()) {
    char c = Serial2.read();
    charsReceived++;

    if (millis() < lastRawDumpEnd) {
      Serial.write(c);
    } else {
      // 階段 2：交給 TinyGPS 解析
      gps.encode(c);
    }
  }

  // ----- 階段 1 結束 → 印統計 -----
  static bool stage1Done = false;
  if (!stage1Done && millis() >= lastRawDumpEnd) {
    stage1Done = true;
    Serial.println();
    Serial.println("\n----- 階段 1 結束 -----");
    Serial.printf("收到 %lu 個字元\n", charsReceived);

    if (charsReceived == 0) {
      Serial.println("❌ 完全沒收到資料！");
      Serial.println("   檢查：VCC 通電？TX→GPIO16？RX→GPIO17？GND 共地？");
      Serial.println("   注意：GPS 的 TX 接 ESP32 的 GPIO16（不是 17）");
    } else if (charsReceived < 100) {
      Serial.println("⚠️  資料太少，可能 baud rate 錯或接觸不良");
    } else {
      Serial.println("✅ UART 通訊正常，開始解析 NMEA");
    }

    Serial.println();
    Serial.println("----- 階段 2：解析後狀態（每 2 秒一次）-----");
    Serial.println("等待衛星訊號（室外可能 30 秒～5 分鐘）...");
    Serial.println();
  }

  // ----- 階段 2：每 2 秒印解析狀態 -----
  if (stage1Done && millis() - lastDbgTime > 2000) {
    lastDbgTime = millis();
    printStatus();
  }
}

void printStatus() {
  unsigned long secs = (millis() - startTime) / 1000;
  Serial.printf("[%3lus] ", secs);

  // 接收統計
  Serial.printf("收=%lu 解析=%lu 失敗=%lu | ",
                charsReceived,
                gps.sentencesWithFix(),
                gps.failedChecksum());

  // 衛星數
  if (gps.satellites.isValid()) {
    Serial.printf("Sat:%2d ", gps.satellites.value());
  } else {
    Serial.printf("Sat:?? ");
  }

  // 位置
  if (gps.location.isValid()) {
    Serial.printf("Lat:%9.5f Lon:%10.5f ",
                  gps.location.lat(), gps.location.lng());
  } else {
    Serial.printf("Lat:--------- Lon:---------- ");
  }

  // 高度
  if (gps.altitude.isValid()) {
    Serial.printf("Alt:%5.1fm ", gps.altitude.meters());
  }

  // HDOP（精度，越小越好）
  if (gps.hdop.isValid()) {
    Serial.printf("HDOP:%.1f ", gps.hdop.hdop());
  }

  // UTC 時間
  if (gps.time.isValid()) {
    Serial.printf("UTC:%02d:%02d:%02d",
                  gps.time.hour(), gps.time.minute(), gps.time.second());
  }

  Serial.println();

  // ----- 取得第一次 fix 時報告 -----
  static bool firstFix = false;
  if (!firstFix && gps.location.isValid()) {
    firstFix = true;
    Serial.println();
    Serial.println("✅✅✅ 取得 GPS Fix！晶片正常 ✅✅✅");
    Serial.printf("   位置：%.6f, %.6f\n", gps.location.lat(), gps.location.lng());
    Serial.printf("   高度：%.1f m\n", gps.altitude.meters());
    Serial.printf("   花了 %lu 秒\n", secs);
    Serial.println();
  }

  // ----- 5 分鐘後還沒 fix 警告 -----
  static bool warnedNoFix = false;
  if (!warnedNoFix && secs > 300 && !gps.location.isValid()) {
    warnedNoFix = true;
    Serial.println();
    Serial.println("⚠️  5 分鐘還沒 fix，可能：");
    Serial.println("    1. 在室內（GPS 訊號被遮蔽）→ 帶到戶外開闊處");
    Serial.println("    2. 天線方向錯 → 模組上的陶瓷天線朝上");
    Serial.println("    3. 模組壞或天線壞");
    Serial.println("    （但 UART 有資料 = 晶片活，只是收不到衛星）");
    Serial.println();
  }
}
