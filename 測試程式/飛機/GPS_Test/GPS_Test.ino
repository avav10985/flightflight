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
#include <BluetoothSerial.h>

#define PIN_GPS_RX  16   // ESP32 接收 GPS 講的話
#define PIN_GPS_TX  17   // ESP32 設定 GPS 用
#define GPS_BAUD    9600 // NEO-6M 預設

TinyGPSPlus gps;
BluetoothSerial SerialBT;   // 藍牙序列埠（手機看用）

// 印到 USB + 藍牙
void out(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  SerialBT.print(buf);
}
void outln(const char *s = "") {
  Serial.println(s);
  SerialBT.println(s);
}

// 解析 $GPGSV 取得「可見衛星」+「個別訊號強度 SNR」
TinyGPSCustom satsInView(gps, "GPGSV", 3);   // GPGSV 第 3 欄 = 可見衛星總數

// 12 顆衛星的 PRN 和 SNR（NMEA $GPGSV 每句包 4 顆，循環送）
TinyGPSCustom satPRN[12];
TinyGPSCustom satSNR[12];

unsigned long startTime;
unsigned long charsReceived = 0;
unsigned long lastDbgTime = 0;
unsigned long lastRawDumpEnd = 0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("GPS_TEST_D32");   // 手機要配對的藍牙名稱
  delay(500);
  outln("\n========================================");
  outln("  GY-NEO6MV2 GPS 測試程式（含藍牙輸出）");
  outln("========================================");
  outln("藍牙裝置名稱：GPS_TEST_D32");
  out("UART2: RX=GPIO%d, TX=GPIO%d, %d baud\n",
      PIN_GPS_RX, PIN_GPS_TX, GPS_BAUD);

  // 設定 $GPGSV 自定義欄位（每顆衛星 PRN + SNR）
  // GPGSV 格式：$GPGSV,total_msgs,msg_idx,sats_view, PRN1,Elev1,Azim1,SNR1, PRN2,Elev2,Azim2,SNR2, ...
  for (int i = 0; i < 12; i++) {
    int slot = i % 4;
    int term = 4 + slot * 4;        // PRN 在 term 4, 8, 12, 16
    satPRN[i].begin(gps, "GPGSV", term);
    satSNR[i].begin(gps, "GPGSV", term + 3);   // SNR 在 PRN+3
  }

  Serial2.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  startTime = millis();
  lastRawDumpEnd = millis() + 10000;   // 前 10 秒印原始 NMEA

  outln("\n----- 階段 1：前 10 秒原始 NMEA（只在 USB 序列埠）-----");
  outln("USB 端應該看到一堆 $GPGGA、$GPRMC 開頭的字串");
  outln("沒有 → 接線錯 / TX RX 對調 / 模組壞");
  outln();
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
    outln();
    outln("\n----- 階段 1 結束 -----");
    out("收到 %lu 個字元\n", charsReceived);

    if (charsReceived == 0) {
      outln("❌ 完全沒收到資料！");
      outln("   檢查：VCC 通電？TX→GPIO16？RX→GPIO17？GND 共地？");
    } else if (charsReceived < 100) {
      outln("⚠️  資料太少，可能 baud rate 錯或接觸不良");
    } else {
      outln("✅ UART 通訊正常，開始解析 NMEA");
    }

    outln();
    outln("----- 階段 2：解析後狀態（每 2 秒一次）-----");
    outln("等待衛星訊號（室外可能 30 秒～5 分鐘）...");
    outln();
  }

  // ----- 階段 2：每 2 秒印解析狀態 -----
  if (stage1Done && millis() - lastDbgTime > 2000) {
    lastDbgTime = millis();
    printStatus();
  }
}

void printStatus() {
  unsigned long secs = (millis() - startTime) / 1000;
  out("\n[%3lus] ─────────────────────────\n", secs);

  int used = gps.satellites.isValid() ? gps.satellites.value() : 0;
  int view = atoi(satsInView.value());

  out("衛星：用於定位 %2d  |  可見 %2d  [", used, view);
  for (int i = 0; i < 12; i++) {
    char c = (i < used ? '#' : (i < view ? '.' : ' '));
    Serial.print(c);
    SerialBT.print(c);
  }
  outln("]");

  out("  訊號強度: ");
  bool anyValid = false;
  for (int i = 0; i < 12; i++) {
    int prn = atoi(satPRN[i].value());
    int snr = atoi(satSNR[i].value());
    if (prn > 0) {
      if (snr > 0) out("[#%02d:%2ddB] ", prn, snr);
      else         out("[#%02d:--] ", prn);
      anyValid = true;
    }
  }
  if (!anyValid) out("（還沒收到 $GPGSV）");
  outln();

  out("  封包: 收=%lu 解析=%lu CRC錯=%lu  HDOP=%.1f\n",
      charsReceived,
      gps.sentencesWithFix(),
      gps.failedChecksum(),
      gps.hdop.isValid() ? gps.hdop.hdop() : 99.9);

  if (gps.location.isValid()) {
    out("  ✓ Lat:%9.5f Lon:%10.5f Alt:%5.1fm ",
        gps.location.lat(), gps.location.lng(),
        gps.altitude.isValid() ? gps.altitude.meters() : 0.0);
  } else {
    out("  ✗ 還沒定位 ");
  }

  if (gps.time.isValid()) {
    out("UTC:%02d:%02d:%02d",
        gps.time.hour(), gps.time.minute(), gps.time.second());
  }
  outln();

  // ----- 取得第一次 fix 時報告 -----
  static bool firstFix = false;
  if (!firstFix && gps.location.isValid()) {
    firstFix = true;
    outln();
    outln("✅✅✅ 取得 GPS Fix！晶片正常 ✅✅✅");
    out("   位置：%.6f, %.6f\n", gps.location.lat(), gps.location.lng());
    out("   高度：%.1f m\n", gps.altitude.meters());
    out("   花了 %lu 秒\n", secs);
    outln();
  }

  // ----- 5 分鐘後還沒 fix 警告 -----
  static bool warnedNoFix = false;
  if (!warnedNoFix && secs > 300 && !gps.location.isValid()) {
    warnedNoFix = true;
    outln();
    outln("⚠️  5 分鐘還沒 fix，可能：");
    outln("    1. 在室內 → 帶到戶外開闊處");
    outln("    2. 天線方向錯 → 模組上的陶瓷天線朝上");
    outln("    3. 模組壞或天線壞");
    outln();
  }
}
