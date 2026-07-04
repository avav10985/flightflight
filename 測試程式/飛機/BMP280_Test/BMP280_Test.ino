// ============================================================
// BMP280 獨立測試程式
// MCU: LOLIN D32 (ESP32)
//
// 用途：驗證 BMP280 接線正確、晶片沒壞
//
// 接線：
//   BMP280       LOLIN D32
//   ─────────    ─────────
//   VCC ────────→ 3V (3.3V)
//   GND ────────→ GND
//   SDA ────────→ GPIO 21
//   SCL ────────→ GPIO 22
//
// 預期輸出（每 500ms 一次）：
//   T: 26.50°C  P: 1013.25 hPa  Alt: 12.34 m
// ============================================================

#include <Wire.h>
#include <Adafruit_BMP280.h>

#define PIN_SDA  21
#define PIN_SCL  22

Adafruit_BMP280 bmp;
bool found = false;
uint8_t foundAddr = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("  BMP280 測試程式");
  Serial.println("========================================");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Serial.println("[1] I2C 啟動 (SDA=21, SCL=22)");

  // ----- 掃 I2C bus 看模組在不在 -----
  Serial.println("[2] 掃描 I2C bus...");
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("    發現裝置：0x%02X\n", addr);
      count++;
    }
  }
  if (count == 0) {
    Serial.println("    ❌ 沒掃到任何 I2C 裝置 → 接線錯誤或模組壞");
    Serial.println("       檢查：VCC 通電？SDA/SCL 接對？GND 共地？");
    while (1) delay(1000);
  }
  Serial.printf("    共 %d 個 I2C 裝置\n", count);

  // ----- 嘗試 0x76 -----
  Serial.println("[3] 嘗試 BMP280 地址 0x76...");
  if (bmp.begin(0x76)) {
    found = true;
    foundAddr = 0x76;
    Serial.println("    ✅ BMP280 在 0x76");
  } else {
    Serial.println("    ❌ 0x76 沒回應，試 0x77");
    if (bmp.begin(0x77)) {
      found = true;
      foundAddr = 0x77;
      Serial.println("    ✅ BMP280 在 0x77");
    } else {
      Serial.println("    ❌ 0x77 也沒回應");
    }
  }

  if (!found) {
    Serial.println();
    Serial.println("⚠️  可能原因：");
    Serial.println("    1. 模組是 BME280（多了濕度），地址一樣但晶片不同 → 換 Adafruit_BME280 函式庫");
    Serial.println("    2. 接線 SDA/SCL 對調");
    Serial.println("    3. 模組沒供電（量 VCC ↔ GND 應 3.3V）");
    Serial.println("    4. 模組壞了");
    while (1) delay(1000);
  }

  // ----- 設定取樣模式 -----
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,    // 溫度
    Adafruit_BMP280::SAMPLING_X16,   // 氣壓
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500
  );
  Serial.println("[4] 取樣設定 OK");
  Serial.println();
  Serial.println("========================================");
  Serial.println("  開始讀值（每 500ms）");
  Serial.println("========================================");
}

void loop() {
  if (!found) {
    delay(1000);
    return;
  }

  float t   = bmp.readTemperature();         // °C
  float p   = bmp.readPressure() / 100.0f;   // hPa
  float alt = bmp.readAltitude(1013.25);     // m（用標準海平面氣壓）

  Serial.printf("T:%6.2f°C  P:%7.2f hPa  Alt:%7.2f m\n", t, p, alt);

  // ----- 合理性檢查 -----
  static bool warned = false;
  if (!warned) {
    if (t < -10 || t > 60) {
      Serial.println("⚠️  溫度異常（合理範圍 -10~60°C），晶片可能有問題");
      warned = true;
    } else if (p < 800 || p > 1100) {
      Serial.println("⚠️  氣壓異常（合理範圍 800~1100 hPa），晶片可能有問題");
      warned = true;
    } else {
      // 第一筆正常 → 報好
      Serial.println("✅ 讀值在合理範圍，晶片運作正常");
      warned = true;   // 只報一次
    }
  }

  delay(500);
}
