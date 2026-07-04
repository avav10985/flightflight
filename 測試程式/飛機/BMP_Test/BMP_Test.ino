// ============================================================
// BMP_Test — BMP280 氣壓計 單獨測試(飛機 ESP32 端)
//
// 只接 BMP280,別接其他 I²C 元件,確認這顆獨立能用。
//
// === 接線 ===
//   VCC ─→ 3V3
//   GND ─→ GND
//   SCL ─→ GPIO 22
//   SDA ─→ GPIO 21
//   CSB ─→ 3V3(6 腳版必接,拉高才進 I²C 模式;4 腳版沒這隻)
//   SDO ─→ GND(6 腳版,接 GND → 位址 0x76;接 VCC → 0x77)
//
// === Serial 指令(115200 baud)===
//   help   印指令
//   scan   I²C 掃描(看有沒有 0x76 或 0x77)
//   read   讀一次氣壓 + 溫度 + 高度
//
// 每秒自動印一次讀值。
// ============================================================

#include <Wire.h>
#include <Adafruit_BMP280.h>

#define PIN_SDA  21
#define PIN_SCL  22

Adafruit_BMP280 bmp;
bool   bmp280OK = false;
float  baseAltitude = 0;
uint8_t addrFound = 0;

void scanI2C() {
  Serial.println("[I²C Scan] 掃描 0x01~0x7F...");
  byte found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      const char* name = "?";
      if (addr == 0x76) name = "BMP280(SDO=GND)";
      else if (addr == 0x77) name = "BMP280(SDO=VCC)";
      Serial.printf("  0x%02X  %s\n", addr, name);
      found++;
    }
  }
  Serial.printf("[I²C Scan] 找到 %d 個元件\n", found);
}

void readOnce() {
  if (!bmp280OK) {
    Serial.println("[!] BMP280 沒 init OK,先確認接線");
    return;
  }
  float pres = bmp.readPressure() / 100.0f;
  float temp = bmp.readTemperature();
  float alt  = bmp.readAltitude(1013.25f);
  float rel  = alt - baseAltitude;
  Serial.printf("[BMP] 氣壓=%.1f hPa  溫度=%.1f°C  絕對高度=%.1fm  相對=%+.1fm\n",
                pres, temp, alt, rel);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("       BMP280 單獨測試");
  Serial.println("========================================");
  Serial.println("接線:VCC→3V3、GND→GND、SCL→GPIO 22、SDA→GPIO 21");
  Serial.println("6 腳版額外:CSB→3V3、SDO→GND(0x76)或 VCC(0x77)");
  Serial.println("");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Serial.println("[+] I²C ready (100 kHz)");

  // 先 I²C scan 看有沒有 BMP280 在線上
  scanI2C();
  Serial.println("");

  // 試 0x76
  if (bmp.begin(0x76)) {
    addrFound = 0x76;
    bmp280OK = true;
    Serial.println("[+] BMP280 OK @ 0x76(SDO=GND)");
  } else if (bmp.begin(0x77)) {
    addrFound = 0x77;
    bmp280OK = true;
    Serial.println("[+] BMP280 OK @ 0x77(SDO=VCC)");
  } else {
    Serial.println("[!] BMP280 init 失敗");
    Serial.println("    1. 確認 VCC 量到 3.3V");
    Serial.println("    2. 6 腳版確認 CSB 接 3V3(拉低變 SPI 模式)");
    Serial.println("    3. SDA/SCL 確認接到 GPIO 21/22");
    Serial.println("    4. scan 結果看 0x76 或 0x77 有沒有在線上");
  }

  if (bmp280OK) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    delay(100);
    baseAltitude = bmp.readAltitude(1013.25f);
    Serial.printf("[+] baseAlt(開機時的高度)= %.2fm\n", baseAltitude);
  }

  Serial.println("\n指令:help、scan、read");
  Serial.println("(每秒自動印一次氣壓)");
}

void loop() {
  // 處理 Serial 指令
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if      (s == "help") {
      Serial.println("指令:help / scan / read");
    } else if (s == "scan") scanI2C();
    else if (s == "read")   readOnce();
    else if (s.length() > 0) Serial.printf("[?] 不認得「%s」\n", s.c_str());
  }

  // 每秒自動印一次
  static unsigned long lastTick = 0;
  if (millis() - lastTick > 1000) {
    lastTick = millis();
    if (bmp280OK) readOnce();
  }
}
