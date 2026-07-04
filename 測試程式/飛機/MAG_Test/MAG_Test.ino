// ============================================================
// MAG_Test — GY-271 磁力計 單獨測試
//
// 自動偵測 HMC5883L(0x1E)或 QMC5883(0x0D,中國複製版)。
//
// === 接線 ===
//   VCC  ─→ 3V3
//   GND  ─→ GND
//   SCL  ─→ GPIO 22
//   SDA  ─→ GPIO 21
//   DRDY ─→ 不接
//
// === Serial 指令 ===
//   help    印指令
//   scan    I²C 掃描
//   read    印一次磁力 X/Y/Z + heading
//   magcal  10 秒繞圈校準(把模組平放繞 360° 轉 2 圈)
//
// 每秒自動印一次。
// ============================================================

#include <Wire.h>
#include <math.h>

#define PIN_SDA       21
#define PIN_SCL       22

#define ADDR_HMC5883  0x1E
#define ADDR_QMC5883  0x0D

bool    magOK = false;
uint8_t magAddr = 0;
bool    magIsQMC = false;
float   magOffsetX = 0, magOffsetY = 0;
float   magHeadingDeg = 0;
int16_t magRawX = 0, magRawY = 0, magRawZ = 0;

void scanI2C() {
  Serial.println("[I²C Scan] 掃描 0x01~0x7F...");
  byte found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      const char* name = "?";
      if (addr == 0x1E) name = "HMC5883L";
      else if (addr == 0x0D) name = "QMC5883";
      Serial.printf("  0x%02X  %s\n", addr, name);
      found++;
    }
  }
  Serial.printf("[I²C Scan] 找到 %d 個\n", found);
}

bool magDetect() {
  Wire.beginTransmission(ADDR_HMC5883);
  if (Wire.endTransmission() == 0) {
    magAddr = ADDR_HMC5883; magIsQMC = false;
    // HMC5883L 初始化:Config A=0x70(8 avg/15Hz)、B=0x20(±1.3G)、Mode=0x00(continuous)
    Wire.beginTransmission(magAddr);
    Wire.write(0x00); Wire.write(0x70); Wire.write(0x20); Wire.write(0x00);
    Wire.endTransmission();
    return true;
  }
  Wire.beginTransmission(ADDR_QMC5883);
  if (Wire.endTransmission() == 0) {
    magAddr = ADDR_QMC5883; magIsQMC = true;
    Wire.beginTransmission(magAddr); Wire.write(0x0B); Wire.write(0x01); Wire.endTransmission();
    Wire.beginTransmission(magAddr); Wire.write(0x09); Wire.write(0x1D); Wire.endTransmission();
    return true;
  }
  return false;
}

bool magRead() {
  if (!magOK) return false;
  int16_t mx, my, mz;
  if (magIsQMC) {
    Wire.beginTransmission(magAddr); Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(magAddr, (uint8_t)6);
    if (Wire.available() < 6) return false;
    mx = Wire.read() | (Wire.read() << 8);
    my = Wire.read() | (Wire.read() << 8);
    mz = Wire.read() | (Wire.read() << 8);
  } else {
    Wire.beginTransmission(magAddr); Wire.write(0x03);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(magAddr, (uint8_t)6);
    if (Wire.available() < 6) return false;
    mx = (Wire.read() << 8) | Wire.read();
    mz = (Wire.read() << 8) | Wire.read();
    my = (Wire.read() << 8) | Wire.read();
  }
  magRawX = mx; magRawY = my; magRawZ = mz;
  float fx = (float)mx - magOffsetX;
  float fy = (float)my - magOffsetY;
  float heading = atan2f(-fy, fx) * 180.0f / M_PI;
  if (heading < 0) heading += 360.0f;
  magHeadingDeg = heading;
  return true;
}

void readOnce() {
  if (!magRead()) {
    Serial.println("[!] 讀失敗");
    return;
  }
  const char* dir = "?";
  float h = magHeadingDeg;
  if      (h >= 337.5 || h < 22.5)  dir = "北";
  else if (h <  67.5)               dir = "東北";
  else if (h < 112.5)               dir = "東";
  else if (h < 157.5)               dir = "東南";
  else if (h < 202.5)               dir = "南";
  else if (h < 247.5)               dir = "西南";
  else if (h < 292.5)               dir = "西";
  else                              dir = "西北";
  Serial.printf("[MAG] X=%+5d Y=%+5d Z=%+5d | heading=%.1f° (%s)\n",
                magRawX, magRawY, magRawZ, magHeadingDeg, dir);
}

void magcal() {
  if (!magOK) { Serial.println("[!] 沒偵測到磁力計"); return; }
  Serial.println(">> 磁力計校準 10 秒,把飛機平放繞 360° 轉 2 圈...");
  float xMin = 32767, xMax = -32768;
  float yMin = 32767, yMax = -32768;
  unsigned long endMs = millis() + 10000;
  int samples = 0;
  while (millis() < endMs) {
    if (magRead()) {
      if (magRawX < xMin) xMin = magRawX; if (magRawX > xMax) xMax = magRawX;
      if (magRawY < yMin) yMin = magRawY; if (magRawY > yMax) yMax = magRawY;
      samples++;
    }
    delay(50);
  }
  magOffsetX = (xMin + xMax) / 2.0f;
  magOffsetY = (yMin + yMax) / 2.0f;
  Serial.printf(">> 校準完成,%d samples,offset X=%.0f Y=%.0f\n", samples, magOffsetX, magOffsetY);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("    GY-271 磁力計 單獨測試");
  Serial.println("========================================");
  Serial.println("接線:VCC→3V3、GND→GND、SCL→GPIO 22、SDA→GPIO 21");
  Serial.println("");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Serial.println("[+] I²C ready (100 kHz)");

  scanI2C();
  Serial.println("");

  if (magDetect()) {
    magOK = true;
    Serial.printf("[+] 磁力計 OK,位址 0x%02X (%s)\n", magAddr, magIsQMC ? "QMC5883" : "HMC5883L");
    Serial.println("[+] 沒校準前讀值會有偏差,做 magcal 校準後 heading 才準");
  } else {
    Serial.println("[!] 磁力計沒偵測到");
    Serial.println("    1. 量 VCC 對 GND 是 3.3V");
    Serial.println("    2. SDA / SCL 通到 GPIO 21 / 22");
    Serial.println("    3. scan 結果看有沒有 0x1E 或 0x0D");
  }

  Serial.println("\n指令:help、scan、read、magcal\n");
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if      (s == "help") Serial.println("help / scan / read / magcal");
    else if (s == "scan") scanI2C();
    else if (s == "read") readOnce();
    else if (s == "magcal") magcal();
    else if (s.length() > 0) Serial.printf("[?] 不認得「%s」\n", s.c_str());
  }

  static unsigned long lastTick = 0;
  if (millis() - lastTick > 1000) {
    lastTick = millis();
    if (magOK) readOnce();
  }
}
