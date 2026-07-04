// ============================================================
// TCA_Test — TCA9548A I²C 多工器 + VL53L0X 單獨測試
//
// 測試:
//   1. TCA9548A 本身有沒有在 I²C 主幹上(0x70)
//   2. CH0~CH7 各通道後面有沒有 VL53L0X(0x29)
//
// === 接線 ===
// TCA9548A 主機:
//   VCC ─→ 3V3
//   GND ─→ GND
//   SCL ─→ GPIO 22(主幹)
//   SDA ─→ GPIO 21(主幹)
//   A0/A1/A2 全 GND(位址 0x70)
//   RESET 不接
//
// VL53L0X(每顆):
//   VCC ─→ 3V3、GND ─→ GND
//   SDA ─→ TCA9548A SD0~SD3(看放哪通道)
//   SCL ─→ TCA9548A SC0~SC3
//
// === Serial 指令 ===
//   help    印指令
//   scan    主幹 I²C 掃描(找 TCA 0x70)
//   tof     讀 4 個通道距離(CH0~CH3)
//   chanX   切到通道 X(0~7),然後子 bus scan 看接什麼
// ============================================================

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define PIN_SDA       21
#define PIN_SCL       22
#define ADDR_TCA9548A 0x70

Adafruit_VL53L0X tofs[4];
bool             tofOK[4]   = { false, false, false, false };
const char*      TOF_NAMES[4] = { "前中", "前左", "前右", "底部" };
bool             tcaOK = false;

void tcaSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(ADDR_TCA9548A);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void scanMainBus() {
  Serial.println("[I²C Scan 主幹]");
  byte found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      const char* name = "?";
      if (addr == 0x70) name = "TCA9548A";
      Serial.printf("  0x%02X  %s\n", addr, name);
      found++;
    }
  }
  Serial.printf("[I²C Scan 主幹] %d 個元件\n", found);
}

void scanChannel(uint8_t ch) {
  if (!tcaOK) { Serial.println("[!] TCA9548A 沒在線上"); return; }
  tcaSelect(ch);
  delay(20);
  Serial.printf("[I²C Scan 通道 CH%d]\n", ch);
  byte found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    if (addr == ADDR_TCA9548A) continue;   // 跳過 TCA 本身
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      const char* name = "?";
      if (addr == 0x29) name = "VL53L0X";
      Serial.printf("  0x%02X  %s\n", addr, name);
      found++;
    }
  }
  Serial.printf("[I²C Scan 通道 CH%d] %d 個元件\n", ch, found);
}

void readAllToF() {
  if (!tcaOK) { Serial.println("[!] TCA9548A 沒在線上"); return; }
  for (int ch = 0; ch < 4; ch++) {
    if (!tofOK[ch]) {
      Serial.printf("[ToF] CH%d (%s): 沒 init OK\n", ch, TOF_NAMES[ch]);
      continue;
    }
    tcaSelect(ch);
    VL53L0X_RangingMeasurementData_t m;
    tofs[ch].rangingTest(&m, false);
    int d = (m.RangeStatus != 4) ? m.RangeMilliMeter : -1;
    Serial.printf("[ToF] CH%d (%s): %d mm\n", ch, TOF_NAMES[ch], d);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("  TCA9548A + VL53L0X 單獨測試");
  Serial.println("========================================");
  Serial.println("接線:");
  Serial.println("  TCA9548A:VCC→3V3、GND→GND、SCL→22、SDA→21、A0/1/2→GND");
  Serial.println("  VL53L0X 各顆:VCC→3V3、GND→GND、SCL/SDA→TCA SC0/SD0~SC3/SD3");
  Serial.println("");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Serial.println("[+] I²C ready (100 kHz)");

  scanMainBus();

  // 確認 TCA9548A 在
  Wire.beginTransmission(ADDR_TCA9548A);
  if (Wire.endTransmission() == 0) {
    tcaOK = true;
    Serial.println("[+] TCA9548A OK @ 0x70");
  } else {
    Serial.println("[!] TCA9548A NOT FOUND");
    Serial.println("    1. 量 VCC 對 GND 是 3.3V");
    Serial.println("    2. A0/A1/A2 全部接 GND(位址 0x70)");
    Serial.println("    3. SDA/SCL 通到 GPIO 21/22");
    Serial.println("    (沒 TCA 後面 VL53L0X init 沒意義,先解決這個)");
  }

  // 各通道初始化 VL53L0X
  if (tcaOK) {
    for (int ch = 0; ch < 4; ch++) {
      tcaSelect(ch);
      delay(50);
      if (tofs[ch].begin(0x29)) {
        tofOK[ch] = true;
        Serial.printf("[+] VL53L0X CH%d (%s): OK\n", ch, TOF_NAMES[ch]);
      } else {
        Serial.printf("[!] VL53L0X CH%d (%s): FAIL\n", ch, TOF_NAMES[ch]);
      }
    }
  }

  Serial.println("\n指令:help、scan(主幹)、tof(讀 4 顆)、chanX(0~7)");
  Serial.println("例如送『chan0』看 CH0 上接什麼");
  Serial.println("");
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if      (s == "help") {
      Serial.println("help / scan / tof / chan0~7");
    } else if (s == "scan") scanMainBus();
    else if (s == "tof") readAllToF();
    else if (s.startsWith("chan")) {
      int ch = s.substring(4).toInt();
      scanChannel(ch);
    }
    else if (s.length() > 0) Serial.printf("[?] 不認得「%s」\n", s.c_str());
  }

  // 每 2 秒自動印一次 ToF
  static unsigned long lastTick = 0;
  if (millis() - lastTick > 2000) {
    lastTick = millis();
    if (tcaOK) readAllToF();
  }
}
