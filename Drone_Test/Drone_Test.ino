// ============================================================
// Drone_Test — 飛機端元件獨立測試(不需要連手把)
//
// 用途:
//   - 試燒燒飛機就跑,Serial 顯示所有感測器讀值
//   - 不啟動 NRF24(排除手把通訊變數)
//   - 可透過 Serial 手動控制 4 顆 ESC PWM(測馬達轉向用)
//   - ⚠️ **絕對不要插螺旋槳!** 馬達裸轉測試
//
// === Serial 指令(115200 baud)===
//   help              ← 印指令清單
//   imu               ← 印 MPU6050 姿態 + gyro
//   gps               ← 印 GPS 狀態
//   tof               ← 印 4 顆雷射距離
//   mag               ← 印磁力計 heading
//   bmp               ← 印氣壓 / 高度
//   bat               ← 印電池電壓
//   all               ← 一次印全部
//   m 1100            ← 4 顆馬達同時設 PWM 1100(微轉怠速)
//   m1 1100           ← 只 M1 設 PWM 1100
//   m2/m3/m4          ← 同上
//   stop              ← 全部馬達停(PWM 1000)
//
// PWM 範圍 1000(停)~ 2000(全速)。建議測試從 1100 開始,逐步加。
// ============================================================

#define ENABLE_BMP280    1
#define ENABLE_VL53L0X   1
#define ENABLE_HMC5883   1

#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <TinyGPSPlus.h>

#if ENABLE_BMP280
#include <Adafruit_BMP280.h>
#endif
#if ENABLE_VL53L0X
#include <Adafruit_VL53L0X.h>
#endif

// === 腳位(跟 Drone_FC_Full 一致)===
#define PIN_ESC1     25
#define PIN_ESC2     26
#define PIN_ESC3     27
#define PIN_ESC4     14
#define PIN_SDA      21
#define PIN_SCL      22
#define PIN_BAT_ADC  34
#define PIN_GPS_RX   33
#define PIN_GPS_TX   32

#if ENABLE_BMP280
#define ADDR_BMP280  0x76
#endif
#if ENABLE_VL53L0X
#define ADDR_TCA9548A 0x70
#endif
#if ENABLE_HMC5883
#define ADDR_HMC5883 0x1E
#define ADDR_QMC5883 0x0D
#endif

const float BAT_DIVIDER = 4.0f;

// === 全域 ===
MPU6050      mpu;
Servo        esc1, esc2, esc3, esc4;
TinyGPSPlus  gps;

#if ENABLE_BMP280
Adafruit_BMP280 bmp;
bool   bmp280OK     = false;
float  baseAltitude = 0;
#endif

#if ENABLE_VL53L0X
Adafruit_VL53L0X tofs[4];
bool     tofOK[4]   = { false, false, false, false };
uint16_t tofDist[4] = { 0, 0, 0, 0 };
const char* TOF_NAMES[4] = { "前中", "前左", "前右", "底部" };
#endif

#if ENABLE_HMC5883
bool    magOK     = false;
uint8_t magAddr   = 0;
bool    magIsQMC  = false;
float   magHeadingDeg = 0;
#endif

// === Helper:I²C 多工器通道切換 ===
#if ENABLE_VL53L0X
void tcaSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(ADDR_TCA9548A);
  Wire.write(1 << ch);
  Wire.endTransmission();
}
#endif

// === 磁力計初始化跟讀取(跟 Drone_FC_Full 一樣)===
#if ENABLE_HMC5883
bool magDetect() {
  Wire.beginTransmission(ADDR_HMC5883);
  if (Wire.endTransmission() == 0) {
    magAddr = ADDR_HMC5883; magIsQMC = false;
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
  float heading = atan2f(-(float)my, (float)mx) * 180.0f / M_PI;
  if (heading < 0) heading += 360.0f;
  magHeadingDeg = heading;
  return true;
}
#endif

// === Serial 指令處理 ===
void setMotorAll(int pwm) {
  pwm = constrain(pwm, 1000, 2000);
  esc1.writeMicroseconds(pwm);
  esc2.writeMicroseconds(pwm);
  esc3.writeMicroseconds(pwm);
  esc4.writeMicroseconds(pwm);
  Serial.printf("[!] 4 顆馬達都設 PWM = %d\n", pwm);
}

void setMotor(int n, int pwm) {
  pwm = constrain(pwm, 1000, 2000);
  switch (n) {
    case 1: esc1.writeMicroseconds(pwm); break;
    case 2: esc2.writeMicroseconds(pwm); break;
    case 3: esc3.writeMicroseconds(pwm); break;
    case 4: esc4.writeMicroseconds(pwm); break;
    default: Serial.println("[!] 馬達 1~4"); return;
  }
  Serial.printf("[!] M%d 設 PWM = %d\n", n, pwm);
}

void printIMU() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float roll  = atan2f(ax, az) * 57.2958f;
  float pitch = atan2f(-ay, sqrtf((float)ax*ax + (float)az*az)) * 57.2958f;
  Serial.printf("[IMU] Roll=%+6.1f° Pitch=%+6.1f° | gyro X=%+5d Y=%+5d Z=%+5d\n",
                roll, pitch, gx, gy, gz);
}

void printGPS() {
  Serial.printf("[GPS] sat=%lu fix=%d lat=%.6f lon=%.6f alt=%.1fm\n",
                gps.satellites.value(),
                gps.location.isValid(),
                gps.location.lat(),
                gps.location.lng(),
                gps.altitude.meters());
}

void printToF() {
#if ENABLE_VL53L0X
  for (int ch = 0; ch < 4; ch++) {
    if (!tofOK[ch]) { Serial.printf("[ToF] %s: 沒接\n", TOF_NAMES[ch]); continue; }
    tcaSelect(ch);
    VL53L0X_RangingMeasurementData_t m;
    tofs[ch].rangingTest(&m, false);
    int d = (m.RangeStatus != 4) ? m.RangeMilliMeter : -1;
    Serial.printf("[ToF] %s: %dmm\n", TOF_NAMES[ch], d);
  }
#else
  Serial.println("[ToF] ENABLE_VL53L0X = 0");
#endif
}

void printMag() {
#if ENABLE_HMC5883
  if (!magOK) { Serial.println("[Mag] 未偵測到"); return; }
  magRead();
  Serial.printf("[Mag] heading = %.1f° (0=北 90=東 180=南 270=西)\n", magHeadingDeg);
#else
  Serial.println("[Mag] ENABLE_HMC5883 = 0");
#endif
}

void printBMP() {
#if ENABLE_BMP280
  if (!bmp280OK) { Serial.println("[BMP] 未偵測到"); return; }
  float pres = bmp.readPressure() / 100.0f;
  float alt  = bmp.readAltitude(1013.25f);
  float rel  = alt - baseAltitude;
  Serial.printf("[BMP] pres=%.1f hPa  abs=%.1fm  rel=%+.1fm(home 為 0)\n", pres, alt, rel);
#else
  Serial.println("[BMP] ENABLE_BMP280 = 0");
#endif
}

void printBat() {
  int raw = analogRead(PIN_BAT_ADC);
  float v = raw * (3.3f / 4095.0f) * BAT_DIVIDER;
  Serial.printf("[Bat] ADC=%d → %.2fV(× %.1f)\n", raw, v, BAT_DIVIDER);
}

void printAll() {
  printIMU();
  printGPS();
  printBMP();
  printMag();
  printToF();
  printBat();
  Serial.println("---");
}

void scanI2C() {
  Serial.println("[I²C Scan] 掃描 0x01~0x7F...");
  byte found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      const char* name = "?";
      if (addr == 0x68) name = "MPU6050";
      else if (addr == 0x69) name = "MPU6050(AD0=VCC)";
      else if (addr == 0x76) name = "BMP280";
      else if (addr == 0x77) name = "BMP280(SDO=VCC)";
      else if (addr == 0x1E) name = "HMC5883L 磁力計";
      else if (addr == 0x0D) name = "QMC5883 磁力計";
      else if (addr == 0x70) name = "TCA9548A 多工器";
      else if (addr == 0x29) name = "VL53L0X(可能在 TCA 通道後)";
      Serial.printf("  0x%02X  ← %s\n", addr, name);
      found++;
    }
  }
  Serial.printf("[I²C Scan] 找到 %d 個元件\n", found);
  if (found == 0) {
    Serial.println("       ⚠️ I²C 主幹完全沒回應 → SDA/SCL 沒接好 或 bus 短路");
  } else if (found == 1) {
    Serial.println("       ⚠️ 只 1 個元件回應 → 其他元件電源/接線/I²C 拉線可能有問題");
  }
}

void printHelp() {
  Serial.println("\n=== 指令 ===");
  Serial.println("  help       此說明");
  Serial.println("  imu        姿態 + gyro");
  Serial.println("  gps        GPS");
  Serial.println("  tof        4 顆雷射");
  Serial.println("  mag        磁力計");
  Serial.println("  bmp        氣壓 / 高度");
  Serial.println("  bat        電池電壓");
  Serial.println("  scan       I²C 掃描(找有哪些位址在線上)");
  Serial.println("  all        一次印全部");
  Serial.println("  m <pwm>    4 顆馬達同時設 PWM(1000~2000)");
  Serial.println("  m1 <pwm>   只 M1(M2/M3/M4 同理)");
  Serial.println("  stop       全部馬達停(PWM 1000)");
  Serial.println("⚠️ 馬達測試別插螺旋槳!");
  Serial.println("");
}

void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;

  if      (s == "help") printHelp();
  else if (s == "imu")  printIMU();
  else if (s == "gps")  printGPS();
  else if (s == "tof")  printToF();
  else if (s == "mag")  printMag();
  else if (s == "bmp")  printBMP();
  else if (s == "bat")  printBat();
  else if (s == "all")  printAll();
  else if (s == "scan") scanI2C();
  else if (s == "stop") setMotorAll(1000);
  else if (s.startsWith("m ")) {
    setMotorAll(s.substring(2).toInt());
  }
  else if (s.startsWith("m1 ")) setMotor(1, s.substring(3).toInt());
  else if (s.startsWith("m2 ")) setMotor(2, s.substring(3).toInt());
  else if (s.startsWith("m3 ")) setMotor(3, s.substring(3).toInt());
  else if (s.startsWith("m4 ")) setMotor(4, s.substring(3).toInt());
  else {
    Serial.printf("[?] 不認得「%s」,送 help 看清單\n", s.c_str());
  }
}

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("  飛機端元件獨立測試(不需要手把)");
  Serial.println("========================================");

  // ESC init(預設停)
  esc1.attach(PIN_ESC1, 1000, 2000);
  esc2.attach(PIN_ESC2, 1000, 2000);
  esc3.attach(PIN_ESC3, 1000, 2000);
  esc4.attach(PIN_ESC4, 1000, 2000);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  Serial.println("[+] ESC × 4 init,PWM 1000(停)");

  // I²C
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Serial.println("[+] I2C ready (100 kHz)");

  // MPU6050
  mpu.initialize();
  Serial.printf("[%s] MPU6050: %s\n", mpu.testConnection() ? "+" : "!",
                mpu.testConnection() ? "OK (0x68)" : "FAIL");

  // BMP280
#if ENABLE_BMP280
  if (bmp.begin(ADDR_BMP280)) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    delay(100);
    baseAltitude = bmp.readAltitude(1013.25f);
    bmp280OK = true;
    Serial.printf("[+] BMP280: OK (0x76),baseAlt=%.2fm\n", baseAltitude);
  } else {
    Serial.println("[!] BMP280: FAIL(檢查 SDO=GND、CSB=3V3、I²C 接線)");
  }
#endif

  // VL53L0X × 4
#if ENABLE_VL53L0X
  for (int ch = 0; ch < 4; ch++) {
    tcaSelect(ch);
    delay(50);
    if (tofs[ch].begin(0x29)) {
      tofOK[ch] = true;
      Serial.printf("[+] VL53L0X #%d (%s): OK\n", ch, TOF_NAMES[ch]);
    } else {
      Serial.printf("[!] VL53L0X #%d (%s): FAIL\n", ch, TOF_NAMES[ch]);
    }
  }
#endif

  // 磁力計
#if ENABLE_HMC5883
  if (magDetect()) {
    magOK = true;
    Serial.printf("[+] 磁力計 OK,位址 0x%02X (%s)\n", magAddr, magIsQMC ? "QMC5883" : "HMC5883L");
  } else {
    Serial.println("[!] 磁力計沒偵測到(檢查 VCC/GND/SDA/SCL)");
  }
#endif

  // GPS UART2
  Serial2.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.println("[+] GPS UART2 @ 9600 啟動(NEO-6M 冷啟動 30~120 秒)");

  // 電池 ADC
  analogReadResolution(12);
  Serial.println("[+] 電池 ADC ready");

  printHelp();
  Serial.println("[+] 就緒,送指令開始測試");
}

// ============================================================
void loop() {
  // 處理 Serial 指令
  handleSerial();

  // 餵 GPS NMEA 字串
  while (Serial2.available()) gps.encode(Serial2.read());

  // 每秒自動印一次 IMU + 電池 + GPS 衛星數(讓你看到飛機活著)
  static unsigned long lastTick = 0;
  if (millis() - lastTick > 1000) {
    lastTick = millis();
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float roll  = atan2f(ax, az) * 57.2958f;
    float pitch = atan2f(-ay, sqrtf((float)ax*ax + (float)az*az)) * 57.2958f;
    int batRaw = analogRead(PIN_BAT_ADC);
    float batV = batRaw * (3.3f / 4095.0f) * BAT_DIVIDER;
    Serial.printf("R%+5.1f° P%+5.1f° | Bat %.2fV | GPS sat%lu\n",
                  roll, pitch, batV, gps.satellites.value());
  }
}
