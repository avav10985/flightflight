// ============================================================
// 階段 4：完整版（PID + BMP280 + GPS + VL53L0X 避障）
// MCU: LOLIN D32 (ESP32-WROOM-32)
//
// 在 Drone_FC_PID 基礎上加入三組感測器（用 ENABLE 旗標逐個啟用）：
//   - BMP280  氣壓計（之後做定高）
//   - NEO-6M  GPS（之後做自動飛行）
//   - VL53L0X 雷射 × 2（之後做避障）
//
// 目前只讀資料、印序列埠，**還沒整合進 PID**。
//
// 序列埠指令（與 PID 階段相同）：
//   kp / ki / kd / ypp / yi <值>
//   p / cal
// ============================================================

// ====== 階段開關（按硬體進度依序打開）======
#define ENABLE_NRF24    1
#define ENABLE_ESC      1
#define ENABLE_PID      1    // PID 自穩
#define ENABLE_BMP280   0    // 接好 BMP280 改 1
#define ENABLE_GPS      0    // 接好 GPS 改 1
#define ENABLE_VL53L0X  0    // 接好 TCA9548A + VL53L0X × 2 改 1

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

#if ENABLE_BMP280
#include <Adafruit_BMP280.h>
#endif

#if ENABLE_GPS
#include <TinyGPSPlus.h>
#endif

#if ENABLE_VL53L0X
#include <Adafruit_VL53L0X.h>
#endif

// ---- 腳位 ----
#define PIN_NRF_CE   4
#define PIN_NRF_CSN  5
#define PIN_ESC1     25
#define PIN_ESC2     26
#define PIN_ESC3     27
#define PIN_ESC4     14
#define PIN_SDA      21
#define PIN_SCL      22
#define PIN_GPS_RX   16   // 接 GPS 的 TX
#define PIN_GPS_TX   17   // 接 GPS 的 RX

// ---- I²C 地址 ----
#define ADDR_BMP280      0x76
#define ADDR_TCA9548A    0x70

// ---- NRF24 ----
const uint64_t PIPE_IN     = 0xABCDABCD71LL;
const uint8_t  NRF_CHANNEL = 100;

struct Signal {
  byte throttle, pitch, roll, yaw, aux1, aux2;
};

RF24    radio(PIN_NRF_CE, PIN_NRF_CSN);
MPU6050 mpu;
Servo   esc1, esc2, esc3, esc4;

#if ENABLE_BMP280
Adafruit_BMP280 bmp;
bool bmp280OK = false;
float baseAltitude = 0;   // 開機時當作高度 0 的基準
#endif

#if ENABLE_GPS
TinyGPSPlus gps;
#endif

#if ENABLE_VL53L0X
Adafruit_VL53L0X tof0;     // CH0 前方
Adafruit_VL53L0X tof1;     // CH1 後方
bool tof0OK = false, tof1OK = false;
#endif

// ---- 全域狀態 ----
Signal        data;
bool          armed       = false;
unsigned long lastRxTime  = 0;
unsigned long lastDbgTime = 0;
float         roll = 0, pitch = 0, yawRate = 0;
int16_t       rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
float         gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float         levelOffsetRoll = 0, levelOffsetPitch = 0;

// 感測器讀值
float  altitude_m   = 0;
double gps_lat = 0, gps_lon = 0;
uint8_t gps_sat = 0;
uint16_t distFront_mm = 0, distBack_mm = 0;

// ---- PID 增益 ----
float Kp_rp = 2.0f, Ki_rp = 0.02f, Kd_rp = 0.5f;
float Kp_y  = 1.5f, Ki_y  = 0.02f;

const float I_LIMIT     = 100.0f;
const float MAX_ANGLE   = 30.0f;
const float MAX_YAWRATE = 90.0f;

float i_roll = 0, i_pitch = 0, i_yaw = 0;
float rollOut = 0, pitchOut = 0, yawOut = 0;

// ============================================================
// TCA9548A 通道切換
// ============================================================
#if ENABLE_VL53L0X
void tcaSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(ADDR_TCA9548A);
  Wire.write(1 << ch);
  Wire.endTransmission();
}
#endif

// ============================================================
// 校正陀螺儀 + 水平基準
// ============================================================
void calibrateGyro() {
  Serial.println("[*] 校正陀螺儀 + 水平基準（3 秒），請把飛機平放...");
  const int N = 300;
  long sx = 0, sy = 0, sz = 0;
  float sumRoll = 0, sumPitch = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < N; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sx += gx; sy += gy; sz += gz;
    sumRoll  += atan2f(ax, az) * 57.2958f;
    sumPitch += atan2f(-ay, sqrtf((float)ax*ax + (float)az*az)) * 57.2958f;
    delay(5);
  }
  gyroOffsetX = sx / (float)N;
  gyroOffsetY = sy / (float)N;
  gyroOffsetZ = sz / (float)N;
  levelOffsetRoll  = sumRoll  / N;
  levelOffsetPitch = sumPitch / N;
  roll = pitch = yawRate = 0;
  i_roll = i_pitch = i_yaw = 0;

#if ENABLE_BMP280
  if (bmp280OK) {
    baseAltitude = bmp.readAltitude(1013.25);
    Serial.printf("[*] 高度基準: %.2f m\n", baseAltitude);
  }
#endif

  Serial.printf("[*] 陀螺儀偏移: gx=%.1f gy=%.1f gz=%.1f\n",
                gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  Serial.printf("[*] 水平基準: R=%.2f° P=%.2f°（已扣除）\n",
                levelOffsetRoll, levelOffsetPitch);
}

void resetData() {
  data.throttle = 0;
  data.pitch = data.roll = data.yaw = 127;
  data.aux1 = data.aux2 = 0;
}

void readAttitude() {
  mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

  float gxCal = rawGx - gyroOffsetX;
  float gyCal = rawGy - gyroOffsetY;
  float gzCal = rawGz - gyroOffsetZ;

  static unsigned long lastT = 0;
  unsigned long now = micros();
  float dt = (now - lastT) / 1000000.0f;
  lastT = now;
  if (dt > 0.05f || dt <= 0) dt = 0.01f;

  float accRoll  = atan2f(rawAx, rawAz) * 57.2958f - levelOffsetRoll;
  float accPitch = atan2f(-rawAy, sqrtf((float)rawAx*rawAx + (float)rawAz*rawAz)) * 57.2958f - levelOffsetPitch;

  float gRoll  = -gyCal / 131.0f;
  float gPitch = -gxCal / 131.0f;
  yawRate      =  gzCal / 131.0f;

  roll  = 0.98f * (roll  + gRoll  * dt) + 0.02f * accRoll;
  pitch = 0.98f * (pitch + gPitch * dt) + 0.02f * accPitch;
}

// ============================================================
// 感測器讀取
// ============================================================
void readBMP280() {
#if ENABLE_BMP280
  if (!bmp280OK) return;
  float h = bmp.readAltitude(1013.25);
  altitude_m = h - baseAltitude;   // 相對高度
#endif
}

void readGPS() {
#if ENABLE_GPS
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }
  if (gps.location.isValid()) {
    gps_lat = gps.location.lat();
    gps_lon = gps.location.lng();
    gps_sat = gps.satellites.value();
  }
#endif
}

void readToF() {
#if ENABLE_VL53L0X
  if (tof0OK) {
    tcaSelect(0);
    VL53L0X_RangingMeasurementData_t m;
    tof0.rangingTest(&m, false);
    distFront_mm = (m.RangeStatus != 4) ? m.RangeMilliMeter : 0;
  }
  if (tof1OK) {
    tcaSelect(1);
    VL53L0X_RangingMeasurementData_t m;
    tof1.rangingTest(&m, false);
    distBack_mm = (m.RangeStatus != 4) ? m.RangeMilliMeter : 0;
  }
#endif
}

// ============================================================
// 通訊
// ============================================================
void recvData() {
#if ENABLE_NRF24
  while (radio.available()) {
    radio.read(&data, sizeof(Signal));
    lastRxTime = millis();
  }
#else
  lastRxTime = millis();
#endif
}

void failsafe() {
#if ENABLE_NRF24
  if (millis() - lastRxTime > 1000) {
    resetData();
    armed = false;
    i_roll = i_pitch = i_yaw = 0;
  }
#endif
}

void checkArm() {
  if (data.aux1 == 1 && data.throttle < 20 && !armed) {
    armed = true;
  } else if (data.aux1 == 0 && armed) {
    armed = false;
    i_roll = i_pitch = i_yaw = 0;
  }
}

void checkCalibrationTrigger() {
  static byte lastAux2 = 0;
  if (!armed && data.throttle < 5 &&
      lastAux2 == 0 && data.aux2 == 1) {
    Serial.println("\n[!] aux2 觸發校正");
    calibrateGyro();
  }
  lastAux2 = data.aux2;
}

// ============================================================
// PID 控制
// ============================================================
void pidControl() {
#if ENABLE_PID
  static unsigned long lastT = 0;
  unsigned long now = micros();
  float dt = (now - lastT) / 1000000.0f;
  lastT = now;
  if (dt > 0.05f || dt <= 0) dt = 0.01f;

  if (!armed || data.throttle < 20) {
    i_roll = i_pitch = i_yaw = 0;
    rollOut = pitchOut = yawOut = 0;
    return;
  }

  float rollSet    = (data.roll  - 127) * (MAX_ANGLE   / 127.0f);
  float pitchSet   = (data.pitch - 127) * (MAX_ANGLE   / 127.0f);
  float yawRateSet = (data.yaw   - 127) * (MAX_YAWRATE / 127.0f);

  float gxRate = -(rawGy - gyroOffsetY) / 131.0f;
  float gyRate = -(rawGx - gyroOffsetX) / 131.0f;

  float errR = rollSet - roll;
  i_roll += Ki_rp * errR * dt;
  i_roll  = constrain(i_roll, -I_LIMIT, I_LIMIT);
  rollOut = Kp_rp * errR + i_roll - Kd_rp * gxRate;

  float errP = pitchSet - pitch;
  i_pitch += Ki_rp * errP * dt;
  i_pitch  = constrain(i_pitch, -I_LIMIT, I_LIMIT);
  pitchOut = Kp_rp * errP + i_pitch - Kd_rp * gyRate;

  float errY = yawRateSet - yawRate;
  i_yaw += Ki_y * errY * dt;
  i_yaw  = constrain(i_yaw, -I_LIMIT, I_LIMIT);
  yawOut = Kp_y * errY + i_yaw;
#endif
}

void writeMotors() {
#if ENABLE_ESC
  if (!armed) {
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
    return;
  }

  int baseT = map(data.throttle, 0, 255, 1000, 2000);

#if ENABLE_PID
  int m1 = baseT + pitchOut + rollOut - yawOut;
  int m2 = baseT + pitchOut - rollOut + yawOut;
  int m3 = baseT - pitchOut + rollOut + yawOut;
  int m4 = baseT - pitchOut - rollOut - yawOut;

  m1 = constrain(m1, 1000, 2000);
  m2 = constrain(m2, 1000, 2000);
  m3 = constrain(m3, 1000, 2000);
  m4 = constrain(m4, 1000, 2000);

  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);
  esc4.writeMicroseconds(m4);
#else
  baseT = constrain(baseT, 1000, 2000);
  esc1.writeMicroseconds(baseT);
  esc2.writeMicroseconds(baseT);
  esc3.writeMicroseconds(baseT);
  esc4.writeMicroseconds(baseT);
#endif
#endif
}

// ============================================================
// 序列埠調參
// ============================================================
void parseSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;

  if (s.startsWith("kp ")) {
    Kp_rp = s.substring(3).toFloat();
    Serial.printf(">> Kp_rp = %.3f\n", Kp_rp);
  } else if (s.startsWith("ki ")) {
    Ki_rp = s.substring(3).toFloat();
    Serial.printf(">> Ki_rp = %.3f\n", Ki_rp);
  } else if (s.startsWith("kd ")) {
    Kd_rp = s.substring(3).toFloat();
    Serial.printf(">> Kd_rp = %.3f\n", Kd_rp);
  } else if (s.startsWith("ypp ")) {
    Kp_y = s.substring(4).toFloat();
    Serial.printf(">> Kp_y = %.3f\n", Kp_y);
  } else if (s.startsWith("yi ")) {
    Ki_y = s.substring(3).toFloat();
    Serial.printf(">> Ki_y = %.3f\n", Ki_y);
  } else if (s == "p") {
    Serial.printf(">> RP: Kp=%.3f Ki=%.3f Kd=%.3f | Yaw: Kp=%.3f Ki=%.3f\n",
                  Kp_rp, Ki_rp, Kd_rp, Kp_y, Ki_y);
  } else if (s == "cal") {
    calibrateGyro();
  } else {
    Serial.println(">> 指令：kp/ki/kd/ypp/yi <值>、p、cal");
  }
}

// ============================================================
// 序列埠輸出
// ============================================================
void debugPrint() {
  if (millis() - lastDbgTime < 200) return;
  lastDbgTime = millis();

  Serial.printf("R%6.1f P%6.1f Yr%6.1f | Thr%3d arm%d | PID r%+5.0f p%+5.0f y%+5.0f",
                roll, pitch, yawRate,
                data.throttle, armed,
                rollOut, pitchOut, yawOut);

#if ENABLE_BMP280
  if (bmp280OK) Serial.printf(" | Alt %.2fm", altitude_m);
#endif

#if ENABLE_GPS
  if (gps.location.isValid()) {
    Serial.printf(" | GPS %.5f,%.5f sat%d", gps_lat, gps_lon, gps_sat);
  } else {
    Serial.printf(" | GPS --- sat%d", gps_sat);
  }
#endif

#if ENABLE_VL53L0X
  Serial.printf(" | F%4dmm B%4dmm", distFront_mm, distBack_mm);
#endif

  Serial.println();
}

// ============================================================
// 感測器初始化
// ============================================================
void initBMP280() {
#if ENABLE_BMP280
  if (bmp.begin(ADDR_BMP280)) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    bmp280OK = true;
    Serial.println("[+] BMP280: OK");
  } else {
    Serial.println("[!] BMP280: FAIL");
  }
#endif
}

void initGPS() {
#if ENABLE_GPS
  Serial2.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.println("[+] GPS UART2 9600 啟動（要時間 cold start）");
#endif
}

void initToF() {
#if ENABLE_VL53L0X
  tcaSelect(0);
  if (tof0.begin(0x29)) {
    tof0OK = true;
    Serial.println("[+] VL53L0X #1 (前): OK");
  } else {
    Serial.println("[!] VL53L0X #1 (前): FAIL");
  }
  tcaSelect(1);
  if (tof1.begin(0x29)) {
    tof1OK = true;
    Serial.println("[+] VL53L0X #2 (後): OK");
  } else {
    Serial.println("[!] VL53L0X #2 (後): FAIL");
  }
#endif
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== 階段 4: 完整版（PID + 感測器擴充）===");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Serial.println("[1] I2C ready");

  mpu.initialize();
  Serial.print("[2] MPU6050: ");
  Serial.println(mpu.testConnection() ? "OK" : "FAIL");

  initBMP280();
  initToF();
  initGPS();

  calibrateGyro();

#if ENABLE_ESC
  esc1.attach(PIN_ESC1, 1000, 2000);
  esc2.attach(PIN_ESC2, 1000, 2000);
  esc3.attach(PIN_ESC3, 1000, 2000);
  esc4.attach(PIN_ESC4, 1000, 2000);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  Serial.println("[3] ESC 初始化");
  delay(2000);
#endif

#if ENABLE_NRF24
  radio.begin();
  radio.openReadingPipe(1, PIPE_IN);
  radio.setChannel(NRF_CHANNEL);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  Serial.println("[4] NRF24 listening");
#endif

  Serial.printf("[5] PID: Kp=%.2f Ki=%.3f Kd=%.2f | Yaw Kp=%.2f Ki=%.3f\n",
                Kp_rp, Ki_rp, Kd_rp, Kp_y, Ki_y);
  resetData();
  Serial.println("=== Ready (disarmed) ===\n");
}

void loop() {
  recvData();
  failsafe();
  checkArm();
  checkCalibrationTrigger();

  readAttitude();
  readBMP280();
  readGPS();
  readToF();

  pidControl();
  writeMotors();

  debugPrint();
  parseSerial();
}
