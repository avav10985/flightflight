// ============================================================
// 四軸無人機 飛控主程式（階段 2：硬體驗證）
// MCU: ESP32 DevKit V1 (ESP32-WROOM-32, 30-pin)
//
// 功能：
//   - 讀取 MPU6050（姿態）、BMP280（高度）、NEO-6M GPS、VL53L1X×4（避障）
//   - 透過 NRF24L01+PA/LNA 接收地面指令
//   - 透過 PWM 控制 4 顆 ESC（無刷馬達）
//   - 失聯 1 秒自動解除武裝（Failsafe）
//
// ⚠️ 安全：
//   - armed = false 是預設狀態，馬達不會轉
//   - 收到 cmd_type = ARM 才會啟動
//   - 程式不含 PID 自穩，請先「不裝螺旋槳」做硬體測試
// ============================================================

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#include <VL53L1X.h>
#include <ESP32Servo.h>

// ---------- 腳位定義 ----------
#define PIN_NRF_CE      4
#define PIN_NRF_CSN     5
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22
#define PIN_GPS_RX      16   // 接 GPS 的 TX
#define PIN_GPS_TX      17   // 接 GPS 的 RX
#define PIN_ESC1        25   // M1 前左
#define PIN_ESC2        26   // M2 前右
#define PIN_ESC3        27   // M3 後左
#define PIN_ESC4        14   // M4 後右
#define PIN_BUZZER      13
#define PIN_BAT_ADC     34

// ---------- I2C 地址 ----------
#define ADDR_MPU6050    0x68
#define ADDR_BMP280     0x76
#define ADDR_TCA9548A   0x70

// ---------- NRF24 設定（與你 TX 程式相容） ----------
const uint64_t PIPE_ADDR = 0xABCDABCD71LL;
const uint8_t  NRF_CHANNEL = 100;

// ---------- 電池分壓參數 ----------
const float BAT_DIVIDER_RATIO = 4.0;   // (30k+10k)/10k
const float BAT_LOW_VOLTAGE   = 11.0;  // 3S 低電警報

// ---------- 指令型態 ----------
enum CmdType : uint8_t {
  CMD_DISARM = 0,
  CMD_ARM    = 1,
  CMD_MANUAL = 2,
  CMD_HOVER  = 3,
  CMD_GOTO   = 4,
  CMD_RTL    = 5,
  CMD_LAND   = 6
};

// ---------- 通訊資料結構 ----------
struct Command {
  uint8_t cmd_type;
  uint8_t throttle;   // 0~255
  uint8_t pitch;      // 127 中位
  uint8_t roll;       // 127 中位
  uint8_t yaw;        // 127 中位
  uint8_t aux1;
  uint8_t aux2;
  int32_t param1;     // 例：GOTO 的 lat * 1e7
  int32_t param2;     // 例：GOTO 的 lon * 1e7
  int32_t param3;     // 例：GOTO 的 alt(cm)
};

struct Telemetry {
  float    roll, pitch, yaw;
  float    altitude;
  float    battery;
  uint16_t dist_F, dist_R, dist_B, dist_L;
  float    gps_lat, gps_lon;
  uint8_t  gps_sat;
  uint8_t  status;        // bit0=armed, bit1=lowBat, bit2=gpsFix
};

// ---------- 全域物件 ----------
RF24            radio(PIN_NRF_CE, PIN_NRF_CSN);
MPU6050         mpu;
Adafruit_BMP280 bmp;
TinyGPSPlus     gps;
VL53L1X         tof[4];
Servo           esc1, esc2, esc3, esc4;

Command   cmd;
Telemetry tele;

bool          armed       = false;
unsigned long lastRxTime  = 0;
unsigned long lastDbgTime = 0;

// ============================================================
// 工具函式
// ============================================================
void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(ADDR_TCA9548A);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void beep(uint16_t freq, uint16_t ms) {
  tone(PIN_BUZZER, freq, ms);
}

void resetCommand() {
  cmd.cmd_type = CMD_DISARM;
  cmd.throttle = 0;
  cmd.pitch = cmd.roll = cmd.yaw = 127;
  cmd.aux1 = cmd.aux2 = 0;
}

// ============================================================
// 感測器初始化
// ============================================================
bool initMPU() {
  mpu.initialize();
  return mpu.testConnection();
}

bool initBMP() {
  return bmp.begin(ADDR_BMP280);
}

void initToF() {
  for (uint8_t i = 0; i < 4; i++) {
    tcaSelect(i);
    tof[i].setTimeout(100);
    if (!tof[i].init()) {
      Serial.printf("VL53L1X #%d init fail\n", i);
      continue;
    }
    tof[i].setDistanceMode(VL53L1X::Long);
    tof[i].setMeasurementTimingBudget(50000);
    tof[i].startContinuous(50);
  }
}

void initESC() {
  esc1.attach(PIN_ESC1, 1000, 2000);
  esc2.attach(PIN_ESC2, 1000, 2000);
  esc3.attach(PIN_ESC3, 1000, 2000);
  esc4.attach(PIN_ESC4, 1000, 2000);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(2000); // 等 ESC 識別「最低油門」
}

void initRadio() {
  radio.begin();
  radio.setChannel(NRF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.openReadingPipe(1, PIPE_ADDR);
  radio.startListening();
}

// ============================================================
// 感測器讀取
// ============================================================
void readMPU() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 簡易互補濾波（粗略，僅供顯示，PID 控制不能直接用這個）
  static float roll = 0, pitch = 0;
  static unsigned long lastT = 0;
  unsigned long now = micros();
  float dt = (now - lastT) / 1000000.0f;
  lastT = now;
  if (dt > 0.05f) dt = 0.01f;

  float accRoll  = atan2f(ay, az) * 57.2958f;
  float accPitch = atan2f(-ax, sqrtf((float)ay*ay + (float)az*az)) * 57.2958f;
  float gyroRollRate  = gx / 131.0f;
  float gyroPitchRate = gy / 131.0f;
  float gyroYawRate   = gz / 131.0f;

  roll  = 0.98f * (roll  + gyroRollRate  * dt) + 0.02f * accRoll;
  pitch = 0.98f * (pitch + gyroPitchRate * dt) + 0.02f * accPitch;
  tele.yaw += gyroYawRate * dt;   // 純積分，會漂移

  tele.roll  = roll;
  tele.pitch = pitch;
}

void readBMP() {
  tele.altitude = bmp.readAltitude(1013.25);
}

void readGPS() {
  while (Serial2.available()) gps.encode(Serial2.read());
  if (gps.location.isValid()) {
    tele.gps_lat = gps.location.lat();
    tele.gps_lon = gps.location.lng();
    tele.gps_sat = gps.satellites.value();
    tele.status |= 0x04;
  }
}

void readToF() {
  uint16_t d[4] = {0, 0, 0, 0};
  for (uint8_t i = 0; i < 4; i++) {
    tcaSelect(i);
    if (tof[i].dataReady()) d[i] = tof[i].read();
  }
  tele.dist_F = d[0];
  tele.dist_R = d[1];
  tele.dist_B = d[2];
  tele.dist_L = d[3];
}

void readBattery() {
  int raw = analogRead(PIN_BAT_ADC);
  tele.battery = raw * (3.3f / 4095.0f) * BAT_DIVIDER_RATIO;
  if (tele.battery < BAT_LOW_VOLTAGE) {
    tele.status |= 0x02;
  } else {
    tele.status &= ~0x02;
  }
}

// ============================================================
// 通訊
// ============================================================
void recvCommand() {
  if (radio.available()) {
    radio.read(&cmd, sizeof(Command));
    lastRxTime = millis();

    switch (cmd.cmd_type) {
      case CMD_ARM:
        if (!armed && cmd.throttle < 10) {
          armed = true;
          tele.status |= 0x01;
          beep(2000, 200);
        }
        break;
      case CMD_DISARM:
        armed = false;
        tele.status &= ~0x01;
        beep(500, 200);
        break;
      default:
        break;
    }
  }
}

// ============================================================
// 安全機制
// ============================================================
void failsafe() {
  if (millis() - lastRxTime > 1000) {
    armed = false;
    tele.status &= ~0x01;
    resetCommand();
  }
}

// ============================================================
// 馬達輸出（階段 2：直接油門，無 PID）
// ⚠️ 階段 3 才會加入 PID 混控
// ============================================================
void writeMotors() {
  if (!armed) {
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
    return;
  }

  // 暫時：四顆馬達同油門（不會飛，但能驗證 ESC 接線）
  int us = map(cmd.throttle, 0, 255, 1000, 2000);
  us = constrain(us, 1000, 2000);
  esc1.writeMicroseconds(us);
  esc2.writeMicroseconds(us);
  esc3.writeMicroseconds(us);
  esc4.writeMicroseconds(us);
}

// ============================================================
// 序列埠除錯輸出（每 200ms）
// ============================================================
void printDebug() {
  if (millis() - lastDbgTime < 200) return;
  lastDbgTime = millis();

  Serial.printf("R%.1f P%.1f Y%.1f | Alt%.1f | Bat%.2fV | Sat%d | F%u R%u B%u L%u | Armed%d Thr%d\n",
                tele.roll, tele.pitch, tele.yaw,
                tele.altitude, tele.battery, tele.gps_sat,
                tele.dist_F, tele.dist_R, tele.dist_B, tele.dist_L,
                armed, cmd.throttle);
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BAT_ADC, INPUT);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Serial2.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  resetCommand();
  beep(1500, 100);

  Serial.println("\n=== Drone FC boot ===");
  Serial.printf("MPU6050: %s\n", initMPU() ? "OK" : "FAIL");
  Serial.printf("BMP280 : %s\n", initBMP() ? "OK" : "FAIL");
  initToF();
  initRadio();
  initESC();

  Serial.println("Ready. ARM via NRF24 to spin motors.");
  beep(2000, 100); delay(50); beep(2000, 100);
}

void loop() {
  recvCommand();
  failsafe();

  readMPU();
  readBMP();
  readGPS();
  readToF();
  readBattery();

  writeMotors();
  printDebug();
}
