// ============================================================
// 飛控階段 1：陀螺儀 + NRF24 + ESC
// MCU: ESP32 DevKit V1 (30-pin)
//
// 功能：
//   - 讀 MPU6050 姿態（互補濾波出 Roll / Pitch）
//   - 接收地面 TX 送來的 Signal 結構（與你現有 TX 程式完全相容）
//   - 控制 4 顆 ESC（無 PID，純油門透傳）
//   - 失聯 1 秒自動解除武裝
//
// ⚠️ 安全：
//   - 預設 disarmed（馬達不轉）
//   - 把 aux1 三段開關打到 1 才會 arm
//   - 螺旋槳請等 PID 寫完再裝
// ============================================================

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

// ---- 腳位 ----
#define PIN_NRF_CE   4
#define PIN_NRF_CSN  5
#define PIN_ESC1     25   // M1 前左 CW
#define PIN_ESC2     26   // M2 前右 CCW
#define PIN_ESC3     27   // M3 後左 CCW
#define PIN_ESC4     14   // M4 後右 CW
#define PIN_SDA      21
#define PIN_SCL      22

// ---- NRF24 設定（與 TX 一致）----
const uint64_t PIPE_IN     = 0xABCDABCD71LL;
const uint8_t  NRF_CHANNEL = 100;

// ---- 通訊資料結構（與你原本 TX 完全一致）----
struct Signal {
  byte throttle;
  byte pitch;
  byte roll;
  byte yaw;
  byte aux1;
  byte aux2;
};

RF24    radio(PIN_NRF_CE, PIN_NRF_CSN);
MPU6050 mpu;
Servo   esc1, esc2, esc3, esc4;

Signal data;
bool          armed       = false;
bool          mpuOK       = false;   // 設為 true 才會真正讀感測器
unsigned long lastRxTime  = 0;
unsigned long lastDbgTime = 0;
float         roll = 0, pitch = 0, yawRate = 0;

// ============================================================
void resetData() {
  data.throttle = 0;
  data.pitch = data.roll = data.yaw = 127;
  data.aux1 = data.aux2 = 0;
}

// 原始感測器值，給 debugPrint 顯示用
int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;

void readAttitude() {
  mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

  static unsigned long lastT = 0;
  unsigned long now = micros();
  float dt = (now - lastT) / 1000000.0f;
  lastT = now;
  if (dt > 0.05f || dt <= 0) dt = 0.01f;

  // 加速度計算重力傾角
  float accRoll  = atan2f(rawAy, rawAz) * 57.2958f;
  float accPitch = atan2f(-rawAx, sqrtf((float)rawAy*rawAy + (float)rawAz*rawAz)) * 57.2958f;

  // 陀螺儀轉速 (±250°/s 範圍下，LSB = 131)
  float gRoll  = rawGx / 131.0f;
  float gPitch = rawGy / 131.0f;
  yawRate      = rawGz / 131.0f;

  // 互補濾波：98% 陀螺儀短期 + 2% 加速度計長期
  roll  = 0.98f * (roll  + gRoll  * dt) + 0.02f * accRoll;
  pitch = 0.98f * (pitch + gPitch * dt) + 0.02f * accPitch;
}

void recvData() {
#if ENABLE_NRF24
  while (radio.available()) {
    radio.read(&data, sizeof(Signal));
    lastRxTime = millis();
  }
#else
  lastRxTime = millis();   // 沒接 NRF24 時，failsafe 不會誤觸發
#endif
}

void failsafe() {
#if ENABLE_NRF24
  if (millis() - lastRxTime > 1000) {
    resetData();
    armed = false;
  }
#endif
}

void checkArm() {
  // aux1 = 1 且油門接近最低 → arm
  // aux1 = 0 → disarm
  if (data.aux1 == 1 && data.throttle < 20 && !armed) {
    armed = true;
  } else if (data.aux1 == 0 && armed) {
    armed = false;
  }
}

void writeMotors() {
#if ENABLE_ESC
  int us = armed ? map(data.throttle, 0, 255, 1000, 2000) : 1000;
  us = constrain(us, 1000, 2000);
  esc1.writeMicroseconds(us);
  esc2.writeMicroseconds(us);
  esc3.writeMicroseconds(us);
  esc4.writeMicroseconds(us);
#endif
}

void debugPrint() {
  if (millis() - lastDbgTime < 200) return;
  lastDbgTime = millis();
  // 原始感測器值（除錯用）
  Serial.printf("raw ax=%6d ay=%6d az=%6d | gx=%6d gy=%6d gz=%6d\n",
                rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);
  Serial.printf("R%6.1f P%6.1f Yr%6.1f | Thr%3d aux1%d aux2%d | armed%d\n",
                roll, pitch, yawRate,
                data.throttle, data.aux1, data.aux2, armed);
}

// ============================================================
// 階段 1 診斷模式：不接 NRF24 / ESC 也能跑
//   - 把 #define ENABLE_NRF24 / ENABLE_ESC 改 0 即可跳過
#define ENABLE_NRF24  0    // 沒接 NRF24 時設 0
#define ENABLE_ESC    0    // 沒接 ESC 時設 0
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);                    // 讓序列埠穩定，避免漏訊息
  Serial.println("\n=== Drone FC boot ===");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Serial.println("[1] I2C ready");

  mpu.initialize();
  Serial.print("[2] MPU6050: ");
  Serial.println(mpu.testConnection() ? "OK" : "FAIL (檢查接線)");

#if ENABLE_ESC
  esc1.attach(PIN_ESC1, 1000, 2000);
  esc2.attach(PIN_ESC2, 1000, 2000);
  esc3.attach(PIN_ESC3, 1000, 2000);
  esc4.attach(PIN_ESC4, 1000, 2000);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  Serial.println("[3] ESC armed at 1000us");
  delay(2000);
#else
  Serial.println("[3] ESC skipped (ENABLE_ESC=0)");
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
#else
  Serial.println("[4] NRF24 skipped (ENABLE_NRF24=0)");
#endif

  resetData();
  Serial.println("=== Ready (disarmed) ===");
}

void loop() {
  recvData();
  failsafe();
  checkArm();
  readAttitude();
  writeMotors();
  debugPrint();
}
