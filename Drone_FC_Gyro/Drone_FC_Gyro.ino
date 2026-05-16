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
#define PIN_BUZZER   13
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
unsigned long lastRxTime  = 0;
unsigned long lastDbgTime = 0;
float         roll = 0, pitch = 0, yawRate = 0;

// ============================================================
void resetData() {
  data.throttle = 0;
  data.pitch = data.roll = data.yaw = 127;
  data.aux1 = data.aux2 = 0;
}

void readAttitude() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  static unsigned long lastT = 0;
  unsigned long now = micros();
  float dt = (now - lastT) / 1000000.0f;
  lastT = now;
  if (dt > 0.05f || dt <= 0) dt = 0.01f;

  // 加速度計算重力傾角
  float accRoll  = atan2f(ay, az) * 57.2958f;
  float accPitch = atan2f(-ax, sqrtf((float)ay*ay + (float)az*az)) * 57.2958f;

  // 陀螺儀轉速 (±250°/s 範圍下，LSB = 131)
  float gRoll  = gx / 131.0f;
  float gPitch = gy / 131.0f;
  yawRate      = gz / 131.0f;

  // 互補濾波：98% 陀螺儀短期 + 2% 加速度計長期
  roll  = 0.98f * (roll  + gRoll  * dt) + 0.02f * accRoll;
  pitch = 0.98f * (pitch + gPitch * dt) + 0.02f * accPitch;
}

void recvData() {
  while (radio.available()) {
    radio.read(&data, sizeof(Signal));
    lastRxTime = millis();
  }
}

void failsafe() {
  if (millis() - lastRxTime > 1000) {
    resetData();
    armed = false;
  }
}

void checkArm() {
  // aux1 = 1 且油門接近最低 → arm
  // aux1 = 0 → disarm
  if (data.aux1 == 1 && data.throttle < 20 && !armed) {
    armed = true;
    tone(PIN_BUZZER, 2000, 200);
  } else if (data.aux1 == 0 && armed) {
    armed = false;
    tone(PIN_BUZZER, 500, 200);
  }
}

void writeMotors() {
  int us = armed ? map(data.throttle, 0, 255, 1000, 2000) : 1000;
  us = constrain(us, 1000, 2000);
  esc1.writeMicroseconds(us);
  esc2.writeMicroseconds(us);
  esc3.writeMicroseconds(us);
  esc4.writeMicroseconds(us);
}

void debugPrint() {
  if (millis() - lastDbgTime < 200) return;
  lastDbgTime = millis();
  Serial.printf("R%6.1f P%6.1f Yr%6.1f | Thr%3d aux1%d aux2%d | armed%d\n",
                roll, pitch, yawRate,
                data.throttle, data.aux1, data.aux2, armed);
}

// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZZER, OUTPUT);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  mpu.initialize();
  Serial.print("MPU6050: ");
  Serial.println(mpu.testConnection() ? "OK" : "FAIL");

  esc1.attach(PIN_ESC1, 1000, 2000);
  esc2.attach(PIN_ESC2, 1000, 2000);
  esc3.attach(PIN_ESC3, 1000, 2000);
  esc4.attach(PIN_ESC4, 1000, 2000);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(2000);   // ESC 識別最低油門

  radio.begin();
  radio.openReadingPipe(1, PIPE_IN);
  radio.setChannel(NRF_CHANNEL);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();

  resetData();
  tone(PIN_BUZZER, 1500, 100);
  Serial.println("Drone Ready (disarmed)");
}

void loop() {
  recvData();
  failsafe();
  checkArm();
  readAttitude();
  writeMotors();
  debugPrint();
}
