// ============================================================
// 階段 3：PID 自穩飛控
// MCU: LOLIN D32 (ESP32-WROOM-32)
//
// 功能：
//   - MPU6050 姿態 + 陀螺儀偏移自動校正
//   - NRF24 接收搖桿
//   - Roll/Pitch 角度 PID、Yaw 角速度 PID
//   - X 型 4 馬達混控
//   - 序列埠即時調參（不用每次重燒）
//
// 需要的硬體：
//   - 階段 2 全部
//
// 用途：
//   - 開始正式調 PID
//   - 拘束測試（綁桿子 / 繩子）
//   - 確認穩定後才裝螺旋槳
//
// 序列埠指令：
//   kp 1.5     ← Roll/Pitch Kp
//   ki 0.05    ← Roll/Pitch Ki
//   kd 0.3     ← Roll/Pitch Kd
//   ypp 2.0    ← Yaw Kp
//   yi 0.01    ← Yaw Ki
//   p          ← 顯示目前所有值
//   cal        ← 重新校正陀螺儀
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
#define PIN_BAT_ADC  34   // 電池分壓中點

// ---- NRF24 設定 ----
const uint64_t PIPE_IN     = 0xABCDABCD71LL;
const uint8_t  NRF_CHANNEL = 100;

// ---- 電池分壓 ----
const float BAT_DIVIDER = 4.0f;   // (30k+10k)/10k

// 地面 → 飛機（指令 + 參數）
struct Signal {
  byte throttle, pitch, roll, yaw, aux1, aux2;
  byte paramID;     // 0=無、1=Kp、2=Ki、3=Kd、4=Kp_y、5=Ki_y
  float paramVal;   // 參數值
};

// 飛機 → 地面（ACK payload 回傳遙測）
struct Telemetry {
  int16_t  roll;        // 角度 ×10
  int16_t  pitch;
  int16_t  yawRate;
  uint16_t battery_mV;  // 電壓 mV
  uint8_t  status;      // bit0=armed
  uint8_t  reserved;
};   // 10 bytes

RF24    radio(PIN_NRF_CE, PIN_NRF_CSN);
MPU6050 mpu;
Servo   esc1, esc2, esc3, esc4;

Signal        data;
Telemetry     tele;
float         batteryV    = 0;
bool          armed       = false;
unsigned long lastRxTime  = 0;
unsigned long lastDbgTime = 0;
float         roll = 0, pitch = 0, yawRate = 0;
int16_t       rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
float         gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float         levelOffsetRoll = 0, levelOffsetPitch = 0;   // 水平基準（解決 MPU6050 沒裝水平）

// ---- PID 增益（從拘束測試調出來的值）----
float Kp_rp = 2.0f, Ki_rp = 0.02f, Kd_rp = 0.5f;
float Kp_y  = 1.5f, Ki_y  = 0.02f;

const float I_LIMIT = 100.0f;
const float MAX_ANGLE = 30.0f;       // ±30° 期望姿態
const float MAX_YAWRATE = 90.0f;     // ±90°/s 期望偏航

float i_roll = 0, i_pitch = 0, i_yaw = 0;
float rollOut = 0, pitchOut = 0, yawOut = 0;

void calibrateGyro() {
  Serial.println("[*] 校正陀螺儀 + 水平基準（3 秒），請把飛機平放...");
  const int N = 300;
  long sx = 0, sy = 0, sz = 0;
  float sumRoll = 0, sumPitch = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < N; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sx += gx; sy += gy; sz += gz;
    // 同時計算靜態角度（用同樣的軸對齊公式）
    sumRoll  += atan2f(ax, az) * 57.2958f;
    sumPitch += atan2f(-ay, sqrtf((float)ax*ax + (float)az*az)) * 57.2958f;
    delay(5);
  }
  gyroOffsetX = sx / (float)N;
  gyroOffsetY = sy / (float)N;
  gyroOffsetZ = sz / (float)N;
  levelOffsetRoll  = sumRoll  / N;
  levelOffsetPitch = sumPitch / N;

  // 重置 PID 與互補濾波狀態
  roll = pitch = yawRate = 0;
  i_roll = i_pitch = i_yaw = 0;

  Serial.printf("[*] 陀螺儀偏移: gx=%.1f gy=%.1f gz=%.1f\n",
                gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  Serial.printf("[*] 水平基準: R=%.2f° P=%.2f°（已扣除）\n",
                levelOffsetRoll, levelOffsetPitch);
}

void resetData() {
  data.throttle = 0;
  data.pitch = data.roll = data.yaw = 127;
  data.aux1 = data.aux2 = 0;
  data.paramID = 0;
  data.paramVal = 0;
}

// 讀電池電壓（分壓 → ADC）
void readBattery() {
  int raw = analogRead(PIN_BAT_ADC);
  batteryV = raw * (3.3f / 4095.0f) * BAT_DIVIDER;
}

// 打包遙測（要回傳給地面）
void buildTelemetry() {
  tele.roll       = (int16_t)(roll * 10);
  tele.pitch      = (int16_t)(pitch * 10);
  tele.yawRate    = (int16_t)(yawRate * 10);
  tele.battery_mV = (uint16_t)(batteryV * 1000);
  tele.status     = armed ? 0x01 : 0x00;
  tele.reserved   = 0;
}

// 套用地面傳來的參數
void applyCommand() {
  switch (data.paramID) {
    case 1: Kp_rp = data.paramVal; Serial.printf(">> 收到 Kp=%.3f\n", Kp_rp); break;
    case 2: Ki_rp = data.paramVal; Serial.printf(">> 收到 Ki=%.3f\n", Ki_rp); break;
    case 3: Kd_rp = data.paramVal; Serial.printf(">> 收到 Kd=%.3f\n", Kd_rp); break;
    case 4: Kp_y  = data.paramVal; Serial.printf(">> 收到 Kp_y=%.3f\n", Kp_y); break;
    case 5: Ki_y  = data.paramVal; Serial.printf(">> 收到 Ki_y=%.3f\n", Ki_y); break;
    default: break;
  }
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

  // MPU6050 軸對齊修正：pitch 用 -Y、roll 用 +X
  // 並扣除水平基準（解決 MPU6050 沒裝水平）
  float accRoll  = atan2f(rawAx, rawAz) * 57.2958f - levelOffsetRoll;
  float accPitch = atan2f(-rawAy, sqrtf((float)rawAx*rawAx + (float)rawAz*rawAz)) * 57.2958f - levelOffsetPitch;

  float gRoll  = -gyCal / 131.0f;
  float gPitch = -gxCal / 131.0f;
  yawRate      =  gzCal / 131.0f;

  roll  = 0.98f * (roll  + gRoll  * dt) + 0.02f * accRoll;
  pitch = 0.98f * (pitch + gPitch * dt) + 0.02f * accPitch;
}

void recvData() {
  while (radio.available()) {
    // 收指令前先把遙測排進 ACK，等地面下次 write 過來就一起回傳
    buildTelemetry();
    radio.writeAckPayload(1, &tele, sizeof(tele));
    radio.read(&data, sizeof(Signal));
    lastRxTime = millis();
    if (data.paramID != 0) applyCommand();   // 有帶參數就套用
  }
}

void failsafe() {
  if (millis() - lastRxTime > 1000) {
    resetData();
    armed = false;
    i_roll = i_pitch = i_yaw = 0;
  }
}

void checkArm() {
  if (data.aux1 == 1 && data.throttle < 20 && !armed) {
    armed = true;
  } else if (data.aux1 == 0 && armed) {
    armed = false;
    i_roll = i_pitch = i_yaw = 0;
  }
}

// aux2 觸發校正：0 → 1 邊緣 + disarmed + 油門 0 才生效
void checkCalibrationTrigger() {
  static byte lastAux2 = 0;
  if (!armed && data.throttle < 5 &&
      lastAux2 == 0 && data.aux2 == 1) {
    Serial.println("\n[!] aux2 觸發校正（請保持飛機平放靜止）");
    calibrateGyro();
  }
  lastAux2 = data.aux2;
}

void pidControl() {
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

  // 軸對齊：D 項要用 drone 框架的角速度
  float gxRate = -(rawGy - gyroOffsetY) / 131.0f;   // roll rate
  float gyRate = -(rawGx - gyroOffsetX) / 131.0f;   // pitch rate

  // Roll
  float errR = rollSet - roll;
  i_roll += Ki_rp * errR * dt;
  i_roll  = constrain(i_roll, -I_LIMIT, I_LIMIT);
  rollOut = Kp_rp * errR + i_roll - Kd_rp * gxRate;

  // Pitch
  float errP = pitchSet - pitch;
  i_pitch += Ki_rp * errP * dt;
  i_pitch  = constrain(i_pitch, -I_LIMIT, I_LIMIT);
  pitchOut = Kp_rp * errP + i_pitch - Kd_rp * gyRate;

  // Yaw（角速度，無 D）
  float errY = yawRateSet - yawRate;
  i_yaw += Ki_y * errY * dt;
  i_yaw  = constrain(i_yaw, -I_LIMIT, I_LIMIT);
  yawOut = Kp_y * errY + i_yaw;
}

void writeMotors() {
  if (!armed) {
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
    return;
  }

  int baseT = map(data.throttle, 0, 255, 1000, 2000);

  // X 型混控
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
}

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

void debugPrint() {
  if (millis() - lastDbgTime < 200) return;
  lastDbgTime = millis();
  Serial.printf("R%6.1f P%6.1f Yr%6.1f | Thr%3d arm%d | Bat%.2fV | PID r%+5.0f p%+5.0f y%+5.0f\n",
                roll, pitch, yawRate,
                data.throttle, armed, batteryV,
                rollOut, pitchOut, yawOut);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== 階段 3: PID 自穩 ===");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Serial.println("[1] I2C ready");

  mpu.initialize();
  Serial.print("[2] MPU6050: ");
  Serial.println(mpu.testConnection() ? "OK" : "FAIL");

  calibrateGyro();

  esc1.attach(PIN_ESC1, 1000, 2000);
  esc2.attach(PIN_ESC2, 1000, 2000);
  esc3.attach(PIN_ESC3, 1000, 2000);
  esc4.attach(PIN_ESC4, 1000, 2000);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  Serial.println("[3] ESC 初始化 1000µs");
  delay(2000);

  pinMode(PIN_BAT_ADC, INPUT);

  radio.begin();
  radio.openReadingPipe(1, PIPE_IN);
  radio.setChannel(NRF_CHANNEL);
  radio.setAutoAck(true);          // 雙向必須 true
  radio.enableAckPayload();        // 開啟 ACK 回傳遙測
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  Serial.println("[4] NRF24 listening（雙向 ACK 遙測已開）");

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
  readBattery();
  pidControl();
  writeMotors();
  debugPrint();
  parseSerial();
}
