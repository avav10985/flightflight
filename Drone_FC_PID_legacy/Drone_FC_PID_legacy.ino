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
#include <WiFi.h>
#include <WebServer.h>

// ---- WiFi Debug AP(免線即時遙測:連 DroneDebug → 開 192.168.4.1)----
const char* DBG_AP_SSID = "DroneDebug";
const char* DBG_AP_PASS = "drone1234";
WebServer dbgServer(80);

// ---- 腳位 ----
#define PIN_NRF_CE   4
#define PIN_NRF_CSN  5
#define PIN_ESC1     25   // M1 前左 CW
#define PIN_ESC2     26   // M2 前右 CCW
#define PIN_ESC3     27   // M3 後左 CCW
#define PIN_ESC4     14   // M4 後右 CW
#define PIN_SDA      21
#define PIN_SCL      22

// ---- NRF24 設定 ----
const uint64_t PIPE_IN     = 0xABCDABCD71LL;
const uint8_t  NRF_CHANNEL = 100;

struct Signal {
  byte throttle, pitch, roll, yaw, aux1, aux2;
};

RF24    radio(PIN_NRF_CE, PIN_NRF_CSN);
MPU6050 mpu;
Servo   esc1, esc2, esc3, esc4;

Signal        data;
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
    radio.read(&data, sizeof(Signal));
    lastRxTime = millis();
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
  Serial.printf("R%6.1f P%6.1f Yr%6.1f | Thr%3d arm%d | PID r%+5.0f p%+5.0f y%+5.0f\n",
                roll, pitch, yawRate,
                data.throttle, armed,
                rollOut, pitchOut, yawOut);
}

// ============================================================
// Debug HTTP:瀏覽器開 http://192.168.4.1 自動每 0.3 秒更新
// ============================================================
void handleDebugRoot() {
  char buf[1200];
  snprintf(buf, sizeof(buf),
    "<!DOCTYPE html><html><head>"
    "<meta http-equiv='refresh' content='0.3'>"
    "<title>Drone Debug</title>"
    "<style>body{font-family:monospace;font-size:18px;background:#111;color:#0f0;padding:20px;line-height:1.5}"
    ".big{font-size:24px;color:#ff0}.armed{color:#f33;font-weight:bold}.safe{color:#888}</style>"
    "</head><body><pre>"
    "<span class='big'>Roll  : %+7.2f deg</span>\n"
    "<span class='big'>Pitch : %+7.2f deg</span>\n"
    "<span class='big'>YawRate: %+7.2f deg/s</span>\n\n"
    "Throttle: %3d / 255    (PWM %d us)\n"
    "ARMED: <span class='%s'>%s</span>\n\n"
    "PID output:\n"
    "  rollOut : %+8.2f\n"
    "  pitchOut: %+8.2f\n"
    "  yawOut  : %+8.2f\n\n"
    "PID gains (R/P):  Kp=%.3f  Ki=%.3f  Kd=%.3f\n"
    "PID gains (Yaw):  Kp=%.3f  Ki=%.3f\n\n"
    "MAX_ANGLE: %.1f deg    MAX_YAWRATE: %.1f deg/s\n"
    "</pre></body></html>",
    roll, pitch, yawRate,
    data.throttle, map(data.throttle, 0, 255, 1000, 2000),
    armed ? "armed" : "safe",
    armed ? "YES" : " NO",
    rollOut, pitchOut, yawOut,
    Kp_rp, Ki_rp, Kd_rp,
    Kp_y, Ki_y,
    MAX_ANGLE, MAX_YAWRATE);
  dbgServer.send(200, "text/html", buf);
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

  radio.begin();
  radio.openReadingPipe(1, PIPE_IN);
  radio.setChannel(NRF_CHANNEL);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  Serial.println("[4] NRF24 listening");

  Serial.println("[5] PID 啟動（所有增益 = 0，等指令）");
  Serial.println("    指令範例：kp 1.0、ki 0.02、kd 0.5、ypp 2.0、p、cal");

  // WiFi Debug AP(用 channel 1,離 NRF24 ch100 很遠不會干擾)
  WiFi.mode(WIFI_AP);
  WiFi.softAP(DBG_AP_SSID, DBG_AP_PASS, 1);
  Serial.printf("[6] Debug AP \"%s\"(密碼 %s)IP=%s\n",
                DBG_AP_SSID, DBG_AP_PASS, WiFi.softAPIP().toString().c_str());
  dbgServer.on("/", handleDebugRoot);
  dbgServer.begin();
  Serial.println("    瀏覽器開 http://192.168.4.1 看即時遙測");

  resetData();
  Serial.println("=== Ready (disarmed) ===\n");
}

void loop() {
  recvData();
  failsafe();
  checkArm();
  checkCalibrationTrigger();
  readAttitude();
  pidControl();
  writeMotors();
  debugPrint();
  parseSerial();
  dbgServer.handleClient();
}
