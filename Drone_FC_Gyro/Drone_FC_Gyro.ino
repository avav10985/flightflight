// ============================================================
// 階段 1：純陀螺儀讀取
// MCU: LOLIN D32 (ESP32-WROOM-32)
//
// 功能：
//   - 讀 MPU6050 姿態（互補濾波 Roll / Pitch）
//   - 序列埠輸出 raw 值 + 角度
//   - 開機自動陀螺儀偏移校正
//
// 需要的硬體：
//   - LOLIN D32 + USB 線
//   - MPU6050（VCC、GND、SDA→21、SCL→22）
//
// 用途：
//   - 驗證 I²C 通訊與感測器讀數
//   - 還不能控制馬達、不需要 NRF24
// ============================================================

#include <Wire.h>
#include <MPU6050.h>

#define PIN_SDA  21
#define PIN_SCL  22

MPU6050 mpu;

float    roll = 0, pitch = 0, yawRate = 0;
int16_t  rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
float    gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float    levelOffsetRoll = 0, levelOffsetPitch = 0;
unsigned long lastDbgTime = 0;

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
  Serial.printf("[*] 陀螺儀偏移: gx=%.1f gy=%.1f gz=%.1f\n",
                gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  Serial.printf("[*] 水平基準: R=%.2f° P=%.2f°（已扣除）\n",
                levelOffsetRoll, levelOffsetPitch);
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

  // MPU6050 軸對齊修正 + 扣水平基準
  float accRoll  = atan2f(rawAx, rawAz) * 57.2958f - levelOffsetRoll;
  float accPitch = atan2f(-rawAy, sqrtf((float)rawAx*rawAx + (float)rawAz*rawAz)) * 57.2958f - levelOffsetPitch;

  float gRoll  = -gyCal / 131.0f;
  float gPitch = -gxCal / 131.0f;
  yawRate      =  gzCal / 131.0f;

  roll  = 0.98f * (roll  + gRoll  * dt) + 0.02f * accRoll;
  pitch = 0.98f * (pitch + gPitch * dt) + 0.02f * accPitch;
}

void debugPrint() {
  if (millis() - lastDbgTime < 200) return;
  lastDbgTime = millis();
  Serial.printf("raw ax=%6d ay=%6d az=%6d | gx=%6d gy=%6d gz=%6d\n",
                rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);
  Serial.printf("R%6.1f P%6.1f Yr%6.1f\n", roll, pitch, yawRate);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== 階段 1: 陀螺儀測試 ===");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Serial.println("[1] I2C ready");

  mpu.initialize();
  Serial.print("[2] MPU6050: ");
  Serial.println(mpu.testConnection() ? "OK" : "FAIL");

  calibrateGyro();
  Serial.println("=== Ready ===\n");
}

void loop() {
  readAttitude();
  debugPrint();
}
