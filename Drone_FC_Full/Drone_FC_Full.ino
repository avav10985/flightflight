// ============================================================
// Drone_FC_Full — 完整版飛控（最終版）
// MCU: LOLIN D32 (ESP32-WROOM-32)
//
// 2026-06-04 升級:從 Drone_FC_PID 移植所有功能,並預埋
// BMP280 / VL53L0X 旗標。這是要燒上飛機的最終版本。
//
// 功能(同 PID):
//   - MPU6050 姿態 + IMU NVS 校準系統(快速 / 完整,動作偵測)
//   - NRF24 雙向 ACK Payload 遙測(18 bytes Telemetry)
//   - PID 自穩 + X 型 4 馬達混控
//   - 9 模式封包(mode/flags/paramID/paramVal)
//   - 電池電壓監測
//   - GPS NEO-6M (UART2 @ 9600)
//   - 多 bit status 回傳手把(armed/cal/calfail/gpsfix)
//
// 預埋骨架(旗標關著,接好硬體再改 1):
//   - ENABLE_BMP280   氣壓計(定高用)
//   - ENABLE_VL53L0X  雷射 × 2(避障用)
//
// GPS 程式碼永久編譯進去(沒接 GPS 不會壞,gps_fix 永遠 false)。
//
// 序列埠指令：
//   kp 1.5     ← Roll/Pitch Kp
//   ki 0.05    ← Roll/Pitch Ki
//   kd 0.3     ← Roll/Pitch Kd
//   ypp 2.0    ← Yaw Kp
//   yi 0.01    ← Yaw Ki
//   p          ← 顯示目前所有 PID 值
//   cal        ← 快速 gyro-only 校準(0.3 秒,寫 NVS)
//   calfull    ← 完整校準 gyro + level(1.5 秒,需平面,寫 NVS)
//   calload    ← 從 NVS 載入校準
//   calclear   ← 清掉 NVS 校準資料
//   gps        ← 印當前 GPS lat/lon/sat
//   wp <lat> <lon> <alt>  ← 設目標 waypoint (Mode 02 用,目前只算數學不飛)
//   wpclear    ← 清掉 waypoint
//   wpinfo     ← 印目前 waypoint + 距離 + 方位
//   homenow    ← 把目前位置記為 home(RTH 起點)
//   home       ← 印 home 位置
//   yawreset   ← bodyYawEst 重置為 0(假設機頭朝北)
// ============================================================

// ====== 階段開關(感測器接好就改 1) ======
#define ENABLE_BMP280    0
#define ENABLE_VL53L0X   0
#define ENABLE_HMC5883   0   // GY-271 磁力計(2026-06-06 加,Mode 02 用)

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <Preferences.h>     // NVS 校準持久化
#include <TinyGPSPlus.h>     // GPS NMEA 解析(GY-NEO6MV2)

#if ENABLE_BMP280
#include <Adafruit_BMP280.h>
#endif

#if ENABLE_VL53L0X
#include <Adafruit_VL53L0X.h>
#endif

#if ENABLE_HMC5883
// GY-271 模組可能是 HMC5883L 或 QMC5883(中國複製版,接腳一樣但暫存器不同)
// HMC5883L 位址 0x1E,QMC5883 位址 0x0D — 兩個都試,自動偵測
#endif

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
// 2026-06-07 對調 GPIO 16/17:對應使用者實際接線
#define PIN_GPS_RX   17   // ESP32 接收 ← GPS TX
#define PIN_GPS_TX   16   // ESP32 傳送 → GPS RX

// ---- I²C 地址(感測器才用) ----
#if ENABLE_BMP280
#define ADDR_BMP280    0x76
#endif
#if ENABLE_VL53L0X
#define ADDR_TCA9548A  0x70
#endif
#if ENABLE_HMC5883
#define ADDR_HMC5883   0x1E   // Honeywell HMC5883L 原版
#define ADDR_QMC5883   0x0D   // QST QMC5883 中國複製版(常見)
#endif

// ---- NRF24 設定 ----
const uint64_t PIPE_IN     = 0xABCDABCD71LL;
const uint8_t  NRF_CHANNEL = 100;

// ---- 電池分壓 ----
const float BAT_DIVIDER = 4.0f;   // (30k+10k)/10k

// ====== 飛行 mode 編碼 ======
// 兩個 3 段開關組合,SW_A 當十位、SW_B 當個位(各 0/1/2),
// 9 個有效 byte 值(**不是 0~8 連續整數**):
//
//   00=安全/校準            01=手動自穩(目前唯一可飛)    02=GPS 自動到目標
//   10=語音控制(喚醒「啟動」) 11=PC 控制(USB 橋接)         12=預留
//   20=預留                  21=預留                       22=預留
//
// 計算公式:data.mode = SW_A_value * 10 + SW_B_value
// SW_A / SW_B 值定義:上=0、中=1、下=2

// 地面 → 飛機（指令 + 參數）
struct Signal {
  byte throttle, pitch, roll, yaw;
  byte mode;        // 兩位數編碼,見上面註解
  byte flags;       // bit0=完整校準觸發;其餘保留
  byte paramID;     // 0=無、1=Kp、2=Ki、3=Kd、4=Kp_y、5=Ki_y
  float paramVal;   // 參數值
};

// 飛機 → 地面（ACK payload 回傳遙測）
struct Telemetry {
  int16_t  roll;        // 角度 ×10
  int16_t  pitch;
  int16_t  yawRate;     // 角速度 ×10°/s
  uint16_t battery_mV;  // 電壓 mV
  uint8_t  status;      // 見下面 STATUS_* 位元定義
  uint8_t  satCount;    // GPS 衛星數
  int32_t  lat_e7;      // 緯度 × 10^7
  int32_t  lon_e7;      // 經度 × 10^7
};   // 18 bytes(NRF24 ACK payload max 32)

// Telemetry.status 位元定義
#define STATUS_ARMED         0x01   // bit0:已 armed
#define STATUS_CALIBRATING   0x02   // bit1:正在校準
#define STATUS_CAL_FAILED    0x04   // bit2:上次校準失敗(動作偵測拒寫)
#define STATUS_GPS_FIX       0x08   // bit3:GPS 有效定位

RF24        radio(PIN_NRF_CE, PIN_NRF_CSN);
MPU6050     mpu;
Servo       esc1, esc2, esc3, esc4;
TinyGPSPlus gps;

#if ENABLE_BMP280
Adafruit_BMP280 bmp;
bool          bmp280OK     = false;
float         baseAltitude = 0;
float         altitude_m   = 0;
#endif

#if ENABLE_VL53L0X
// 4 顆 VL53L0X 透過 TCA9548A 多工器:CH0 前中、CH1 前左、CH2 前右、CH3 底部
Adafruit_VL53L0X tofs[4];
bool          tofOK[4]  = { false, false, false, false };
uint16_t      tofDist[4] = { 0, 0, 0, 0 };       // 各方向距離(mm)
const uint16_t OBSTACLE_THRESHOLD_MM = 1800;     // 任一前雷射 < 1.8m → 避障
const uint16_t LANDING_TRIGGER_MM    = 300;      // 底部 < 30cm → 觸發接地
#endif

Signal        data;
Telemetry     tele;
float         batteryV    = 0;
bool          armed       = false;
bool          safetyReleased = false;   // 開機 LOCK：看到 mode 0 才解鎖
unsigned long lastRxTime  = 0;
unsigned long lastDbgTime = 0;
float         roll = 0, pitch = 0, yawRate = 0;
int16_t       rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
float         gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float         levelOffsetRoll = 0, levelOffsetPitch = 0;
Preferences   imuPrefs;
bool          calLoaded = false;
const int16_t MOTION_GYRO_THRESHOLD = 500;

// 校準狀態旗標(給 telemetry status bit 用)
bool          isCalibrating = false;
bool          lastCalFailed = false;

// GPS 狀態
double        gps_lat = 0;
double        gps_lon = 0;
uint8_t       gps_sat = 0;
bool          gps_fix = false;

// ---- PID 增益(測試用初始值;實際飛起來後再調) ----
// 之前在 PID 階段拘束測試摸出來的值,還沒實飛驗證最終值
float Kp_rp = 2.0f, Ki_rp = 0.02f, Kd_rp = 0.5f;
float Kp_y  = 1.5f, Ki_y  = 0.02f;

// ============================================================
// Mode 02 GPS 自動導航 — 純數學 + 狀態機骨架(2026-06-04)
// ----
// 目前狀態:純算 + Serial 印,**不送進 PID,飛機不會自動飛**
// 為什麼:modeFlyable() 還只接受 mode 01,撥到 mode 02 飛機不會 arm
// 還缺什麼才能真飛:
//   1. BMP280 開啟(目前 ENABLE_BMP280=0)→ 高度控制
//   2. VL53L0X 開啟 → 避障 + 底部降落
//   3. 磁力計 GY-271 → 絕對 yaw(短飛行可勉強用 gyro 積分)
//   4. modeFlyable() 加 m==2
//   5. pidControl() 在 mode 2 用 nav* 取代 data.roll/pitch/yaw
//   6. 完整飛測驗證
// ============================================================

struct Waypoint {
  double lat;
  double lon;
  float  alt;     // 相對 home 高度(米),0 = 同高
  bool   active;
};
Waypoint targetWP = {0, 0, 0, false};   // 目標座標
Waypoint homeWP   = {0, 0, 0, false};   // home(起飛位置,失聯 RTL 用)

enum NavState {
  NAV_IDLE,        // 沒目標 / mode != 2 / gps 沒鎖
  NAV_CRUISE,      // 飛去目標(距離 > brake 半徑)
  NAV_APPROACH,    // 接近(距離 < brake 半徑)
  NAV_LANDING,     // 降落中(尚未實作)
  NAV_LANDED,      // 完成
  NAV_FAILED       // 失敗(失聯 / 超時)
};
NavState navState = NAV_IDLE;

const float NAV_MAX_LEAN_DEG     = 15.0f;  // 最大傾角(自動模式不敢太激進)
const float NAV_BRAKE_RADIUS_M   = 3.0f;   // 接近多少米開始煞車
const float NAV_ARRIVE_RADIUS_M  = 1.0f;   // 到達判定半徑
const float NAV_LEAN_GAIN        = 5.0f;   // 距離 → 傾角的比例(度 / 米)

// 導航輸出(目前還沒送進 PID)
float navRollSet = 0, navPitchSet = 0, navYawRateSet = 0;
float distance_to_target = 0;       // 米
float bearing_to_target  = 0;       // 度(0~360,北=0 順時針)
float bodyYawEst         = 0;       // 機頭朝向 0~360°(沒磁力計時用 gyro 積分,有磁力計時用絕對讀值)

// ---- GY-271 磁力計狀態(ENABLE_HMC5883=1 才用)----
#if ENABLE_HMC5883
bool   magOK         = false;
uint8_t magAddr      = 0;     // 偵測到的 I²C 位址(0x1E 或 0x0D)
bool   magIsQMC      = false; // 是否為 QMC 版(暫存器不同)
float  magHeadingDeg = 0;     // 從磁力計讀到的絕對方位
// 硬鐵校準偏移(放磁鐵轉一圈量出來,先給 0 待校準)
float magOffsetX = 0, magOffsetY = 0, magOffsetZ = 0;
#endif

// ---- 球面距離 + 方位角數學 ----

const float EARTH_RADIUS_M = 6371000.0f;

// Haversine:兩個 lat/lon 之間的地表距離(米)
float haversineDistance(double lat1, double lon1, double lat2, double lon2) {
  double dLat = (lat2 - lat1) * DEG_TO_RAD;
  double dLon = (lon2 - lon1) * DEG_TO_RAD;
  double rlat1 = lat1 * DEG_TO_RAD;
  double rlat2 = lat2 * DEG_TO_RAD;
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(rlat1) * cos(rlat2) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return (float)(EARTH_RADIUS_M * c);
}

// 方位角:從點1看點2 的方向(0~360,北=0 順時針)
float bearingDegrees(double lat1, double lon1, double lat2, double lon2) {
  double rlat1 = lat1 * DEG_TO_RAD;
  double rlat2 = lat2 * DEG_TO_RAD;
  double dLon  = (lon2 - lon1) * DEG_TO_RAD;
  double y = sin(dLon) * cos(rlat2);
  double x = cos(rlat1) * sin(rlat2) - sin(rlat1) * cos(rlat2) * cos(dLon);
  double b = atan2(y, x) * RAD_TO_DEG;
  return (float)fmod(b + 360.0, 360.0);
}

// 角度差,規一化到 -180~+180
float angleDiffDeg(float target, float current) {
  float d = fmodf(target - current + 540.0f, 360.0f) - 180.0f;
  return d;
}

// ---- 導航主函式 ----

void navigateMode02() {
  // 沒目標 / 沒 GPS fix → 不動,輸出歸 0
  if (!targetWP.active || !gps_fix) {
    if (navState != NAV_IDLE) navState = NAV_IDLE;
    navRollSet = navPitchSet = navYawRateSet = 0;
    distance_to_target = 0;
    bearing_to_target = 0;
    return;
  }

  // 算距離 + 方位
  distance_to_target = haversineDistance(gps_lat, gps_lon, targetWP.lat, targetWP.lon);
  bearing_to_target  = bearingDegrees(gps_lat, gps_lon, targetWP.lat, targetWP.lon);

  // 狀態機
  switch (navState) {
    case NAV_IDLE:
      navState = NAV_CRUISE;
      break;
    case NAV_CRUISE:
      if (distance_to_target < NAV_BRAKE_RADIUS_M) navState = NAV_APPROACH;
      break;
    case NAV_APPROACH:
      if (distance_to_target < NAV_ARRIVE_RADIUS_M) navState = NAV_LANDING;
      else if (distance_to_target > NAV_BRAKE_RADIUS_M * 2) navState = NAV_CRUISE;
      break;
    case NAV_LANDING:
      // TODO 降落邏輯(等 BMP280 / 底部 VL53L0X 接上)
      break;
    default: break;
  }

  // 計算機體座標的 pitch / roll
  // 飛機朝向跟目標方位的差(機體要轉到面對目標)
  float headingError = angleDiffDeg(bearing_to_target, bodyYawEst);  // -180~+180

  // 距離 → 傾角(線性,有上限)
  float leanAngle = fminf(distance_to_target * NAV_LEAN_GAIN, NAV_MAX_LEAN_DEG);

  // 接近目標時縮小傾角(煞車)
  if (distance_to_target < NAV_BRAKE_RADIUS_M) {
    leanAngle *= distance_to_target / NAV_BRAKE_RADIUS_M;
  }

  // 轉成機體 pitch / roll
  float headingRad = headingError * DEG_TO_RAD;
  navPitchSet = -leanAngle * cosf(headingRad);   // 朝前傾 = 機頭朝下 = pitch 負
  navRollSet  = -leanAngle * sinf(headingRad);   // 朝右側傾 = roll 負

  // Yaw:暫時不動(目前沒絕對 yaw 參考,加磁力計後改為「轉到面對目標」)
  navYawRateSet = 0;
}

const char* navStateName(NavState s) {
  switch (s) {
    case NAV_IDLE:     return "IDLE";
    case NAV_CRUISE:   return "CRUISE";
    case NAV_APPROACH: return "APPROACH";
    case NAV_LANDING:  return "LANDING";
    case NAV_LANDED:   return "LANDED";
    case NAV_FAILED:   return "FAILED";
    default:           return "????";
  }
}

const float I_LIMIT     = 100.0f;
const float MAX_ANGLE   = 30.0f;
const float MAX_YAWRATE = 90.0f;

float i_roll = 0, i_pitch = 0, i_yaw = 0;
float rollOut = 0, pitchOut = 0, yawOut = 0;

// ============================================================
// TCA9548A 通道切換(VL53L0X 才會用)
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
// GY-271 磁力計(HMC5883L 或 QMC5883)
//
// 接線:VCC→3V3, GND→GND, SCL→GPIO 22, SDA→GPIO 21(跟 BMP280/MPU6050 共用 I²C)
//
// 校準:第一次接好 → Serial 送指令 `magcal` → 把飛機平放繞 360° 轉 2 圈
// → 程式記下 X/Y 最大最小值算 offset → 存 NVS。
// 沒校準前讀值會有大偏移(地磁 ~ 25~65µT,加上機體鐵磁 100~500µT 漂移)。
// ============================================================
#if ENABLE_HMC5883

// 偵測模組型號(HMC5883L 或 QMC5883)
bool magDetect() {
  // 先試 HMC5883L
  Wire.beginTransmission(ADDR_HMC5883);
  if (Wire.endTransmission() == 0) {
    magAddr   = ADDR_HMC5883;
    magIsQMC  = false;
    // HMC5883L 初始化:Config A=0x70(8 avg, 15Hz, normal),B=0x20(±1.3 gauss),Mode=0x00(continuous)
    Wire.beginTransmission(magAddr);
    Wire.write(0x00); Wire.write(0x70); Wire.write(0x20); Wire.write(0x00);
    Wire.endTransmission();
    return true;
  }
  // 再試 QMC5883(常見的中國複製版)
  Wire.beginTransmission(ADDR_QMC5883);
  if (Wire.endTransmission() == 0) {
    magAddr   = ADDR_QMC5883;
    magIsQMC  = true;
    // QMC5883:0x09 control reg = 0x1D(continuous, 200Hz, 2G, OSR=512)
    // 0x0B set/reset period = 0x01
    Wire.beginTransmission(magAddr); Wire.write(0x0B); Wire.write(0x01); Wire.endTransmission();
    Wire.beginTransmission(magAddr); Wire.write(0x09); Wire.write(0x1D); Wire.endTransmission();
    return true;
  }
  return false;
}

// 讀原始 XYZ + 算磁北方向(度,0~360 順時針)
bool magRead() {
  if (!magOK) return false;

  int16_t mx, my, mz;
  if (magIsQMC) {
    Wire.beginTransmission(magAddr);
    Wire.write(0x00);    // QMC 從 0x00 開始讀 X-low, X-high, Y-low, Y-high, Z-low, Z-high
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(magAddr, (uint8_t)6);
    if (Wire.available() < 6) return false;
    mx = Wire.read() | (Wire.read() << 8);
    my = Wire.read() | (Wire.read() << 8);
    mz = Wire.read() | (Wire.read() << 8);
  } else {
    Wire.beginTransmission(magAddr);
    Wire.write(0x03);    // HMC 從 0x03 開始讀 X-high, X-low, Z-high, Z-low, Y-high, Y-low
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(magAddr, (uint8_t)6);
    if (Wire.available() < 6) return false;
    mx = (Wire.read() << 8) | Wire.read();
    mz = (Wire.read() << 8) | Wire.read();   // HMC 順序是 X→Z→Y
    my = (Wire.read() << 8) | Wire.read();
  }

  // 套用硬鐵校準
  float fx = (float)mx - magOffsetX;
  float fy = (float)my - magOffsetY;

  // atan2 算磁北方向(度,0~360 順時針,北=0)
  float heading = atan2f(-fy, fx) * 180.0f / M_PI;   // -fy 因為 NED 慣例
  if (heading < 0) heading += 360.0f;
  magHeadingDeg = heading;
  return true;
}

#endif  // ENABLE_HMC5883

// ============================================================
// IMU 校準系統
//
// 兩種校準:
//   1. 快速 (Quick) - 0.3 秒,只校 gyro 漂移,不動 level
//      → 開機時做、進 mode 0 自動做、flags bit0 觸發
//   2. 完整 (Full)  - 1.5 秒,gyro + level 一起校
//      → 只在首次安裝/Serial 指令觸發,需要儘量水平的地面
//
// 動作偵測:校準採樣期間如果 gyro range 超過 MOTION_GYRO_THRESHOLD,
//          表示有動,拒寫 offset,保留原值。
//
// NVS:命名空間 "imu",key = gx_off/gy_off/gz_off/lvl_r/lvl_p/cal_ok
// ============================================================

bool loadCalibration() {
  imuPrefs.begin("imu", true);   // readonly
  bool ok = imuPrefs.getUChar("cal_ok", 0) == 1;
  if (ok) {
    gyroOffsetX     = imuPrefs.getFloat("gx_off", 0);
    gyroOffsetY     = imuPrefs.getFloat("gy_off", 0);
    gyroOffsetZ     = imuPrefs.getFloat("gz_off", 0);
    levelOffsetRoll  = imuPrefs.getFloat("lvl_r", 0);
    levelOffsetPitch = imuPrefs.getFloat("lvl_p", 0);
  }
  imuPrefs.end();
  return ok;
}

void saveCalibration() {
  imuPrefs.begin("imu", false);  // read-write
  imuPrefs.putFloat("gx_off", gyroOffsetX);
  imuPrefs.putFloat("gy_off", gyroOffsetY);
  imuPrefs.putFloat("gz_off", gyroOffsetZ);
  imuPrefs.putFloat("lvl_r", levelOffsetRoll);
  imuPrefs.putFloat("lvl_p", levelOffsetPitch);
  imuPrefs.putUChar("cal_ok", 1);
  imuPrefs.end();
}

void clearCalibration() {
  imuPrefs.begin("imu", false);
  imuPrefs.clear();
  imuPrefs.end();
  gyroOffsetX = gyroOffsetY = gyroOffsetZ = 0;
  levelOffsetRoll = levelOffsetPitch = 0;
  calLoaded = false;
  lastCalFailed = false;   // NVS 清乾淨,重置失敗旗標
}

// IMU 採樣 + 動作偵測
// 回傳 true = OK,false = 有動或失敗(outX 不要用)
bool sampleIMU(int N, bool includeLevel,
               float& outGx, float& outGy, float& outGz,
               float& outR,  float& outP) {
  long sx = 0, sy = 0, sz = 0;
  float sumRoll = 0, sumPitch = 0;
  int16_t minGx = 32767, maxGx = -32768;
  int16_t minGy = 32767, maxGy = -32768;
  int16_t minGz = 32767, maxGz = -32768;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < N; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sx += gx; sy += gy; sz += gz;
    if (gx < minGx) minGx = gx; if (gx > maxGx) maxGx = gx;
    if (gy < minGy) minGy = gy; if (gy > maxGy) maxGy = gy;
    if (gz < minGz) minGz = gz; if (gz > maxGz) maxGz = gz;
    if (includeLevel) {
      sumRoll  += atan2f(ax, az) * 57.2958f;
      sumPitch += atan2f(-ay, sqrtf((float)ax*ax + (float)az*az)) * 57.2958f;
    }
    delay(5);
  }
  if ((maxGx - minGx) > MOTION_GYRO_THRESHOLD ||
      (maxGy - minGy) > MOTION_GYRO_THRESHOLD ||
      (maxGz - minGz) > MOTION_GYRO_THRESHOLD) {
    return false;
  }
  outGx = sx / (float)N;
  outGy = sy / (float)N;
  outGz = sz / (float)N;
  if (includeLevel) {
    outR = sumRoll  / N;
    outP = sumPitch / N;
  }
  return true;
}

bool calibrateGyroQuick() {
  Serial.println("[*] 快速 gyro 校準 (0.3 秒,請靜止)...");
  isCalibrating = true;
  float newGx, newGy, newGz, dummy_r, dummy_p;
  if (!sampleIMU(60, false, newGx, newGy, newGz, dummy_r, dummy_p)) {
    Serial.println("[!] 校準失敗:採樣期間有動作,保留原 gyro offset");
    isCalibrating = false;
    lastCalFailed = true;
    return false;
  }
  gyroOffsetX = newGx;
  gyroOffsetY = newGy;
  gyroOffsetZ = newGz;
  roll = pitch = yawRate = 0;
  i_roll = i_pitch = i_yaw = 0;
  saveCalibration();
  Serial.printf("[*] gyro offset: gx=%.1f gy=%.1f gz=%.1f (寫 NVS)\n",
                gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  isCalibrating = false;
  lastCalFailed = false;
  return true;
}

bool calibrateFull() {
  Serial.println("[*] 完整校準 (gyro + level, 1.5 秒,放水平面靜止)...");
  isCalibrating = true;
  float newGx, newGy, newGz, newR, newP;
  if (!sampleIMU(300, true, newGx, newGy, newGz, newR, newP)) {
    Serial.println("[!] 校準失敗:採樣期間有動作,保留原 offset");
    isCalibrating = false;
    lastCalFailed = true;
    return false;
  }
  gyroOffsetX = newGx;
  gyroOffsetY = newGy;
  gyroOffsetZ = newGz;
  levelOffsetRoll  = newR;
  levelOffsetPitch = newP;
  roll = pitch = yawRate = 0;
  i_roll = i_pitch = i_yaw = 0;
  saveCalibration();
  Serial.printf("[*] gyro offset: gx=%.1f gy=%.1f gz=%.1f\n",
                gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  Serial.printf("[*] level offset: R=%.2f° P=%.2f° (寫 NVS)\n",
                levelOffsetRoll, levelOffsetPitch);

#if ENABLE_BMP280
  // 完整校準時順便重設高度基準(把目前高度當 0)
  if (bmp280OK) {
    baseAltitude = bmp.readAltitude(1013.25);
    Serial.printf("[*] 高度基準: %.2f m\n", baseAltitude);
  }
#endif

  isCalibrating = false;
  lastCalFailed = false;
  return true;
}

// 舊名相容
void calibrateGyro() { calibrateFull(); }

// ============================================================
// 感測器讀取
// ============================================================
void readGPS() {
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }
  if (gps.location.isValid() && gps.location.isUpdated()) {
    gps_lat = gps.location.lat();
    gps_lon = gps.location.lng();
    gps_fix = true;
  } else if (gps.location.age() > 5000) {
    // 5 秒沒新資料 → 失效
    gps_fix = false;
  }
  if (gps.satellites.isValid()) {
    gps_sat = gps.satellites.value();
  }
}

#if ENABLE_BMP280
void readBMP280() {
  if (!bmp280OK) return;
  float h = bmp.readAltitude(1013.25);
  altitude_m = h - baseAltitude;   // 相對高度
}
#endif

#if ENABLE_VL53L0X
void readToF() {
  for (int ch = 0; ch < 4; ch++) {
    if (!tofOK[ch]) continue;
    tcaSelect(ch);
    VL53L0X_RangingMeasurementData_t m;
    tofs[ch].rangingTest(&m, false);
    tofDist[ch] = (m.RangeStatus != 4) ? m.RangeMilliMeter : 0;
  }
}

// 避障 helper:三顆前雷射任一 < 1.8m → true
bool tofObstacleDetected() {
  for (int ch = 0; ch < 3; ch++) {                 // CH0/1/2 = 前面三顆
    if (tofOK[ch] && tofDist[ch] > 0 && tofDist[ch] < OBSTACLE_THRESHOLD_MM) return true;
  }
  return false;
}

// 找最 clear 的方向(供 Mode 02 避障旋轉決定):0=中、-1=左、+1=右
int tofClearestDir() {
  uint16_t center = tofOK[0] ? tofDist[0] : 0;
  uint16_t left   = tofOK[1] ? tofDist[1] : 0;
  uint16_t right  = tofOK[2] ? tofDist[2] : 0;
  if (center >= left && center >= right) return  0;
  if (left   >  right)                   return -1;
  return +1;
}

// 底部雷射:接近地面 → 觸發接地降落
bool tofLandingTrigger() {
  return tofOK[3] && tofDist[3] > 0 && tofDist[3] < LANDING_TRIGGER_MM;
}
#endif

void resetData() {
  data.throttle = 0;
  data.pitch = data.roll = data.yaw = 127;
  data.mode  = 0;          // 失聯 → 安全模式
  data.flags = 0;
  data.paramID = 0;
  data.paramVal = 0;
}

void readBattery() {
  int raw = analogRead(PIN_BAT_ADC);
  batteryV = raw * (3.3f / 4095.0f) * BAT_DIVIDER;
}

void buildTelemetry() {
  tele.roll       = (int16_t)(roll * 10);
  tele.pitch      = (int16_t)(pitch * 10);
  tele.yawRate    = (int16_t)(yawRate * 10);
  tele.battery_mV = (uint16_t)(batteryV * 1000);

  uint8_t st = 0;
  if (armed)         st |= STATUS_ARMED;
  if (isCalibrating) st |= STATUS_CALIBRATING;
  if (lastCalFailed) st |= STATUS_CAL_FAILED;
  if (gps_fix)       st |= STATUS_GPS_FIX;
  tele.status     = st;

  tele.satCount   = gps_sat;
  tele.lat_e7     = gps_fix ? (int32_t)(gps_lat * 1e7) : 0;
  tele.lon_e7     = gps_fix ? (int32_t)(gps_lon * 1e7) : 0;
}

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

  // Yaw 絕對角度估計:有磁力計用 complementary filter 融合(98% gyro 積分 + 2% 磁力計絕對)
  // 沒磁力計就純 gyro 積分,給 Mode 02 GPS 導航算「飛機朝向跟目標方位的差」用
  bodyYawEst += yawRate * dt;
  if (bodyYawEst >= 360.0f) bodyYawEst -= 360.0f;
  if (bodyYawEst < 0.0f)    bodyYawEst += 360.0f;

#if ENABLE_HMC5883
  // 磁力計低頻校正,消除 gyro 漂移
  static unsigned long lastMagRead = 0;
  if (magOK && millis() - lastMagRead > 50) {   // 20 Hz 讀磁力計就夠
    lastMagRead = millis();
    if (magRead()) {
      // complementary filter:2% 用磁力計絕對讀值拉回
      float diff = magHeadingDeg - bodyYawEst;
      // 處理 360 wrap-around 找最短角差
      if (diff > 180.0f)  diff -= 360.0f;
      if (diff < -180.0f) diff += 360.0f;
      bodyYawEst += diff * 0.02f;
      if (bodyYawEst >= 360.0f) bodyYawEst -= 360.0f;
      if (bodyYawEst < 0.0f)    bodyYawEst += 360.0f;
    }
  }
#endif
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

// 目前只有 mode 01(手動自穩,byte 值 1)已實作可飛;其他 mode 一律拒武裝。
// 未來:Mode 02(GPS)、10(語音)、11(PC)實作後要加進條件式,例如:
//   return m == 1 || m == 2 || m == 10 || m == 11;
inline bool modeFlyable(byte m) { return m == 1; }

void disarm() {
  if (armed) { armed = false; i_roll = i_pitch = i_yaw = 0; }
}

void checkArm() {
  // 看到 mode 0（校準/安全）一次 → 解除開機 LOCK
  if (data.mode == 0) safetyReleased = true;

  // 未解鎖 / 安全模式 / 非可飛模式 → 強制 disarm
  if (!safetyReleased || data.mode == 0 || !modeFlyable(data.mode)) {
    disarm();
    return;
  }

  // 已解鎖 + 可飛模式 + 油門在底 → arm
  if (!armed && data.throttle < 20) armed = true;
}

// 校準觸發:
//   1. 進 mode 0 的瞬間(從非 0 → 0)+ disarmed + 油門 0 → 自動「快速 gyro」
//   2. flags bit0 邊緣(0→1)+ disarmed + 油門 0 + mode 0 → 觸發「完整校準」
void checkCalibrationTrigger() {
  static byte lastMode = 0xFF;
  static byte lastCalFlag = 0;

  // 1. 進 mode 0 那一瞬間 → 自動快速 gyro
  if (data.mode == 0 && lastMode != 0 &&
      !armed && data.throttle < 5) {
    Serial.println("\n[!] 進 mode 0 觸發快速 gyro 校準");
    calibrateGyroQuick();
  }

  // 2. flags bit0 邊緣 → 完整校準
  byte calFlag = data.flags & 0x01;
  if (!armed && data.mode == 0 && data.throttle < 5 &&
      lastCalFlag == 0 && calFlag == 1) {
    Serial.println("\n[!] flags bit0 觸發完整校準");
    calibrateFull();
  }

  lastMode = data.mode;
  lastCalFlag = calFlag;
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
    calibrateGyroQuick();
  } else if (s == "calfull") {
    calibrateFull();
  } else if (s == "calload") {
    if (loadCalibration()) {
      calLoaded = true;
      Serial.printf("[*] 從 NVS 載入 OK: gyro(%.1f,%.1f,%.1f) level(%.2f°,%.2f°)\n",
                    gyroOffsetX, gyroOffsetY, gyroOffsetZ,
                    levelOffsetRoll, levelOffsetPitch);
    } else {
      Serial.println("[!] NVS 無校準資料");
    }
  } else if (s == "calclear") {
    clearCalibration();
    Serial.println("[!] NVS 已清,offset 全部歸 0");
  } else if (s == "gps") {
    if (gps_fix) {
      Serial.printf(">> GPS: lat=%.7f lon=%.7f sat=%d age=%lums\n",
                    gps_lat, gps_lon, gps_sat, (unsigned long)gps.location.age());
    } else {
      Serial.printf(">> GPS: no fix, sat=%d (NEO-6M 冷啟動 30~120 秒,天線朝天)\n",
                    gps_sat);
    }
  } else if (s.startsWith("wp ")) {
    // 格式:wp <lat> <lon> <alt>(空白分隔)
    String args = s.substring(3);
    int sp1 = args.indexOf(' ');
    int sp2 = args.indexOf(' ', sp1 + 1);
    if (sp1 > 0 && sp2 > 0) {
      targetWP.lat = args.substring(0, sp1).toDouble();
      targetWP.lon = args.substring(sp1 + 1, sp2).toDouble();
      targetWP.alt = args.substring(sp2 + 1).toFloat();
      targetWP.active = true;
      navState = NAV_IDLE;   // 重置狀態機,下次 navigateMode02 進 CRUISE
      Serial.printf(">> 目標 WP 已設:(%.7f, %.7f, alt=%.1fm)\n",
                    targetWP.lat, targetWP.lon, targetWP.alt);
    } else {
      Serial.println(">> 用法:wp <lat> <lon> <alt>  例:wp 25.0331 121.5654 10");
    }
  } else if (s == "wpclear") {
    targetWP.active = false;
    navState = NAV_IDLE;
    Serial.println(">> 目標 WP 清掉");
  } else if (s == "wpinfo") {
    if (!targetWP.active) {
      Serial.println(">> 沒設 WP(用 'wp <lat> <lon> <alt>' 設)");
    } else if (!gps_fix) {
      Serial.printf(">> WP=(%.7f, %.7f, %.1fm),但 GPS 沒鎖 → 無法算距離\n",
                    targetWP.lat, targetWP.lon, targetWP.alt);
    } else {
      Serial.printf(">> WP=(%.7f, %.7f, %.1fm) | 距離 %.1fm | 方位 %.1f° | yaw估計 %.1f° | nav %s\n",
                    targetWP.lat, targetWP.lon, targetWP.alt,
                    distance_to_target, bearing_to_target, bodyYawEst,
                    navStateName(navState));
    }
  } else if (s == "homenow") {
    if (gps_fix) {
      homeWP.lat = gps_lat;
      homeWP.lon = gps_lon;
      homeWP.alt = 0;
      homeWP.active = true;
      Serial.printf(">> Home 記在當前位置:(%.7f, %.7f)\n", homeWP.lat, homeWP.lon);
    } else {
      Serial.println(">> GPS 沒鎖,沒辦法記 home");
    }
  } else if (s == "home") {
    if (homeWP.active) {
      Serial.printf(">> Home:(%.7f, %.7f)\n", homeWP.lat, homeWP.lon);
    } else {
      Serial.println(">> Home 還沒記(用 'homenow' 記當前位置)");
    }
  } else if (s == "yawreset") {
    bodyYawEst = 0;
    Serial.println(">> bodyYawEst 重置為 0(假設現在機頭朝北)");
#if ENABLE_HMC5883
  } else if (s == "magshow") {
    if (!magOK) { Serial.println(">> 磁力計未偵測到"); }
    else {
      magRead();
      Serial.printf(">> mag heading = %.1f° (offset X=%.0f Y=%.0f Z=%.0f)\n",
                    magHeadingDeg, magOffsetX, magOffsetY, magOffsetZ);
    }
  } else if (s == "magcal") {
    if (!magOK) { Serial.println(">> 磁力計未偵測到"); }
    else {
      Serial.println(">> 磁力計校準開始 10 秒,把飛機平放繞 360° 轉 2 圈...");
      float xMin = 32767, xMax = -32768;
      float yMin = 32767, yMax = -32768;
      unsigned long endMs = millis() + 10000;
      int samples = 0;
      while (millis() < endMs) {
        int16_t mx, my, mz;
        Wire.beginTransmission(magAddr);
        Wire.write(magIsQMC ? 0x00 : 0x03);
        if (Wire.endTransmission(false) == 0) {
          Wire.requestFrom(magAddr, (uint8_t)6);
          if (Wire.available() >= 6) {
            if (magIsQMC) {
              mx = Wire.read() | (Wire.read() << 8);
              my = Wire.read() | (Wire.read() << 8);
            } else {
              mx = (Wire.read() << 8) | Wire.read();
              Wire.read(); Wire.read();          // 跳過 Z
              my = (Wire.read() << 8) | Wire.read();
            }
            if (mx < xMin) xMin = mx; if (mx > xMax) xMax = mx;
            if (my < yMin) yMin = my; if (my > yMax) yMax = my;
            samples++;
          }
        }
        delay(50);
      }
      magOffsetX = (xMin + xMax) / 2.0f;
      magOffsetY = (yMin + yMax) / 2.0f;
      Serial.printf(">> 校準完成,%d samples,offset X=%.0f Y=%.0f\n", samples, magOffsetX, magOffsetY);
      Serial.println(">> (TODO: NVS 持久化還沒寫,重開機要重新校準)");
    }
#endif
  } else {
    Serial.println(">> 指令:kp/ki/kd/ypp/yi <值>、p、cal、calfull、calload、calclear、gps、");
    Serial.println(">>      wp <lat> <lon> <alt>、wpclear、wpinfo、homenow、home、yawreset");
#if ENABLE_HMC5883
    Serial.println(">>      magcal、magshow");
#endif
  }
}

void debugPrint() {
  if (millis() - lastDbgTime < 200) return;
  lastDbgTime = millis();

  Serial.printf("R%6.1f P%6.1f Yr%6.1f | M%02d lock%d arm%d Thr%3d | Bat%.2fV | GPS sat%d %s",
                roll, pitch, yawRate,
                data.mode, safetyReleased, armed, data.throttle, batteryV,
                gps_sat, gps_fix ? "FIX" : "---");

#if ENABLE_BMP280
  if (bmp280OK) Serial.printf(" | Alt %.2fm", altitude_m);
#endif

#if ENABLE_VL53L0X
  Serial.printf(" | F%4dmm B%4dmm", distFront_mm, distBack_mm);
#endif

  // Mode 02 GPS 導航狀態(有目標才印)
  if (targetWP.active) {
    Serial.printf(" | NAV %s d=%.1fm brg=%.0f° yawEst=%.0f° navR=%+.1f navP=%+.1f",
                  navStateName(navState), distance_to_target, bearing_to_target,
                  bodyYawEst, navRollSet, navPitchSet);
  }

  Serial.printf(" | PID r%+5.0f p%+5.0f y%+5.0f\n",
                rollOut, pitchOut, yawOut);
}

// ============================================================
// 感測器初始化
// ============================================================
#if ENABLE_BMP280
void initBMP280() {
  if (bmp.begin(ADDR_BMP280)) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    bmp280OK = true;
    delay(100);
    baseAltitude = bmp.readAltitude(1013.25);
    Serial.printf("[+] BMP280: OK,baseAltitude=%.2fm\n", baseAltitude);
  } else {
    Serial.println("[!] BMP280: FAIL");
  }
}
#endif

#if ENABLE_VL53L0X
// 4 顆 VL53L0X 透過 TCA9548A I²C 多工器(避免位址衝突)
// CH0 = 前中、CH1 = 前左、CH2 = 前右、CH3 = 底部(降落輔助)
const char* TOF_NAMES[4] = { "前中", "前左", "前右", "底部" };

void initToF() {
  for (int ch = 0; ch < 4; ch++) {
    tcaSelect(ch);
    delay(50);
    if (tofs[ch].begin(0x29)) {
      tofOK[ch] = true;
      Serial.printf("[+] VL53L0X #%d (%s): OK\n", ch, TOF_NAMES[ch]);
    } else {
      tofOK[ch] = false;
      Serial.printf("[!] VL53L0X #%d (%s): FAIL\n", ch, TOF_NAMES[ch]);
    }
  }
}

// 讀單顆 VL53L0X 距離(mm)。回 -1 = 超出範圍或讀失敗
int16_t tofRead(uint8_t ch) {
  if (ch > 3 || !tofOK[ch]) return -1;
  tcaSelect(ch);
  VL53L0X_RangingMeasurementData_t m;
  tofs[ch].rangingTest(&m, false);
  if (m.RangeStatus != 4) return m.RangeMilliMeter;
  return -1;
}
#endif

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Drone_FC_Full (最終版) ===");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Serial.println("[1] I2C ready");

  Serial2.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.println("[1b] GPS UART2 @ 9600 啟動 (NEO-6M 冷啟動 30~120 秒)");

  mpu.initialize();
  Serial.print("[2] MPU6050: ");
  Serial.println(mpu.testConnection() ? "OK" : "FAIL");

#if ENABLE_BMP280
  initBMP280();
#endif

#if ENABLE_VL53L0X
  initToF();
#endif

#if ENABLE_HMC5883
  if (magDetect()) {
    magOK = true;
    Serial.printf("[*] 磁力計 OK,位址 0x%02X(%s),記得執行 magcal 校準\n",
                  magAddr, magIsQMC ? "QMC5883" : "HMC5883L");
  } else {
    Serial.println("[!] 磁力計沒偵測到,Mode 02 仍可飛但 yaw 會漂");
  }
#endif

  // 校準:先試 NVS 載入,沒有再做首次完整校準
  if (loadCalibration()) {
    calLoaded = true;
    Serial.println("[*] 從 NVS 載入校準資料:");
    Serial.printf("    gyro: gx=%.1f gy=%.1f gz=%.1f\n",
                  gyroOffsetX, gyroOffsetY, gyroOffsetZ);
    Serial.printf("    level: R=%.2f° P=%.2f°\n",
                  levelOffsetRoll, levelOffsetPitch);
    // 開機 quick gyro 重校(修正溫度漂移),level 用 NVS 那筆
    calibrateGyroQuick();
  } else {
    Serial.println("[!] NVS 無校準資料,做首次完整校準");
    calibrateFull();
  }

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
  Serial.println("=== Ready (LOCKED：先把模式開關切到 00 解鎖) ===\n");
}

void loop() {
  recvData();
  failsafe();
  checkArm();
  checkCalibrationTrigger();
  readAttitude();
  readBattery();
  readGPS();
#if ENABLE_BMP280
  readBMP280();
#endif
#if ENABLE_VL53L0X
  readToF();
#endif
  // Mode 02 GPS 導航(只計算 + 印,沒送進 PID,飛機不會自動飛)
  // 未來啟用時:在 pidControl 裡偵測 mode==2 → 用 navRollSet 等取代 data.roll/pitch
  navigateMode02();
  pidControl();
  writeMotors();
  debugPrint();
  parseSerial();
}
