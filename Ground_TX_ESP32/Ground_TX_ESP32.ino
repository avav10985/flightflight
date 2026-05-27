// ============================================================
// 地面站發射器（38-pin ESP32 + LCD 2004A）
//
// 功能：
//   - Mode 2 雙搖桿控制（油門/yaw/pitch/roll）
//   - NRF24 雙向：送指令 + 收飛機遙測（ACK Payload）
//   - LCD 2004A 顯示飛機狀態
//   - 選單系統：用搖桿調 PID 參數，即時傳給飛機
//
// 硬體接線（38-pin ESP32）：
//   搖桿 左上下(油門) → GPIO 34
//   搖桿 左左右(yaw)  → GPIO 35
//   搖桿 右上下(pitch)→ GPIO 32
//   搖桿 右左右(roll) → GPIO 33
//   開關 武裝 SW1     → GPIO 25（按下對 GND）
//   開關 選單 SW2     → GPIO 26（按下對 GND）
//   NRF24 CE=4 CSN=5 SCK=18 MOSI=23 MISO=19  VCC=3V3 +100µF
//   LCD   SDA=21 SCL=22  VCC=5V
//
// 操作：
//   飛行模式：搖桿控飛機、LCD 顯示遙測
//   按 SW2 → 進選單 → 左搖桿上下選項目、左右調值 → 再按 SW2 離開
// ============================================================

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---- 腳位 ----
#define PIN_NRF_CE   4
#define PIN_NRF_CSN  5
#define J_THROTTLE   34   // 左搖桿 上下
#define J_YAW        35   // 左搖桿 左右
#define J_PITCH      32   // 右搖桿 上下
#define J_ROLL       33   // 右搖桿 左右
#define SW_ARM       25   // 武裝開關
#define SW_MENU      26   // 選單開關

// ---- 方向反轉（測試後不對就改）----
const bool REV_THROTTLE = true;
const bool REV_YAW      = false;
const bool REV_PITCH    = true;
const bool REV_ROLL     = true;

// ---- NRF24 ----
const uint64_t PIPE_OUT    = 0xABCDABCD71LL;
const uint8_t  NRF_CHANNEL = 100;

RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
LiquidCrystal_I2C lcd(0x27, 20, 4);   // 地址可能 0x27 或 0x3F

// ---- 通訊結構（與飛機端一致）----
struct Signal {
  byte throttle, pitch, roll, yaw, aux1, aux2;
  byte paramID;
  float paramVal;
};

struct Telemetry {
  int16_t  roll;
  int16_t  pitch;
  int16_t  yawRate;
  uint16_t battery_mV;
  uint8_t  status;
  uint8_t  reserved;
};

Signal    data;
Telemetry tele;
bool      teleOK = false;   // 有收到遙測

// ---- 選單 ----
enum Mode { FLIGHT, MENU };
Mode mode = FLIGHT;

// PID 本地副本（開機與飛機預設一致）
struct Param { const char* name; float val; float step; byte id; };
Param params[] = {
  { "Kp  ", 2.0f,  0.1f,  1 },
  { "Ki  ", 0.02f, 0.005f, 2 },
  { "Kd  ", 0.5f,  0.05f, 3 },
  { "KpY ", 1.5f,  0.1f,  4 },
  { "KiY ", 0.02f, 0.005f, 5 },
};
const int N_PARAM = 5;
int menuCursor = 0;

unsigned long lastLcdTime = 0;

// ============================================================
int centerMap(int raw, bool reverse) {
  raw = constrain(raw, 0, 4095);          // ESP32 12-bit
  int out = map(raw, 0, 4095, 0, 255);
  return reverse ? 255 - out : out;
}

// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(SW_ARM, INPUT_PULLUP);
  pinMode(SW_MENU, INPUT_PULLUP);
  analogReadResolution(12);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(" Drone Ground St.");
  lcd.setCursor(0, 1);
  lcd.print(" NRF24 init...");

  radio.begin();
  radio.openWritingPipe(PIPE_OUT);
  radio.setChannel(NRF_CHANNEL);
  radio.setAutoAck(true);        // 雙向必須 true
  radio.enableAckPayload();      // 收飛機遙測
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();         // 發射端

  delay(500);
  lcd.clear();
  Serial.println("Ground Station Ready");
}

// ============================================================
// 讀搖桿打包指令
// ============================================================
void readSticks() {
  data.throttle = centerMap(analogRead(J_THROTTLE), REV_THROTTLE);
  data.yaw      = centerMap(analogRead(J_YAW),      REV_YAW);
  data.pitch    = centerMap(analogRead(J_PITCH),    REV_PITCH);
  data.roll     = centerMap(analogRead(J_ROLL),     REV_ROLL);
  data.aux1 = (digitalRead(SW_ARM) == LOW) ? 1 : 0;
  data.aux2 = (digitalRead(SW_MENU) == LOW) ? 1 : 0;
}

// ============================================================
// 選單導航（左搖桿）
// ============================================================
void handleMenu() {
  static unsigned long lastMove = 0;
  if (millis() - lastMove < 250) return;   // 防連跳

  int yawRaw = centerMap(analogRead(J_YAW), false);     // 左右
  int thrRaw = centerMap(analogRead(J_THROTTLE), false); // 上下

  // 上下選項目
  if (thrRaw > 200) { menuCursor = (menuCursor + 1) % N_PARAM; lastMove = millis(); }
  if (thrRaw < 55)  { menuCursor = (menuCursor - 1 + N_PARAM) % N_PARAM; lastMove = millis(); }

  // 左右調值
  bool changed = false;
  if (yawRaw > 200) { params[menuCursor].val += params[menuCursor].step; changed = true; lastMove = millis(); }
  if (yawRaw < 55)  { params[menuCursor].val -= params[menuCursor].step; changed = true; lastMove = millis(); }
  if (params[menuCursor].val < 0) params[menuCursor].val = 0;

  // 改了就傳給飛機
  if (changed) {
    data.paramID  = params[menuCursor].id;
    data.paramVal = params[menuCursor].val;
    radio.write(&data, sizeof(Signal));
    data.paramID = 0;   // 傳完清除
  }
}

// ============================================================
// LCD 顯示
// ============================================================
void drawFlight() {
  // 第1行：武裝 + 電量
  lcd.setCursor(0, 0);
  lcd.print(tele.status & 0x01 ? "ARM " : "OFF ");
  lcd.printf("Bat:%4.1fV", tele.battery_mV / 1000.0);
  lcd.print(teleOK ? "  OK" : " ---");

  // 第2行：姿態
  lcd.setCursor(0, 1);
  lcd.printf("R:%5.1f P:%5.1f   ", tele.roll / 10.0, tele.pitch / 10.0);

  // 第3行：yaw + 油門
  lcd.setCursor(0, 2);
  lcd.printf("Yaw:%5.1f Thr:%3d%% ", tele.yawRate / 10.0,
             (int)(data.throttle * 100 / 255));

  // 第4行：提示
  lcd.setCursor(0, 3);
  lcd.print("SW2=Menu          ");
}

void drawMenu() {
  lcd.setCursor(0, 0);
  lcd.print("== PID 設定 ==     ");
  for (int i = 0; i < 3 && i < N_PARAM; i++) {
    int idx = i;   // 顯示前 3 個（簡化）
    lcd.setCursor(0, i + 1);
    lcd.print(idx == menuCursor ? ">" : " ");
    lcd.print(params[idx].name);
    lcd.printf("%6.3f       ", params[idx].val);
  }
}

// ============================================================
void loop() {
  readSticks();

  // SW2 邊緣切換選單
  static byte lastMenuSw = 0;
  byte menuSw = (digitalRead(SW_MENU) == LOW) ? 1 : 0;
  if (lastMenuSw == 0 && menuSw == 1) {
    mode = (mode == FLIGHT) ? MENU : FLIGHT;
    lcd.clear();
    delay(50);
  }
  lastMenuSw = menuSw;

  if (mode == MENU) {
    handleMenu();
  }

  // 送指令 + 收遙測（ACK payload）
  if (radio.write(&data, sizeof(Signal))) {
    if (radio.isAckPayloadAvailable()) {
      radio.read(&tele, sizeof(tele));
      teleOK = true;
    }
  } else {
    teleOK = false;   // 沒收到 ACK = 飛機沒回應
  }

  // LCD 更新（每 150ms，不要太快）
  if (millis() - lastLcdTime > 150) {
    lastLcdTime = millis();
    if (mode == FLIGHT) drawFlight();
    else                drawMenu();
  }

  delay(20);   // 約 50Hz
}
