// ============================================================
// 地面站發射器 V2-A:ESP32-S3-N16R8 + MSP2806 2.8" SPI TFT(直立)
//
// 功能:
//   - 油門(左搖桿上下,拆彈簧)/ pitch / roll
//   - yaw 由肩鍵 L / R(按著 = 固定 ±60°/s)
//   - 兩個 3 段開關組 9 模式,**mode = A×10 + B 兩位數編碼**:
//     00=安全/校準、01=手動、02=GPS、10=語音、11=PC、12/20/21/22=預留
//   - NRF24 雙向 ACK(送指令 + 收遙測)
//   - MSP2806 2.8" SPI ILI9341 TFT 240×320 直立顯示模式/姿態/狀態
//     (跟 NRF24/SD 共用 SPI 匯流排,只佔 CS=48 + DC=21 兩隻獨立腳)
//   - mode 0 按「OK」進選單,4 鈕電阻階梯調 PID
//   - SD 卡與 NRF24 共用 SPI(V2-A 只 mount,未寫入)
//
// 硬體接線見 手把接線總表_V2.md。
//
// 開發板:ESP32S3 Dev Module;USB CDC On Boot=Enabled;
//          Flash=16MB;PSRAM=OPI PSRAM;Partition=16M with OTA。
//
// 函式庫:LovyanGFX(per-sketch config,下面 LGFX class)、RF24、SD、SPI。
//
// 飛機端 Drone_FC_PID 已改成新封包(mode/flags/paramID/paramVal),
// 兩端必須同時重燒,封包對不上會收到亂碼。
//
// V2-A 變更歷程:
//   - 原本用 MAR2406 並列 8-bit TFT,佔 11 隻 GPIO
//   - 2026-06-02 換成 MSP2806 SPI TFT,只佔 2 隻 + 共用 SPI,省 9 隻
//     GPIO 11~18 全部釋出給 V2-B 或其他擴充
// ============================================================

#include <SPI.h>
#include <RF24.h>
#include <SD.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// ============================================================
// LovyanGFX：MSP2806 2.8" SPI ILI9341
// 配置照 tyty/tyty.ino 試燒成功的設定為基準,只改我們共用 SPI 必要的部分
// 直立 240×320
// ============================================================
class LGFX : public lgfx::LGFX_Device {
  lgfx::Bus_SPI       _bus_instance;
  lgfx::Panel_ILI9341 _panel_instance;
public:
  LGFX() {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 40000000;
      cfg.freq_read   = 16000000;
      cfg.pin_sclk    = 38;                // 跟 NRF24/SD 共用(設計如此)
      cfg.pin_mosi    = 39;                // 同上
      cfg.pin_miso    = -1;                // 不接(tyty 也是 -1,我們也不用讀)
      cfg.pin_dc      = 21;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs        = 48;
      cfg.pin_rst       = 16;              // GPIO 16 主動驅動(同 tyty,沒上拉電阻)
      cfg.pin_busy      = -1;
      cfg.memory_width  = 240;
      cfg.memory_height = 320;
      cfg.panel_width   = 240;
      cfg.panel_height  = 320;
      cfg.readable      = false;
      cfg.invert        = false;
      cfg.rgb_order     = false;
      cfg.dlen_16bit    = false;
      cfg.bus_shared    = true;            // ★ 跟 NRF24/SD 共用 SPI 必開(tyty 是 false 因獨立)
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};
LGFX tft;

// ---- 腳位（V2-A 已定案,見 手把接線總表_V2.md）----
// 2026-06-04 GPIO 2 ↔ 10 對調:左搖桿訊號集中右側、右搖桿集中左側
#define J_THROTTLE   1    // 右#4:左搖桿 Y(油門,拆彈簧)
#define J_LEFT_X     2    // 右#5:左搖桿 X(可選 yaw,**目前只定義不讀**)
#define J_ROLL       4    // 左#4:右搖桿 X(roll)
#define J_PITCH      10   // 左#16:右搖桿 Y(pitch)
#define SW_A         5
#define SW_B         6
#define MENU_BTN     7
#define SHOULDER_L   8
#define SHOULDER_R   9

// TFT LED:MSP2806 的 LED 腳沒內接 VCC,**直接外接 B 板 3V3 軌常亮**(不走 GPIO/PWM)
// GPIO 15 因此回歸「真正釋出」可用腳
#define PIN_NRF_CE   41
#define PIN_NRF_CSN  42
#define PIN_SPI_SCK  38
#define PIN_SPI_MOSI 39
#define PIN_SPI_MISO 40
#define PIN_SD_CS    0   // BOOT 腳，外部 10k 上拉到 3V3，開機 HIGH 安全

// ---- 方向反轉（測試後不對就改）----
const bool REV_THROTTLE = true;
const bool REV_PITCH    = true;
const bool REV_ROLL     = true;
// yaw 方向若相反，把 SHOULDER_L / SHOULDER_R 兩腳對調即可

// ---- NRF24 ----
const uint64_t PIPE_OUT    = 0xABCDABCD71LL;
const uint8_t  NRF_CHANNEL = 100;

RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);

// ---- 通訊結構（與飛機端一致）----
// mode 兩位數編碼,9 個值:00/01/02/10/11/12/20/21/22(見上面 header)
struct Signal {
  byte throttle, pitch, roll, yaw;
  byte mode;        // SW_A×10 + SW_B(各 0/1/2),例:01=A上+B中
  byte flags;       // bit0=完整校準觸發
  byte paramID;     // 0=無、1=Kp、2=Ki、3=Kd、4=Kp_y、5=Ki_y
  float paramVal;
};

// 飛機 → 手把(NRF24 ACK Payload,**必須跟飛機端 Telemetry struct 完全一致**)
struct Telemetry {
  int16_t  roll;        // 角度 ×10
  int16_t  pitch;
  int16_t  yawRate;     // 角速度 ×10°/s
  uint16_t battery_mV;
  uint8_t  status;      // 多 bit:見下面 STATUS_*
  uint8_t  satCount;    // GPS 衛星數
  int32_t  lat_e7;      // 緯度 ×10^7
  int32_t  lon_e7;      // 經度 ×10^7
};   // 18 bytes(NRF24 ACK Payload 上限 32)

// status 位元定義(跟飛機端一致)
#define STATUS_ARMED         0x01   // bit0:已 armed
#define STATUS_CALIBRATING   0x02   // bit1:正在校準
#define STATUS_CAL_FAILED    0x04   // bit2:上次校準失敗
#define STATUS_GPS_FIX       0x08   // bit3:GPS 有效定位

Signal    data;
Telemetry tele;
bool      teleOK = false;
bool      sdOK   = false;

// ---- 模式名稱(中文化,2026-06-06)----
// 兩位數 mode 編碼,值為 0/1/2/10/11/12/20/21/22(9 個離散值)
const char* getModeName(byte m) {
  switch (m) {
    case 0:  return "安全校準";   // 安全 / 校準 / 設定 hub
    case 1:  return "手動自穩";   // 手動自穩(目前唯一可飛)
    case 2:  return "GPS導航";    // GPS 自動到目標 + 避障 + 降落
    case 10: return "語音控制";   // 喚醒詞「啟動」
    case 11: return "電腦控制";   // PC 透過 USB 控制
    case 12: case 20:
    case 21: case 22:    return "保留";    // 預留
    default:             return "未知";    // 無效值
  }
}

// 哪些 mode 已實作(影響 TFT「MODE NOT IMPLEMENTED」提示)。
// 目前只有 00(安全/設定畫面)跟 01(手動飛行)有完整 UI 跟邏輯。
// 將來 02 / 10 / 11 加進來後也要更新這個 list。
bool modeImplemented(byte m) { return m == 0 || m == 1; }

// ---- 選單鈕 ----
enum { BTN_NONE, BTN_PLUS, BTN_MINUS, BTN_OK, BTN_BACK };

// ---- UI 狀態 ----
enum UiState { UI_FLIGHT, UI_MENU };
UiState ui = UI_FLIGHT;

// PID 本地副本（開機與飛機預設一致）
struct Param { const char* name; float val; float step; byte id; };
Param params[] = {
  { "Kp ", 2.0f,  0.1f,   1 },
  { "Ki ", 0.02f, 0.005f, 2 },
  { "Kd ", 0.5f,  0.05f,  3 },
  { "KpY", 1.5f,  0.1f,   4 },
  { "KiY", 0.02f, 0.005f, 5 },
};
const int N_PARAM = 5;
int menuCursor = 0;

unsigned long lastDrawTime = 0;

// ============================================================
// 公用函式
// ============================================================
int centerMap(int raw, bool reverse) {
  raw = constrain(raw, 0, 4095);
  int out = map(raw, 0, 4095, 0, 255);
  return reverse ? 255 - out : out;
}

byte readSwitch3(int pin) {
  int v = analogRead(pin);
  // 2026-06-06 試燒發現實體開關上下與 ADC 讀值相反,把回傳值對調
  // 撥到上 = ADC 低電位 → 回傳 0;撥到下 = ADC 高電位 → 回傳 2
  if (v > 2730) return 2;   // 下
  if (v < 1365) return 0;   // 上
  return 1;                  // 中(開路分壓中點)
}

int readMenuBtn() {
  int v = analogRead(MENU_BTN);
  if (v > 3250) return BTN_PLUS;
  if (v > 2415) return BTN_MINUS;
  if (v > 1665) return BTN_OK;
  if (v > 640)  return BTN_BACK;
  return BTN_NONE;
}

void readInputs(byte mode) {
  data.throttle = centerMap(analogRead(J_THROTTLE), REV_THROTTLE);
  data.pitch    = centerMap(analogRead(J_PITCH),    REV_PITCH);
  data.roll     = centerMap(analogRead(J_ROLL),     REV_ROLL);

  // yaw 由肩鍵：L→42、R→212、都沒按/同時按→127（對應 ±60°/s）
  bool yawL = (digitalRead(SHOULDER_L) == LOW);
  bool yawR = (digitalRead(SHOULDER_R) == LOW);
  data.yaw = (yawL && !yawR) ? 42 : (yawR && !yawL) ? 212 : 127;

  data.mode = mode;

  // 進入 mode 0 → 送一次校準旗標（持續 600ms 確保飛機收到 0→1 邊緣）
  static byte lastMode = 255;
  static unsigned long calHoldUntil = 0;
  if (mode == 0 && lastMode != 0) calHoldUntil = millis() + 600;
  lastMode = mode;
  data.flags = (millis() < calHoldUntil) ? 0x01 : 0x00;
}

void handleMenuCursor() {
  // 2026-06-06 試燒發現搖桿中位偏離 127 時游標會自己跳,死區從 55/200 改成 15/240
  // (搖桿要推到接近底才動游標,避免靜止時誤觸)
  static unsigned long lastMove = 0;
  if (millis() - lastMove < 250) return;
  int p = centerMap(analogRead(J_PITCH), false);
  if (p > 240) { menuCursor = (menuCursor + 1) % N_PARAM;            lastMove = millis(); }
  if (p < 15)  { menuCursor = (menuCursor - 1 + N_PARAM) % N_PARAM;  lastMove = millis(); }
}

void handleMenuButton(int btn) {
  if (btn == BTN_BACK) { ui = UI_FLIGHT; tft.fillScreen(TFT_BLACK); return; }
  if (btn == BTN_PLUS)  params[menuCursor].val += params[menuCursor].step;
  if (btn == BTN_MINUS) params[menuCursor].val -= params[menuCursor].step;
  if (params[menuCursor].val < 0) params[menuCursor].val = 0;
  // +/- /OK → 排好參數，交給主迴圈 write 帶給飛機
  data.paramID  = params[menuCursor].id;
  data.paramVal = params[menuCursor].val;
}

// ============================================================
// TFT UI(直立 240×320):static 一次畫底色 + dynamic 文字覆寫
// ============================================================
void drawFlightStatic() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0,   0, 240, 32, TFT_NAVY);     // 頂部標題列
  tft.fillRect(0, 280, 240, 40, TFT_DARKGREY); // 底部狀態列
}

void drawFlightDynamic(byte mode) {
  char buf[40];
  bool armed = tele.status & STATUS_ARMED;

  // 頂部:模式 + 武裝狀態
  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.setCursor(5, 4);
  snprintf(buf, sizeof(buf), "M%02d %s", mode, getModeName(mode));
  tft.print(buf);

  tft.setTextColor(armed ? TFT_RED : TFT_LIGHTGREY, TFT_NAVY);
  tft.setCursor(180, 4);
  tft.print(armed ? "已武" : "安全");

  // 主體:已實作/未實作 切換時清一次
  static byte lastBlock = 255;
  byte block = modeImplemented(mode) ? 1 : 0;
  if (block != lastBlock) {
    tft.fillRect(0, 34, 240, 244, TFT_BLACK);
    lastBlock = block;
  }

  if (!modeImplemented(mode)) {
    tft.setFont(&fonts::efontTW_24);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(40, 140);
    tft.print("此模式");
    tft.setCursor(40, 175);
    tft.print("未實作");
  } else {
    // 直立排版,三個姿態各佔一段
    tft.setFont(&fonts::efontTW_24);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    // 翻滾
    tft.setCursor(10, 50);
    tft.print("翻滾");
    tft.setCursor(10, 80);
    snprintf(buf, sizeof(buf), "%+7.1f", tele.roll / 10.0);
    tft.print(buf);
    tft.setCursor(180, 80);
    tft.print("°");
    // 俯仰
    tft.setCursor(10, 130);
    tft.print("俯仰");
    tft.setCursor(10, 160);
    snprintf(buf, sizeof(buf), "%+7.1f", tele.pitch / 10.0);
    tft.print(buf);
    tft.setCursor(180, 160);
    tft.print("°");
    // 偏航速率
    tft.setCursor(10, 210);
    tft.print("偏航");
    tft.setCursor(10, 240);
    snprintf(buf, sizeof(buf), "%+7.1f", tele.yawRate / 10.0);
    tft.print(buf);
    tft.setCursor(170, 240);
    tft.print("°/秒");
  }

  // 底部狀態列(兩行)
  tft.setFont(&fonts::efontTW_14);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  snprintf(buf, sizeof(buf), "油門 %3d%%   電池 %4.1fV",
           (int)(data.throttle * 100 / 255),
           tele.battery_mV / 1000.0);
  tft.setCursor(5, 285);
  tft.print(buf);
  snprintf(buf, sizeof(buf), "%s  %s",
           teleOK ? "連線" : "斷線",
           sdOK ? "SD好" : "無SD");
  tft.setCursor(5, 302);
  tft.print(buf);
}

void drawMenuStatic() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0,   0, 240, 32, TFT_PURPLE);
  tft.fillRect(0, 280, 240, 40, TFT_DARKGREY);

  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(TFT_WHITE, TFT_PURPLE);
  tft.setCursor(40, 4);
  tft.print("═ PID 設定 ═");

  tft.setFont(&fonts::efontTW_14);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setCursor(5, 285);
  tft.print("右搖桿:游標  +/-:調值");
  tft.setCursor(5, 302);
  tft.print("OK:送出    返回:離開");
}

void drawMenuDynamic() {
  // 只重畫狀態變過的列(cursor 跳走 / 值改了),避免閃爍
  static int   lastCursor = -1;
  static float lastVal[N_PARAM] = { -999, -999, -999, -999, -999 };
  char buf[24];
  const int rowH = 46;     // 直立空間多,每列更高

  for (int i = 0; i < N_PARAM; i++) {
    bool sel    = (i == menuCursor);
    bool wasSel = (i == lastCursor);
    bool valChanged = (lastVal[i] != params[i].val);

    if (sel != wasSel || valChanged || lastCursor == -1) {
      int y = 40 + i * rowH;
      uint16_t bg = sel ? TFT_DARKGREEN : TFT_BLACK;
      uint16_t fg = sel ? TFT_YELLOW    : TFT_WHITE;
      tft.fillRect(0, y, 240, rowH, bg);
      tft.setFont(&fonts::efontTW_24);
      tft.setTextColor(fg, bg);
      tft.setCursor(10, y + 10);
      tft.print(sel ? ">" : " ");
      tft.print(params[i].name);
      tft.setCursor(120, y + 10);
      snprintf(buf, sizeof(buf), "%7.3f", params[i].val);
      tft.print(buf);
      lastVal[i] = params[i].val;
    }
  }
  lastCursor = menuCursor;
}

// ============================================================
void setup() {
  Serial.begin(115200);

  // 關掉 ESP32-S3 板上的 WS2812 RGB LED(常在 GPIO 48,跟我們 TFT CS 共腳)
  // 在 tft.init() 之前送一次「全 0 = 關燈」訊號,之後 GPIO 48 切換成 SPI CS
  // 用,LED 會保持上次的狀態(熄滅)。如果你的板 LED 不在 GPIO 48,改下面數字。
  neopixelWrite(48, 0, 0, 0);

  pinMode(SHOULDER_L, INPUT_PULLUP);
  pinMode(SHOULDER_R, INPUT_PULLUP);
  analogReadResolution(12);

  // TFT 背光:MSP2806 的 LED 腳直接接 B 板 3V3 軌常亮,不需要程式控制
  // (試燒驗證:LED 接 GPIO 15 PWM 不會亮,直接拉 3V3 才亮)

  // TFT 先起來，方便顯示開機進度
  tft.init();
  tft.setRotation(2);   // 240×320 portrait 翻 180°(實體裝機後上下顛倒,故設 2)
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::efontTW_16);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(5, 5);
  tft.print("無人機地面站 V2-A");
  tft.setCursor(5, 28);
  tft.print("初始化 SPI / NRF24 / SD ...");

  // SPI:NRF24 + SD 共用
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);

  // NRF24(關鍵連線,先起)
  radio.begin();
  radio.openWritingPipe(PIPE_OUT);
  radio.setChannel(NRF_CHANNEL);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
  Serial.println("[+] NRF24 就緒");

  // SD(選用,沒卡也繼續)
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  if (SD.begin(PIN_SD_CS, SPI)) {
    sdOK = true;
    Serial.println("[+] SD 卡掛載成功");
  } else {
    Serial.println("[*] SD 卡未接或初始化失敗(繼續)");
  }

  delay(300);
  drawFlightStatic();
  Serial.println("地面站 V2-A 就緒");
}

// ============================================================
void loop() {
  byte mode = readSwitch3(SW_A) * 10 + readSwitch3(SW_B);   // 兩位數編碼:A 十位 + B 個位

  // 按鈕單一邊緣偵測
  int btn = readMenuBtn();
  static int lastBtn = BTN_NONE;
  bool btnEdge = (btn != lastBtn && btn != BTN_NONE);
  lastBtn = btn;

  // UI 狀態機
  if (ui == UI_FLIGHT) {
    if (btnEdge && mode == 0 && btn == BTN_OK) {
      ui = UI_MENU;
      drawMenuStatic();
    }
  } else {  // UI_MENU
    if (mode != 0) {
      ui = UI_FLIGHT;
      drawFlightStatic();
    } else {
      handleMenuCursor();
      if (btnEdge) handleMenuButton(btn);
    }
  }

  readInputs(mode);

  // 送指令 + 收遙測（ACK payload）
  if (radio.write(&data, sizeof(Signal))) {
    if (radio.isAckPayloadAvailable()) {
      radio.read(&tele, sizeof(tele));
      teleOK = true;
    }
  } else {
    teleOK = false;
  }
  data.paramID = 0;   // 參數送完清掉

  // TFT 更新（每 150ms）
  if (millis() - lastDrawTime > 150) {
    lastDrawTime = millis();
    if (ui == UI_FLIGHT) drawFlightDynamic(mode);
    else                 drawMenuDynamic();
  }

  delay(20);   // 約 50Hz
}
