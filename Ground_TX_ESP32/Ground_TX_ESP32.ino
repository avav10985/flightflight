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
#include <driver/i2s_std.h>   // 音樂播放 I²S TX(ENABLE_MUSIC)
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
      cfg.pin_miso    = 40;                // 跟 SD 共用 SPI 必須設 40,否則 SD 拿不到 MISO 掛載失敗
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
// 2026-06-07 試燒實測:使用者只接了油門(GPIO 1)+ 右搖桿 Y(GPIO 4)
//   GPIO 2(左 X)、GPIO 10(右 X) 兩條線還沒接
//   把 J_PITCH 改到 GPIO 4(有訊號),J_ROLL 留 GPIO 10(以後接 右 X 才動)
#define J_THROTTLE   1    // 左搖桿 Y(油門,拆彈簧)— ✅ 已接
#define J_LEFT_X     2    // 左搖桿 X(可選 yaw)— ❌ 還沒接
#define J_PITCH      4    // 右搖桿 Y(俯仰)— ✅ 已接(從 GPIO 10 對調過來)
#define J_ROLL      10    // 右搖桿 X(翻滾)— ❌ 還沒接(從 GPIO 4 對調過來)
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
// SD 卡:獨立第二組 SPI(SPI3/HSPI),跟 NRF24/TFT 的 bus 完全分開。
// 2026-06-12 實測:SD 模組 level shifter 在共用 bus 上不放開 MISO,
// 連新電源架構也救不了 → 搬到專用腳位,讓它佔自己的線。
// MISO 曾誤選 GPIO 3(接線總表標「避開」,實測也失敗),改用 47;
// CS 回歸接線總表原始設計 GPIO 0(BOOT 腳輸出閒置 HIGH 安全,R11 上拉)
#define PIN_SD_CS    0   // SD CS(原始設計,R11 10k 上拉)
#define PIN_SD_SCK  15   // SD 專用 SCK(真正釋出腳)
#define PIN_SD_MOSI 17   // SD 專用 MOSI(真正釋出腳)
#define PIN_SD_MISO 47   // SD 專用 MISO(原 CS 腳讓出來)

// ====== V2-B 跟其他 feature 旗標(2026-06-06)======
// 硬體 / API key 齊全才改 1,**程式編譯時不會占用太多 flash**
#define ENABLE_PC_MODE11        0   // Mode 11:PC 透過 USB Serial 控制飛機(透明橋接)
#define ENABLE_VOICE_MODE10     0   // Mode 10:語音控制(需 INMP441 + MAX98357A + WiFi + API keys)
#define ENABLE_MUSIC            1   // 任意 mode 都可放音樂(需 SD + MAX98357A,2026-06-12 開)
#define ENABLE_VIDEO_MODE20     1   // Mode 20:媒體模式,SD /video/*.mjp 影片播放
                                    // (需 ENABLE_MUSIC=1 共用 I²S + JPEGDEC 函式庫)
#define ENABLE_PERSISTENT_MENU  0   // TFT 下方常駐選單(肩鍵 L 長按 1 秒進入)
#define ENABLE_SD               1   // SD 卡模組:1 = 啟用(2026-06-11 新電源架構後重測。
                                    // 2026-06-09 曾因模組 level shifter 搶 SPI bus 拖爆 NRF24 停用;
                                    // 若 chip connected 變 NO 或飛機連不上,改回 0)
#define PIN_I2S_BCLK   11           // V2-B 語音模組腳位(共用 INMP441 + MAX98357A)
#define PIN_I2S_WS     12
#define PIN_I2S_DOUT   13           // → MAX98357A DIN
#define PIN_I2S_DIN    14           // INMP441 SD →
#define PIN_AMP_SD     18           // MAX98357A SD(shutdown):LOW=休眠靜音、HIGH=啟用
                                    // 預設拉低消除「沙沙」雜訊,Mode 10 / 音樂 要播放時設 HIGH

#if ENABLE_VIDEO_MODE20
#include <JPEGDEC.h>                // Library Manager 搜 JPEGDEC(Larry Bank)
#if !ENABLE_MUSIC
#error "ENABLE_VIDEO_MODE20 需要 ENABLE_MUSIC=1(共用 I2S TX 通道)"
#endif
void enterVideoMode();
void exitVideoMode();
void videoModeLoop(int btnEdge);
#endif

// ---- 方向反轉（測試後不對就改）----
const bool REV_THROTTLE = true;
const bool REV_PITCH    = true;
const bool REV_ROLL     = true;
// yaw 方向若相反，把 SHOULDER_L / SHOULDER_R 兩腳對調即可

// ---- NRF24 ----
// 2026-06-07 換新 pipe + channel(跟 Drone_FC_Full PIPE_IN 必須一致)
const uint64_t PIPE_OUT    = 0x4E6F9C2D5BLL;
const uint8_t  NRF_CHANNEL = 88;

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
// 2026-06-09 移除 battery_mV(使用者沒裝),改放 altitude + heading
struct Telemetry {
  int16_t  roll;        // 角度 ×10
  int16_t  pitch;       // 角度 ×10
  int16_t  yawRate;     // 角速度 ×10°/s
  int16_t  altitude_dm; // 相對高度 ×10(decimeter,範圍 ±3276.7m)
  int16_t  heading;     // 機頭方位 ×10(0~3599,0=北 900=東 1800=南 2700=西)
  uint8_t  status;      // 多 bit:見下面 STATUS_*
  uint8_t  satCount;    // GPS 衛星數
  int32_t  lat_e7;      // 緯度 ×10^7
  int32_t  lon_e7;      // 經度 ×10^7
};   // 20 bytes(NRF24 ACK Payload 上限 32)

// status 位元定義(跟飛機端一致)
#define STATUS_ARMED         0x01   // bit0:已 armed
#define STATUS_CALIBRATING   0x02   // bit1:正在校準
#define STATUS_CAL_FAILED    0x04   // bit2:上次校準失敗
#define STATUS_GPS_FIX       0x08   // bit3:GPS 有效定位

Signal    data;
Telemetry tele;
bool      teleOK = false;
bool      sdOK   = false;
#if ENABLE_SD
SPIClass  spiSD(HSPI);   // SD 專用第二組 SPI(S3 上 HSPI = SPI3)
#endif

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

// PID 本地副本(開機與飛機預設一致)
// 2026-06-09 跟飛機 Drone_FC_Full(commit fe466f0)同步降到保守值
// 避免一進選單就送舊高值給飛機 → 飛機 PID 被覆蓋成激進值
// isAction = true → 動作項目(校準/存檔),OK 觸發、不顯示數值
// 動作項目用 paramVal 當 pulse counter,每按一次 +1,飛機 dedup 看 (id,val) 變化才動作
struct Param { const char* name; float val; float step; byte id; bool isAction; };
Param params[] = {
  { "Kp ",      1.0f,  0.1f,   1,   false },
  { "Ki ",      0.01f, 0.005f, 2,   false },
  { "Kd ",      0.3f,  0.05f,  3,   false },
  { "KpY",      0.8f,  0.1f,   4,   false },
  { "KiY",      0.01f, 0.005f, 5,   false },
  { "保存PID",  0.0f,  1.0f,   102, true  },
  { "快速校準",  0.0f,  1.0f,   100, true  },
  { "完整校準",  0.0f,  1.0f,   101, true  },
  { "音樂",     0.0f,  1.0f,   200, true  },   // id 200 = 本地動作,不送飛機
};
const int N_PARAM = 9;

#if ENABLE_MUSIC
void musicToggle();   // 前置宣告,實作在音樂區塊
#endif
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

// 全 redraw flag:drawFlightStatic 設 true,drawFlightDynamic 第一次跑會強制重畫所有
// (從 PID 選單返回後 mode 沒變但畫面被清掉,沒這個會空一片)
bool g_flightDirty = false;
bool g_menuDirty   = false;

// 2026-06-07 試燒實測:+ 3810~3850、− 2640~2690、OK 1910~1940、返回 1160~1200
// 門檻取中點,邊緣最穩(舊門檻邊緣太靠近實測值,瞬間切換時容易誤觸)
int readMenuBtn() {
  int v = analogRead(MENU_BTN);
  if (v > 3300) return BTN_PLUS;
  if (v > 2300) return BTN_MINUS;
  if (v > 1550) return BTN_OK;
  if (v > 600)  return BTN_BACK;
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
  // 2026-06-07 修:推下選下、推上選上(用 REV_PITCH 跟飛行 mode 同方向)
  // 死區 55/200 改成 15/240(搖桿中位偏離時不誤觸)
  static unsigned long lastMove = 0;
  if (millis() - lastMove < 250) return;
  int p = centerMap(analogRead(J_PITCH), REV_PITCH);   // 跟 data.pitch 用同方向
  if (p > 240) { menuCursor = (menuCursor + 1) % N_PARAM;            lastMove = millis(); }
  if (p < 15)  { menuCursor = (menuCursor - 1 + N_PARAM) % N_PARAM;  lastMove = millis(); }
}

void handleMenuButton(int btn) {
  if (btn == BTN_BACK) { ui = UI_FLIGHT; drawFlightStatic(); return; }
  Param &p = params[menuCursor];
  if (p.isAction) {
    // 動作項目:只認 OK,每按一次 paramVal +1 當 pulse,飛機看 val 變才會做
    if (btn == BTN_OK) {
#if ENABLE_MUSIC
      if (p.id == 200) { musicToggle(); return; }   // 本地動作,不送飛機
#endif
      p.val += 1.0f;
      data.paramID  = p.id;
      data.paramVal = p.val;
    }
  } else {
    if (btn == BTN_PLUS)  p.val += p.step;
    if (btn == BTN_MINUS) p.val -= p.step;
    if (p.val < 0) p.val = 0;
    data.paramID  = p.id;
    data.paramVal = p.val;
  }
}

// ============================================================
// TFT UI(直立 240×320):static 一次畫底色 + dynamic 文字覆寫
// ============================================================
void drawFlightStatic() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0,   0, 240, 32, TFT_NAVY);     // 頂部標題列
  tft.fillRect(0, 280, 240, 40, TFT_DARKGREY); // 底部狀態列

  // 三個姿態 label 預先畫上(不會變,不用每次重畫)
  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 50);  tft.print("翻滾");
  tft.setCursor(10, 130); tft.print("俯仰");
  tft.setCursor(10, 210); tft.print("偏航");

  g_flightDirty = true;   // 強制下一次 drawFlightDynamic 重畫所有 cached 部分
}

void drawFlightDynamic(byte mode) {
  char buf[40];
  bool armed = tele.status & STATUS_ARMED;

  // === 頂部 mode 文字:只在 mode 改變時重畫 ===
  static byte lastModeDrawn = 255;
  if (mode != lastModeDrawn || g_flightDirty) {
    tft.fillRect(0, 0, 175, 32, TFT_NAVY);   // 先清舊文字(留 165 給已武/安全)
    tft.setFont(&fonts::efontTW_24);
    tft.setTextColor(TFT_WHITE, TFT_NAVY);
    tft.setCursor(5, 4);
    snprintf(buf, sizeof(buf), "M%02d %s", mode, getModeName(mode));
    tft.print(buf);
    lastModeDrawn = mode;
  }

  // === 武裝狀態:只在改變時重畫 ===
  static bool lastArmed = false;
  static bool armedInit = false;
  if (armed != lastArmed || !armedInit || g_flightDirty) {
    tft.fillRect(175, 0, 65, 32, TFT_NAVY);
    tft.setFont(&fonts::efontTW_24);
    tft.setTextColor(armed ? TFT_RED : TFT_LIGHTGREY, TFT_NAVY);
    tft.setCursor(180, 4);
    tft.print(armed ? "已武" : "安全");
    lastArmed = armed;
    armedInit = true;
  }

  // === 主體:已實作/未實作 切換時清主體 + 重畫 label ===
  static byte lastBlock = 255;
  byte block = modeImplemented(mode) ? 1 : 0;
  if (block != lastBlock || g_flightDirty) {
    tft.fillRect(0, 34, 240, 244, TFT_BLACK);
    if (modeImplemented(mode)) {
      // 把 label 補回去(static 第一次有畫,主體被清掉後再補)
      tft.setFont(&fonts::efontTW_24);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setCursor(10, 50);  tft.print("翻滾");
      tft.setCursor(10, 130); tft.print("俯仰");
      tft.setCursor(10, 210); tft.print("偏航");
    } else {
      tft.setFont(&fonts::efontTW_24);
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.setCursor(40, 140); tft.print("此模式");
      tft.setCursor(40, 175); tft.print("未實作");
    }
    lastBlock = block;
    g_flightDirty = true;   // 主體被清過,下面值檢查強制重畫(否則 tele 沒變就空白)
  }

  // === 姿態數值:只在改變時重畫 ===
  if (modeImplemented(mode)) {
    static int16_t lastRoll = -32000, lastPitch = -32000, lastYaw = -32000;
    tft.setFont(&fonts::efontTW_24);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    if (tele.roll != lastRoll || g_flightDirty) {
      tft.fillRect(10, 80, 200, 30, TFT_BLACK);
      tft.setCursor(10, 80);
      snprintf(buf, sizeof(buf), "%+7.1f °", tele.roll / 10.0);
      tft.print(buf);
      lastRoll = tele.roll;
    }
    if (tele.pitch != lastPitch || g_flightDirty) {
      tft.fillRect(10, 160, 200, 30, TFT_BLACK);
      tft.setCursor(10, 160);
      snprintf(buf, sizeof(buf), "%+7.1f °", tele.pitch / 10.0);
      tft.print(buf);
      lastPitch = tele.pitch;
    }
    if (tele.yawRate != lastYaw || g_flightDirty) {
      tft.fillRect(10, 240, 230, 30, TFT_BLACK);
      tft.setCursor(10, 240);
      snprintf(buf, sizeof(buf), "%+7.1f °/秒", tele.yawRate / 10.0);
      tft.print(buf);
      lastYaw = tele.yawRate;
    }
  }
  g_flightDirty = false;   // 消耗 dirty 旗標

  // 底部狀態列(三行,塞 GPS / 高度 / 方位 / 油門 / 連線)
  tft.setFont(&fonts::efontTW_14);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  // 第 1 行:油門 + 高度
  snprintf(buf, sizeof(buf), "油門%3d%%  高度%+5.1fm",
           (int)(data.throttle * 100 / 255),
           tele.altitude_dm / 10.0);
  tft.setCursor(5, 283);
  tft.print(buf);
  // 第 2 行:方位 + GPS 衛星數
  const char* dir = "?";
  int h10 = tele.heading;
  if      (h10 >= 3375 || h10 < 225)  dir = "北";
  else if (h10 <  675)                dir = "東北";
  else if (h10 < 1125)                dir = "東";
  else if (h10 < 1575)                dir = "東南";
  else if (h10 < 2025)                dir = "南";
  else if (h10 < 2475)                dir = "西南";
  else if (h10 < 2925)                dir = "西";
  else                                dir = "西北";
  snprintf(buf, sizeof(buf), "方位%5.1f° %s  GPS sat%d",
           tele.heading / 10.0, dir, tele.satCount);
  tft.setCursor(5, 300);
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
  g_menuDirty = true;   // 進選單後第一次 drawMenuDynamic 強制畫所有列
}

void drawMenuDynamic() {
  // 只重畫狀態變過的列(cursor 跳走 / 值改了),避免閃爍
  static int   lastCursor = -1;
  static float lastVal[N_PARAM] = { -999, -999, -999, -999, -999, -999, -999, -999, -999 };
  char buf[24];
  const int rowH = 26;     // 9 列要塞進 40~280 px(9×26=234),行高再壓低

  for (int i = 0; i < N_PARAM; i++) {
    bool sel    = (i == menuCursor);
    bool wasSel = (i == lastCursor);
    bool valChanged = (lastVal[i] != params[i].val);

    if (sel != wasSel || valChanged || lastCursor == -1 || g_menuDirty) {
      int y = 40 + i * rowH;
      uint16_t bg = sel ? TFT_DARKGREEN : TFT_BLACK;
      uint16_t fg = sel ? TFT_YELLOW    : TFT_WHITE;
      tft.fillRect(0, y, 240, rowH, bg);
      tft.setFont(&fonts::efontTW_24);
      tft.setTextColor(fg, bg);
      tft.setCursor(10, y + 1);
      tft.print(sel ? ">" : " ");
      tft.print(params[i].name);
      tft.setCursor(160, y + 1);
      if (params[i].isAction) {
        tft.print("GO");
      } else {
        snprintf(buf, sizeof(buf), "%6.3f", params[i].val);
        tft.print(buf);
      }
      lastVal[i] = params[i].val;
    }
  }
  lastCursor = menuCursor;
  g_menuDirty = false;
}

// ============================================================
void setup() {
  Serial.begin(115200);
  // 等 USB CDC 真的連上(最多 3 秒),沒 host 連接也照樣繼續
  // 比固定 delay(2000) 可靠:有 Serial Monitor 接著就立刻過,沒接就 3 秒後超時
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(200);   // 額外緩衝,讓 host 端 buffer 準備好
  Serial.println("\n=== 地面站 V2-A 啟動 ===");
  Serial.flush();

  // 關掉 ESP32-S3 板上的 WS2812 RGB LED(常在 GPIO 48,跟我們 TFT CS 共腳)
  // 在 tft.init() 之前送一次「全 0 = 關燈」訊號,之後 GPIO 48 切換成 SPI CS
  // 用,LED 會保持上次的狀態(熄滅)。如果你的板 LED 不在 GPIO 48,改下面數字。
  neopixelWrite(48, 0, 0, 0);

  pinMode(SHOULDER_L, INPUT_PULLUP);
  pinMode(SHOULDER_R, INPUT_PULLUP);
  analogReadResolution(12);

  // MAX98357A SD 拉低 → 休眠,沒「沙沙」雜訊。Mode 10 / 音樂播放時程式再設 HIGH
  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, LOW);

  // TFT 背光:MSP2806 的 LED 腳直接接 B 板 3V3 軌常亮,不需要程式控制
  // (試燒驗證:LED 接 GPIO 15 PWM 不會亮,直接拉 3V3 才亮)

  // === SPI / NRF24 / SD 先 init,TFT 後加入共用 ===
  // 順序顛倒(TFT 先)會讓 SD 拿不到 MISO 掛載失敗(2026-06-06 試燒實證)
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);

  // NRF24
  Serial.print("[*] NRF24 radio.begin() ... ");
  bool nrfOK = radio.begin();
  Serial.println(nrfOK ? "OK" : "FAIL");
  Serial.printf("[*] NRF24 chip connected: %s\n",
                radio.isChipConnected() ? "YES" : "NO ⚠️ 模組沒回應 SPI");
  if (!nrfOK || !radio.isChipConnected()) {
    Serial.println("[!] 手把 NRF24 init 失敗,通常是:");
    Serial.println("    1. 100µF 電容沒焊");
    Serial.println("    2. CE/CSN/SCK/MISO/MOSI 接線錯");
    Serial.println("    3. 模組壞了");
    Serial.println("    4. 3V3 軌電壓不夠");
  }
  radio.openWritingPipe(PIPE_OUT);
  radio.setChannel(NRF_CHANNEL);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setDataRate(RF24_250KBPS);
  // 2026-06-09 PA_HIGH 還是讓 3V3 軌 sag(LDO 撐不住 50mA 尖峰)
  // 降到 PA_MIN(~7mA 尖峰),確認對通邏輯先,距離 ~10m 夠桌面測試
  // 飛實機要長距離再加大電容 / 加獨立 LDO 後改回 HIGH 或 MAX
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  Serial.println("[+] NRF24 就緒");

#if ENABLE_SD
  // SD(選用,沒卡也繼續)— 用獨立 SPI3,不碰 NRF24/TFT 的 bus
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  spiSD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (SD.begin(PIN_SD_CS, spiSD)) {
    sdOK = true;
    Serial.println("[+] SD 卡掛載成功(獨立 SPI3)");
  } else {
    Serial.println("[*] SD 卡未接或初始化失敗(繼續)");
  }
#else
  // ENABLE_SD = 0:跳過 SD init,GPIO 47 不操作,留給未來其他用途
  Serial.println("[*] SD 停用(ENABLE_SD = 0)");
  sdOK = false;
#endif

  // MAX98357A SD 腳:開機立刻拉低靜音(浮空會放大 I²S 雜訊,memory 老坑)
  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, LOW);
#if ENABLE_MUSIC
  initMusicI2S();
  Serial.println("[+] 音樂 I²S 就緒(選單→音樂 播放/停止)");
#endif

  // TFT(SD 之後 init,共用 SPI bus_shared 模式)
  tft.init();
  tft.setRotation(2);   // 240×320 portrait 翻 180°
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::efontTW_16);

  // 開機狀態畫面(因 USB CDC 在 ESP32-S3 有問題,debug 訊息全部上 TFT)
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(5, 5);
  tft.print("地面站 V2-A 啟動");

  // NRF24 狀態:大字顯示 OK / FAIL,3 秒後才進飛行 UI
  tft.setCursor(5, 40);
  tft.setTextColor(nrfOK ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.print("NRF24 begin: ");
  tft.print(nrfOK ? "OK" : "FAIL");

  tft.setCursor(5, 70);
  bool chipOK = radio.isChipConnected();
  tft.setTextColor(chipOK ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.print("chip connected: ");
  tft.print(chipOK ? "YES" : "NO");

  tft.setCursor(5, 100);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.printf("PIPE = 0x%04X%04X", (uint16_t)(PIPE_OUT >> 16), (uint16_t)PIPE_OUT);
  tft.setCursor(5, 130);
  tft.printf("CHAN = %d", NRF_CHANNEL);

  tft.setCursor(5, 170);
  if (sdOK) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print("SD: 已掛載");
  } else {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.print("SD: 未接");
  }

  tft.setCursor(5, 250);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.print("3 秒後進飛行 UI...");

  delay(3000);

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

#if ENABLE_VIDEO_MODE20
  // Mode 20 媒體模式:接管整個 loop。NRF24 照送(mode=20 → 飛機白名單
  // 擋掉自動 disarm),模式開關一撥走立刻退出
  static bool inVideoMode = false;
  if (mode == 20) {
    if (!inVideoMode) { inVideoMode = true; enterVideoMode(); }
    videoModeLoop(btnEdge ? btn : BTN_NONE);
    readInputs(mode);                       // data.mode = 20
    radio.write(&data, sizeof(Signal));
    return;
  } else if (inVideoMode) {
    inVideoMode = false;
    exitVideoMode();
    ui = UI_FLIGHT;
    drawFlightStatic();
  }
#endif

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

#if ENABLE_MUSIC
  musicUpdate();   // 非阻塞餵 I²S,DMA 滿了立刻返回
#endif

  // 送指令 + 收遙測(ACK payload)
  // Debug:每秒印一次 TX 統計,看到底有沒有送出去 + 飛機有沒有 ACK 回來
  static uint32_t txOk = 0, txFail = 0, lastTxStat = 0;
  bool ok = radio.write(&data, sizeof(Signal));
  if (ok) {
    txOk++;
    if (radio.isAckPayloadAvailable()) {
      radio.read(&tele, sizeof(tele));
      teleOK = true;
    }
  } else {
    txFail++;
    teleOK = false;
  }
  if (millis() - lastTxStat > 1000) {
    lastTxStat = millis();
    Serial.printf("[NRF] TX ok=%lu fail=%lu (1 秒內) | mode=%02d throttle=%d\n",
                  txOk, txFail, data.mode, data.throttle);
    txOk = 0; txFail = 0;
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


// ============================================================
// ============================================================
//                  以下都是 feature 骨架
//   ENABLE_XXX 旗標 = 0 時不會啟用,只是先把架構放這方便將來補
// ============================================================
// ============================================================


// ============================================================
// Mode 11:PC 控制(USB CDC 透明橋接)
//
// 開啟方法:#define ENABLE_PC_MODE11 1
//
// 使用流程:
// 1. PC 用 USB 接手把(會佔用 Serial 0,無法看 debug log)
// 2. 撥到 mode = 11(SW_A 中 + SW_B 中),手把進入 PC 橋接模式
// 3. PC 端 app 透過 USB Serial 送 13-byte PcCommand,手把轉成 Signal 送 NRF24
// 4. NRF24 回的 telemetry 透過 USB Serial 回 PC
//
// PC 端 app 建議:Python + pyserial + pygame.joystick + opencv(相機影像)
// 詳細實作等使用者要做時再細談
// ============================================================
#if ENABLE_PC_MODE11
struct PcCommand {
  uint8_t startByte;    // 必須 0xAA
  uint8_t throttle, pitch, roll, yaw;
  uint8_t mode;         // PC 可強制覆蓋手把開關位置
  uint8_t flags;
  uint8_t paramID;
  float   paramVal;
  uint8_t checksum;     // sum of bytes 1~11 取低 8 bit
};

void pcBridgeUpdate(Signal& sig) {
  // 從 USB Serial 讀 PC 指令,覆蓋 sig
  if (Serial.available() >= (int)sizeof(PcCommand)) {
    PcCommand cmd;
    Serial.readBytes((uint8_t*)&cmd, sizeof(cmd));
    if (cmd.startByte != 0xAA) return;
    // TODO:checksum 驗證
    sig.throttle = cmd.throttle;
    sig.pitch    = cmd.pitch;
    sig.roll     = cmd.roll;
    sig.yaw      = cmd.yaw;
    sig.mode     = cmd.mode;
    sig.flags    = cmd.flags;
    sig.paramID  = cmd.paramID;
    sig.paramVal = cmd.paramVal;
  }
  // 把當前 telemetry 推給 PC(每次都送,讓 PC 端隨時有最新狀態)
  Serial.write((const uint8_t*)&tele, sizeof(Telemetry));
}
#endif


// ============================================================
// Mode 10:AI 語音控制
//
// 開啟方法:#define ENABLE_VOICE_MODE10 1 + 建立 secrets.h 含:
//   WIFI_SSID / WIFI_PASS
//   WIT_AI_TOKEN(免費,Wit.ai 中文 STT)
//   GROQ_API_KEY(免費 14400 req/day,Llama 3.3)
//   VOICERSS_API_KEY(免費 350 req/day,中文 TTS)
//
// 流程:
//   ESP-SR 監聽喚醒詞「啟動」(本地,免雲端)
//   ↓ 偵測到
//   錄音 5 秒 → 上傳 Wit.ai → 取得 STT 結果(中文文字)
//   ↓
//   比對本地指令清單(起飛/降落/返航/懸停/聊天/取消/狀態/加速/減速)
//   ↓ 是本地指令 → 直接執行(改 Signal.mode 或 flags)
//   ↓ 否則 → 送 Groq LLM 取得對話回應 → 上傳 VoiceRSS 取得 MP3 → 播放
//
// 詳細申請 API key 跟 ESP-SR 設定 在 [[mode-10-ai-voice-apis]] 記憶
// ============================================================
#if ENABLE_VOICE_MODE10
#include <WiFi.h>
#include <HTTPClient.h>
#include "secrets.h"   // 自己建立,git ignore

enum VoiceState { VS_IDLE, VS_WAKE, VS_RECORDING, VS_STT, VS_LLM, VS_TTS, VS_PLAY };
VoiceState voiceState = VS_IDLE;

const char* LOCAL_CMDS[] = {
  "啟動", "起飛", "降落", "返航", "懸停",
  "取消", "狀態", "加速", "減速", "聊天"
};

void initVoice() {
  // WiFi 連線
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) { delay(500); }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[+] WiFi:%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[!] WiFi 連線失敗,Mode 10 雲端 STT/LLM/TTS 無法使用");
  }
  // TODO:ESP-SR MultiNet 載入「啟動」喚醒詞模型
}

void voiceUpdate() {
  // 狀態機:每次 loop 呼叫一次,各狀態做一小段工作避免卡 NRF24
  switch (voiceState) {
    case VS_IDLE:
      // TODO:if (espSrDetectWakeWord("啟動")) voiceState = VS_WAKE;
      break;
    case VS_WAKE:
      // TODO:嗶提示音 → voiceState = VS_RECORDING
      break;
    case VS_RECORDING:
      // TODO:錄音 5 秒到 PSRAM buffer
      break;
    case VS_STT:
      // TODO:HTTPS POST 到 https://api.wit.ai/speech,header WIT_AI_TOKEN
      break;
    case VS_LLM:
      // TODO:HTTPS POST 到 https://api.groq.com/openai/v1/chat/completions
      break;
    case VS_TTS:
      // TODO:HTTPS GET https://api.voicerss.org/?key=...&hl=zh-tw&src=回應
      break;
    case VS_PLAY:
      // TODO:解 MP3 推 I²S → MAX98357A
      break;
  }
}
#endif


// ============================================================
// 音樂播放(SD 上 /music/*.WAV → I²S → MAX98357A)
//
// 開啟:#define ENABLE_MUSIC 1
// 觸發:常駐選單 → 「音樂」項目 → 選曲 → OK 播放
//      或 Mode 10 語音「啟動 → 放音樂」
//
// 優先級:警示音 > Mode 10 語音回應 > 背景音樂
// 預設音量 60%,Mode 10 識別期間自動降到 20%
// ============================================================
#if ENABLE_MUSIC
File     musicFile;
bool     musicPlaying = false;
uint8_t  musicVolume  = 60;     // 0~100

#define MUSIC_SAMPLE_RATE 32000  // WAV 需轉成 32kHz / 單聲道 / 16-bit PCM
i2s_chan_handle_t musicTx = nullptr;

// I²S TX 初始化(沿用 Play_Test 驗證過的設定)
void initMusicI2S() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  // 主程式 loop 比 Play_Test 忙(NRF24 重試 + TFT 重繪),緩衝拉到 ~190ms
  // 6 × 1023 frames × 4B ≈ 24KB 內部 RAM,換 6138 samples 餘裕
  chan_cfg.dma_desc_num  = 6;
  chan_cfg.dma_frame_num = 1023;
  i2s_new_channel(&chan_cfg, &musicTx, NULL);

  i2s_std_config_t std_cfg = {};
  std_cfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(MUSIC_SAMPLE_RATE);
  std_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
  std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  std_cfg.gpio_cfg.bclk = (gpio_num_t)PIN_I2S_BCLK;
  std_cfg.gpio_cfg.ws   = (gpio_num_t)PIN_I2S_WS;
  std_cfg.gpio_cfg.dout = (gpio_num_t)PIN_I2S_DOUT;
  std_cfg.gpio_cfg.din  = I2S_GPIO_UNUSED;
  i2s_channel_init_std_mode(musicTx, &std_cfg);
  i2s_channel_enable(musicTx);

  // 預灌 silence 讓 MAX98357A 醒來看到穩定 0 訊號(boot pop 消除)
  int32_t silence[256] = {0};
  size_t w = 0;
  for (int i = 0; i < 4; i++)
    i2s_channel_write(musicTx, silence, sizeof(silence), &w, 50 / portTICK_PERIOD_MS);
}

void musicPlay(const char* path) {
  if (musicPlaying) { musicFile.close(); }
  musicFile = SD.open(path, FILE_READ);
  if (!musicFile) { Serial.printf("[!] 開不了 %s\n", path); return; }
  musicFile.seek(44);   // 跳 WAV header
  musicPlaying = true;
  digitalWrite(PIN_AMP_SD, HIGH);   // 開擴大機
  Serial.printf("[+] 播放 %s\n", path);
}

void musicStop() {
  if (musicPlaying) { musicFile.close(); musicPlaying = false; }
  digitalWrite(PIN_AMP_SD, LOW);    // 關擴大機(靜音 + 消雜訊)
}

// 每迴圈呼叫:非阻塞地把 SD 資料餵進 I²S DMA。
// timeout 0:DMA 滿了就立刻返回,絕不卡住 50Hz 控制迴圈。
void musicUpdate() {
  if (!musicPlaying) return;
  static uint8_t pcm[512];
  static int32_t tx[256];
  for (int round = 0; round < 16; round++) {    // 每迴圈最多餵 4096 samples(128ms 音訊);
                                                 // DMA 滿會提前 return,實際讀 SD 時間只 2~4ms
    int n = musicFile.read(pcm, sizeof(pcm));
    if (n <= 0) {                                // 播完
      Serial.println("[+] 音樂播畢");
      musicStop();
      return;
    }
    int samples = n / 2;
    int16_t* s = (int16_t*)pcm;
    for (int i = 0; i < samples; i++)
      tx[i] = ((int32_t)(s[i] * musicVolume / 100)) << 16;
    size_t written = 0;
    i2s_channel_write(musicTx, tx, samples * 4, &written, 0);   // 非阻塞
    size_t consumedPcm = written / 2;            // tx bytes 是 pcm bytes 的 2 倍
    if ((int)consumedPcm < n) {
      // DMA 滿:把沒寫進去的部分倒帶,下迴圈再餵
      musicFile.seek(musicFile.position() - (n - consumedPcm));
      return;
    }
  }
}

// 選單「音樂」項目:播放 ⇄ 停止
void musicToggle() {
  if (musicPlaying) { musicStop(); return; }
  if (!sdOK) { Serial.println("[!] SD 沒掛載,不能放音樂"); return; }
  // 先找 /music/ 第一首,沒有就退回根目錄 REC_001.WAV
  String list[1];
  extern int musicListFiles(String list[], int maxN);
  if (musicListFiles(list, 1) > 0) {
    String path = String("/music/") + list[0];
    musicPlay(path.c_str());
  } else {
    musicPlay("/REC_001.WAV");
  }
}

// 列出 SD /music/ 資料夾的所有 .WAV 檔(常駐選單呼叫)
int musicListFiles(String list[], int maxN) {
  int n = 0;
  File dir = SD.open("/music");
  if (!dir) return 0;
  while (n < maxN) {
    File f = dir.openNextFile();
    if (!f) break;
    String name = f.name();
    if (name.endsWith(".WAV") || name.endsWith(".wav")) list[n++] = name;
    f.close();
  }
  dir.close();
  return n;
}
#endif


// ============================================================
// Mode 20:媒體模式(SD 影片播放,Video_Test 2026-06-12 移植)
//
// 檔案:/video/*.mjp(MJPEG)+ 同名 .wav 聲音(可無 → 無聲播放)
// 轉檔:ffmpeg -i in.mp4 -vf "scale=240:320:force_original_aspect_ratio=decrease:force_divisible_by=2,fps=15" -q:v 8 -f mjpeg xxx.mjp
//       ffmpeg -i in.mp4 -ar 32000 -ac 1 -sample_fmt s16 xxx.wav
// 操作:右搖桿選檔 → OK 播放 → 返回/OK 停止 → 模式開關撥走立刻退出
// 同步:聲音當時鐘,解碼落後 >2 幀跳幀
// ============================================================
#if ENABLE_VIDEO_MODE20
#define VID_FPS        15.0f
#define VID_FRAME_MAX  (160 * 1024)
#define VID_MAX_FILES  8

JPEGDEC  vidJpeg;
File     vidFile, vidAud;
uint8_t* vidFrameBuf  = nullptr;
bool     vidPlaying   = false;
bool     vidHasAudio  = false;
uint64_t vidAudioFed  = 0;
uint32_t vidFrameShown = 0;
unsigned long vidStartMs = 0;
String   vidList[VID_MAX_FILES];
bool     vidIsVideo[VID_MAX_FILES];   // true=/video/*.mjp,false=/music/*.wav
int      vidCount  = 0;
int      vidCursor = 0;
bool     mediaPaused = false;
unsigned long vidPauseStart = 0;

int vidJpegDraw(JPEGDRAW* d) {
  tft.pushImage(d->x, d->y, d->iWidth, d->iHeight, (uint16_t*)d->pPixels);
  return 1;
}

void vidScanFiles() {
  vidCount = 0;
  // 影片:/video/*.mjp
  File dir = SD.open("/video");
  if (dir) {
    while (vidCount < VID_MAX_FILES) {
      File f = dir.openNextFile();
      if (!f) break;
      String name = f.name();
      if (name.endsWith(".mjp") || name.endsWith(".MJP")) {
        vidIsVideo[vidCount] = true;
        vidList[vidCount++]  = name;
      }
      f.close();
    }
    dir.close();
  }
  // 音樂:/music/*.wav
  dir = SD.open("/music");
  if (dir) {
    while (vidCount < VID_MAX_FILES) {
      File f = dir.openNextFile();
      if (!f) break;
      String name = f.name();
      if (name.endsWith(".wav") || name.endsWith(".WAV")) {
        vidIsVideo[vidCount] = false;
        vidList[vidCount++]  = name;
      }
      f.close();
    }
    dir.close();
  }
}

void drawVideoMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, 240, 32, TFT_NAVY);
  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.setCursor(30, 4);
  tft.print("Mode 20 媒體");
  if (vidCount == 0) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.setCursor(10, 100);
    tft.print("SD /video /music 都空");
    return;
  }
  for (int i = 0; i < vidCount; i++) {
    bool sel = (i == vidCursor);
    int y = 44 + i * 30;
    tft.fillRect(0, y, 240, 30, sel ? TFT_DARKGREEN : TFT_BLACK);
    tft.setTextColor(sel ? TFT_YELLOW : TFT_WHITE, sel ? TFT_DARKGREEN : TFT_BLACK);
    tft.setCursor(10, y + 3);
    tft.print(sel ? ">" : " ");
    tft.print(vidIsVideo[i] ? "[影]" : "[樂]");
    tft.print(vidList[i]);
  }
  tft.setFont(&fonts::efontTW_14);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(5, 290);
  tft.print("搖桿:選 OK:播 ↑↓音量 ←退出");
}

// 播放中的音量提示(畫在頂部,影片下一幀若蓋到會自己消失)
void drawVolOverlay() {
  char buf[20];
  snprintf(buf, sizeof(buf), " 音量 %d%% ", musicVolume);
  tft.setFont(&fonts::efontTW_16);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(6, 4);
  tft.print(buf);
}

// 音樂播放畫面
void drawMusicScreen(const String& name) {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, 240, 32, TFT_NAVY);
  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.setCursor(30, 4);
  tft.print("音樂播放中");
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, 110);
  tft.println(name);
  tft.setFont(&fonts::efontTW_14);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(5, 290);
  tft.print("↑↓音量 →暫停/繼續 ←退出");
}

// 暫停/繼續狀態顯示
void drawPauseState() {
  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(mediaPaused ? TFT_ORANGE : TFT_GREEN, TFT_BLACK);
  tft.setCursor(90, 160);
  tft.print(mediaPaused ? "[暫停]" : "[播放]");
}

void vidStop() {
  if (vidFile) vidFile.close();
  if (vidAud)  vidAud.close();
  vidPlaying  = false;
  mediaPaused = false;
  digitalWrite(PIN_AMP_SD, LOW);
}

void vidStart(const String& name) {
  vidStop();
  String base = String("/video/") + name;
  vidFile = SD.open(base, FILE_READ);
  if (!vidFile) return;
  String wav = base;                       // 同名 .wav 當聲音
  wav.replace(".mjp", ".wav");
  wav.replace(".MJP", ".WAV");
  vidAud = SD.open(wav, FILE_READ);
  vidHasAudio = (bool)vidAud;
  if (vidHasAudio) { vidAud.seek(44); digitalWrite(PIN_AMP_SD, HIGH); }
  vidAudioFed   = 0;
  vidFrameShown = 0;
  vidStartMs    = millis();
  vidPlaying    = true;
  tft.fillScreen(TFT_BLACK);
}

void vidFeedAudio() {
  if (!vidHasAudio) return;
  static uint8_t pcm[512];
  static int32_t txb[256];
  for (int round = 0; round < 16; round++) {
    int n = vidAud.read(pcm, sizeof(pcm));
    if (n <= 0) {
      // 聲音先播完:把時鐘無縫切到 millis 繼續推影像
      vidHasAudio = false;
      vidStartMs  = millis() - (unsigned long)((vidAudioFed / (float)(MUSIC_SAMPLE_RATE * 2)) * 1000);
      vidAudioFed = 0;
      return;
    }
    int samples = n / 2;
    int16_t* s = (int16_t*)pcm;
    for (int i = 0; i < samples; i++)
      txb[i] = ((int32_t)(s[i] * musicVolume / 100)) << 16;
    size_t written = 0;
    i2s_channel_write(musicTx, txb, samples * 4, &written, 0);
    size_t used = written / 2;
    vidAudioFed += used;
    if ((int)used < n) { vidAud.seek(vidAud.position() - (n - used)); return; }
  }
}

size_t vidReadFrame() {
  static uint8_t chunk[1024];
  int len = 0, prev = -1;
  bool inFrame = false;
  while (true) {
    int n = vidFile.read(chunk, sizeof(chunk));
    if (n <= 0) return 0;
    for (int i = 0; i < n; i++) {
      int c = chunk[i];
      if (!inFrame) {
        if (prev == 0xFF && c == 0xD8) { inFrame = true; vidFrameBuf[0] = 0xFF; vidFrameBuf[1] = 0xD8; len = 2; }
        prev = c;
        continue;
      }
      if (len < VID_FRAME_MAX) vidFrameBuf[len++] = (uint8_t)c;
      if (prev == 0xFF && c == 0xD9) { vidFile.seek(vidFile.position() - (n - i - 1)); return len; }
      prev = c;
    }
  }
}

void vidUpdate() {
  vidFeedAudio();
  uint32_t target = vidHasAudio
      ? (uint32_t)((vidAudioFed / (float)(MUSIC_SAMPLE_RATE * 2)) * VID_FPS)
      : (uint32_t)((millis() - vidStartMs) / 1000.0f * VID_FPS);
  if (vidFrameShown >= target) return;
  size_t len = vidReadFrame();
  if (len == 0) { vidStop(); drawVideoMenu(); return; }   // 播完回選單
  vidFrameShown++;
  if (target - vidFrameShown > 2) return;                  // 跳幀追時鐘
  if (vidJpeg.openRAM(vidFrameBuf, len, vidJpegDraw)) {
    int xoff = (240 - vidJpeg.getWidth())  / 2;
    int yoff = (320 - vidJpeg.getHeight()) / 2;
    if (xoff < 0) xoff = 0;
    if (yoff < 0) yoff = 0;
    vidJpeg.decode(xoff, yoff, 0);
    vidJpeg.close();
  }
}

void enterVideoMode() {
#if ENABLE_MUSIC
  musicStop();   // 影片聲音跟背景音樂共用 I²S,先停音樂
#endif
  if (!vidFrameBuf)
    vidFrameBuf = (uint8_t*)heap_caps_malloc(VID_FRAME_MAX, MALLOC_CAP_SPIRAM);
  vidCursor = 0;
  vidScanFiles();
  drawVideoMenu();
}

void exitVideoMode() {
  vidStop();
}

// 音量調整(播放中 ↑↓ 鍵)
void mediaVolume(int delta) {
  int v = (int)musicVolume + delta;
  if (v < 0)   v = 0;
  if (v > 100) v = 100;
  musicVolume = (uint8_t)v;
  drawVolOverlay();
}

// 暫停/繼續(→ 鍵)
void mediaTogglePause(bool hasAudioNow) {
  mediaPaused = !mediaPaused;
  if (mediaPaused) {
    vidPauseStart = millis();
    digitalWrite(PIN_AMP_SD, LOW);            // 靜音
  } else {
    vidStartMs += millis() - vidPauseStart;   // millis 時鐘補償暫停時間
    if (hasAudioNow) digitalWrite(PIN_AMP_SD, HIGH);
  }
}

void videoModeLoop(int btnEdge) {
  // --- 影片播放中 ---
  if (vidPlaying) {
    if (btnEdge == BTN_BACK)  { vidStop(); drawVideoMenu(); return; }     // ← 退出
    if (btnEdge == BTN_OK)    { mediaTogglePause(vidHasAudio); drawPauseState(); }  // → 暫停/繼續
    if (btnEdge == BTN_PLUS)  mediaVolume(+10);                            // ↑ 音量
    if (btnEdge == BTN_MINUS) mediaVolume(-10);                            // ↓ 音量
    if (!mediaPaused) vidUpdate();
    return;
  }
  // --- 音樂播放中(共用 ENABLE_MUSIC 的 musicPlay/musicUpdate)---
  if (musicPlaying) {
    if (btnEdge == BTN_BACK)  { musicStop(); mediaPaused = false; drawVideoMenu(); return; }
    if (btnEdge == BTN_OK)    { mediaTogglePause(true); drawPauseState(); }
    if (btnEdge == BTN_PLUS)  mediaVolume(+10);
    if (btnEdge == BTN_MINUS) mediaVolume(-10);
    if (!mediaPaused) {
      musicUpdate();
      if (!musicPlaying) { mediaPaused = false; drawVideoMenu(); }  // 播完回選單
    }
    return;
  }
  // --- 選單瀏覽 ---
  static unsigned long lastMove = 0;
  if (vidCount > 0 && millis() - lastMove > 250) {
    int p = centerMap(analogRead(J_PITCH), REV_PITCH);
    if (p > 240) { vidCursor = (vidCursor + 1) % vidCount;            drawVideoMenu(); lastMove = millis(); }
    if (p < 15)  { vidCursor = (vidCursor - 1 + vidCount) % vidCount; drawVideoMenu(); lastMove = millis(); }
  }
  if (btnEdge == BTN_OK && vidCount > 0) {
    if (vidIsVideo[vidCursor]) {
      vidStart(vidList[vidCursor]);
    } else {
      String path = String("/music/") + vidList[vidCursor];
      musicPlay(path.c_str());
      if (musicPlaying) drawMusicScreen(vidList[vidCursor]);
    }
  }
}
#endif  // ENABLE_VIDEO_MODE20


// ============================================================
// 常駐選單(任何 mode TFT 下方 30px 區域常駐顯示)
//
// 開啟:#define ENABLE_PERSISTENT_MENU 1
// 進入:肩鍵 L 長按 1 秒
// 操作:+ - 切換項目,OK 進入該功能,返回 退出
// 項目:音樂、亮度(預留,MSP2806 LED 直連 3V3 無法軟控)、系統資訊
//
// 設計上跟 mode 0 的 PID 選單分開:那是「設定」,常駐選單是「附屬功能」
// ============================================================
#if ENABLE_PERSISTENT_MENU
enum PMItem { PM_MUSIC, PM_BRIGHTNESS, PM_INFO, PM_COUNT };
const char* PM_NAMES[PM_COUNT] = { "音樂", "亮度", "系統資訊" };

bool          pmActive       = false;
int           pmCursor       = 0;
unsigned long pmShoulderDown = 0;   // 用來偵測長按

void persistentMenuUpdate() {
  bool shldL = (digitalRead(SHOULDER_L) == LOW);
  if (shldL) {
    if (pmShoulderDown == 0) pmShoulderDown = millis();
    else if (millis() - pmShoulderDown > 1000 && !pmActive) {
      pmActive = true;
      // TODO:暫停其他 UI 重畫,畫常駐選單
    }
  } else {
    pmShoulderDown = 0;
  }
  // pmActive 時 + - 改 cursor,OK 進入該功能,返回 退出
  // TODO
}

void persistentMenuDraw() {
  // 在 TFT 底部畫常駐選單
  // TODO:fillRect 區域 + 畫 PM_NAMES[pmCursor]
}
#endif


// ============================================================
// ============================================================
//        以上 feature 旗標都 0,以下空函式不會被呼叫
// ============================================================
// ============================================================
