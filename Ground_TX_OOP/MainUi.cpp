#include "MainUi.h"
#include "config.h"

void MainUi::showFlight() {
  ui_ = UI_FLIGHT;
  drawFlightStatic();
}

void MainUi::showMenu() {
  ui_ = UI_MENU;
  drawMenuStatic();
}

int MainUi::update(uint8_t mode, int btnEdge, Inputs& in, Signal& data) {
  if (ui_ == UI_FLIGHT) {
    if (btnEdge == BTN_OK && mode == 0) showMenu();
  } else {  // UI_MENU
    if (mode != 0) {
      showFlight();
    } else {
      handleMenuCursor(in);
      if (btnEdge != BTN_NONE) return handleMenuButton(btnEdge, data);
    }
  }
  return 0;
}

void MainUi::showCalPrompt(const char* name, unsigned long durMs) {
  calBusyName_      = name;
  calBusyStart_     = millis();
  calBusyUntil_     = millis() + durMs;
  calResultPending_ = true;
}

// 把選單 PID(前 5 列)同步成飛機回傳的實際值。只在開機第一筆有效遙測時做一次,
// 之後手把端為準(使用者調的值送給飛機,不再被回傳覆蓋)。
void MainUi::syncPidOnce() {
  if (pidSynced_) return;
  const Telemetry& tele = link_.telemetry();
  // 防呆:飛機若還是舊韌體(沒回傳 PID),欄位會是 0/垃圾 → Kp 不可能 ≤0,跳過不同步
  if (tele.kp_rp_m <= 0 || tele.kp_rp_m > 30000) return;
  params_[0].val = tele.kp_rp_m / 1000.0f;   // Kp
  params_[1].val = tele.ki_rp_m / 1000.0f;   // Ki
  params_[2].val = tele.kd_rp_m / 1000.0f;   // Kd
  params_[3].val = tele.kp_y_m  / 1000.0f;   // KpY
  params_[4].val = tele.ki_y_m  / 1000.0f;   // KiY
  pidSynced_ = true;
  menuDirty_ = true;   // 選單若正開著,強制重畫成新值
}

void MainUi::handleMenuCursor(Inputs& in) {
  // 2026-06-07 修:推下選下、推上選上(用 REV_PITCH 跟飛行 mode 同方向)
  // 死區 55/200 改成 15/240(搖桿中位偏離時不誤觸)
  static unsigned long lastMove = 0;
  if (millis() - lastMove < 250) return;
  int p = in.pitchPos();   // 跟 data.pitch 用同方向
  if (p > 240) { menuCursor_ = (menuCursor_ + 1) % N_PARAM;            lastMove = millis(); }
  if (p < 15)  { menuCursor_ = (menuCursor_ - 1 + N_PARAM) % N_PARAM;  lastMove = millis(); }
}

int MainUi::handleMenuButton(int btn, Signal& data) {
  if (btn == BTN_BACK) { showFlight(); return 0; }
  Param &p = params_[menuCursor_];
  if (p.isAction) {
    // 動作項目:只認 OK,每按一次 paramVal +1 當 pulse,飛機看 val 變才會做
    if (btn == BTN_OK) {
#if ENABLE_MUSIC
      if (p.id == 200) return 200;   // 本地動作(音樂切換),.ino 處理,不送飛機
#endif
      p.val += 1.0f;
      data.paramID  = p.id;
      data.paramVal = p.val;
      paramHoldUntil_ = millis() + 400;   // 重送窗口,避免丟包漏掉
      if (p.id == 100) showCalPrompt("快速校準", 700);    // FC 阻塞 ~0.3s
      if (p.id == 101) showCalPrompt("完整校準", 2200);   // FC 阻塞 ~1.5s + 餘裕
      if (p.id == 102) pidSaveUntil_ = millis() + 2800;   // 顯示「PID 已存」確認屏
    }
  } else {
    if (btn == BTN_PLUS)  p.val += p.step;
    if (btn == BTN_MINUS) p.val -= p.step;
    if (p.val < 0) p.val = 0;
    data.paramID  = p.id;
    data.paramVal = p.val;
    paramHoldUntil_ = millis() + 400;     // 重送窗口,避免丟包漏掉
  }
  return 0;
}

void MainUi::draw(uint8_t mode, const Signal& data) {
  if (drawCalOverlay()) {
    // 校準提示 modal 正佔畫面,跳過一般繪製
  } else if (ui_ == UI_FLIGHT) {
    drawFlightDynamic(mode, data);
  } else {
    drawMenuDynamic();
  }
}

// ============================================================
// 以下繪製實作,邏輯與原版逐行一致
// ============================================================
void MainUi::drawFlightStatic() {
  tft_.fillScreen(TFT_BLACK);
  tft_.fillRect(0,   0, 240, 32, TFT_NAVY);     // 頂部標題列
  tft_.fillRect(0, 280, 240, 40, TFT_DARKGREY); // 底部狀態列

  // 三個姿態 label 預先畫上(不會變,不用每次重畫)
  tft_.setFont(&fonts::efontTW_24);
  tft_.setTextColor(TFT_WHITE, TFT_BLACK);
  tft_.setCursor(10, 50);  tft_.print("翻滾");
  tft_.setCursor(10, 130); tft_.print("俯仰");
  tft_.setCursor(10, 210); tft_.print("偏航");

  flightDirty_ = true;   // 強制下一次 drawFlightDynamic 重畫所有 cached 部分
}

void MainUi::drawFlightDynamic(uint8_t mode, const Signal& data) {
  char buf[40];
  const Telemetry& tele = link_.telemetry();
  bool armed = tele.status & STATUS_ARMED;

  // === 頂部 mode 文字:只在 mode 改變時重畫 ===
  static uint8_t lastModeDrawn = 255;
  if (mode != lastModeDrawn || flightDirty_) {
    tft_.fillRect(0, 0, 175, 32, TFT_NAVY);   // 先清舊文字(留 165 給已武/安全)
    tft_.setFont(&fonts::efontTW_24);
    tft_.setTextColor(TFT_WHITE, TFT_NAVY);
    tft_.setCursor(5, 4);
    snprintf(buf, sizeof(buf), "M%02d %s", mode, getModeName(mode));
    tft_.print(buf);
    lastModeDrawn = mode;
  }

  // === 武裝狀態:只在改變時重畫 ===
  static bool lastArmed = false;
  static bool armedInit = false;
  if (armed != lastArmed || !armedInit || flightDirty_) {
    tft_.fillRect(175, 0, 65, 32, TFT_NAVY);
    tft_.setFont(&fonts::efontTW_24);
    tft_.setTextColor(armed ? TFT_RED : TFT_LIGHTGREY, TFT_NAVY);
    tft_.setCursor(180, 4);
    tft_.print(armed ? "已武" : "安全");
    lastArmed = armed;
    armedInit = true;
  }

  // === 主體:已實作/未實作 切換時清主體 + 重畫 label ===
  static uint8_t lastBlock = 255;
  uint8_t block = modeImplemented(mode) ? 1 : 0;
  if (block != lastBlock || flightDirty_) {
    tft_.fillRect(0, 34, 240, 244, TFT_BLACK);
    if (modeImplemented(mode)) {
      // 把 label 補回去(static 第一次有畫,主體被清掉後再補)
      tft_.setFont(&fonts::efontTW_24);
      tft_.setTextColor(TFT_WHITE, TFT_BLACK);
      tft_.setCursor(10, 50);  tft_.print("翻滾");
      tft_.setCursor(10, 130); tft_.print("俯仰");
      tft_.setCursor(10, 210); tft_.print("偏航");
    } else {
      tft_.setFont(&fonts::efontTW_24);
      tft_.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft_.setCursor(40, 140); tft_.print("此模式");
      tft_.setCursor(40, 175); tft_.print("未實作");
    }
    lastBlock = block;
    flightDirty_ = true;   // 主體被清過,下面值檢查強制重畫(否則 tele 沒變就空白)
  }

  // === 姿態數值:只在改變時重畫 ===
  if (modeImplemented(mode)) {
    static int16_t lastRoll = -32000, lastPitch = -32000, lastYaw = -32000;
    tft_.setFont(&fonts::efontTW_24);
    tft_.setTextColor(TFT_WHITE, TFT_BLACK);
    // 重畫門檻(deg×10):靜止時 ±0.x° 感測雜訊不重畫,不閃也不延遲真實值
    if (abs(tele.roll - lastRoll) >= 8 || flightDirty_) {
      tft_.fillRect(10, 80, 200, 30, TFT_BLACK);
      tft_.setCursor(10, 80);
      snprintf(buf, sizeof(buf), "%+7.1f °", tele.roll / 10.0);
      tft_.print(buf);
      lastRoll = tele.roll;
    }
    if (abs(tele.pitch - lastPitch) >= 8 || flightDirty_) {
      tft_.fillRect(10, 160, 200, 30, TFT_BLACK);
      tft_.setCursor(10, 160);
      snprintf(buf, sizeof(buf), "%+7.1f °", tele.pitch / 10.0);
      tft_.print(buf);
      lastPitch = tele.pitch;
    }
    if (abs(tele.yawRate - lastYaw) >= 20 || flightDirty_) {
      tft_.fillRect(10, 240, 230, 30, TFT_BLACK);
      tft_.setCursor(10, 240);
      snprintf(buf, sizeof(buf), "%+7.1f °/秒", tele.yawRate / 10.0);
      tft_.print(buf);
      lastYaw = tele.yawRate;
    }
  }
  flightDirty_ = false;   // 消耗 dirty 旗標

  // 底部狀態列(三行,塞 GPS / 高度 / 方位 / 油門 / 連線)
  tft_.setFont(&fonts::efontTW_14);
  tft_.setTextColor(TFT_WHITE, TFT_DARKGREY);
  // 第 1 行:油門 + 高度
  snprintf(buf, sizeof(buf), "油門%3d%%  高度%+5.1fm",
           (int)(data.throttle * 100 / 255),
           tele.altitude_dm / 10.0);
  tft_.setCursor(5, 283);
  tft_.print(buf);
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
  tft_.setCursor(5, 300);
  tft_.print(buf);
}

void MainUi::drawMenuStatic() {
  tft_.fillScreen(TFT_BLACK);
  tft_.fillRect(0,   0, 240, 32, TFT_PURPLE);
  tft_.fillRect(0, 280, 240, 40, TFT_DARKGREY);

  tft_.setFont(&fonts::efontTW_24);
  tft_.setTextColor(TFT_WHITE, TFT_PURPLE);
  tft_.setCursor(40, 4);
  tft_.print("═ PID 設定 ═");

  tft_.setFont(&fonts::efontTW_14);
  tft_.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft_.setCursor(5, 285);
  tft_.print("右搖桿:游標  +/-:調值");
  tft_.setCursor(5, 302);
  tft_.print("OK:送出    返回:離開");
  menuDirty_ = true;   // 進選單後第一次 drawMenuDynamic 強制畫所有列
}

void MainUi::drawMenuDynamic() {
  // 只重畫狀態變過的列(cursor 跳走 / 值改了),避免閃爍
  static int   lastCursor = -1;
  static float lastVal[N_PARAM] = { -999, -999, -999, -999, -999, -999, -999, -999, -999 };
  char buf[24];
  const int rowH = 26;     // 9 列要塞進 40~280 px(9×26=234),行高再壓低

  for (int i = 0; i < N_PARAM; i++) {
    bool sel    = (i == menuCursor_);
    bool wasSel = (i == lastCursor);
    bool valChanged = (lastVal[i] != params_[i].val);

    if (sel != wasSel || valChanged || lastCursor == -1 || menuDirty_) {
      int y = 40 + i * rowH;
      uint16_t bg = sel ? TFT_DARKGREEN : TFT_BLACK;
      uint16_t fg = sel ? TFT_YELLOW    : TFT_WHITE;
      tft_.fillRect(0, y, 240, rowH, bg);
      tft_.setFont(&fonts::efontTW_24);
      tft_.setTextColor(fg, bg);
      tft_.setCursor(10, y + 1);
      tft_.print(sel ? ">" : " ");
      tft_.print(params_[i].name);
      tft_.setCursor(160, y + 1);
      if (params_[i].isAction) {
        tft_.print("GO");
      } else {
        snprintf(buf, sizeof(buf), "%6.3f", params_[i].val);
        tft_.print(buf);
      }
      lastVal[i] = params_[i].val;
    }
  }
  lastCursor = menuCursor_;
  menuDirty_ = false;
}

// 全畫面 modal(校準提示 / PID 存檔確認):回傳 true 代表 modal 正佔用畫面,
// 呼叫端應跳過一般繪製。結束後自動還原底層畫面。
bool MainUi::drawCalOverlay() {
  static uint8_t shown = 0;            // 0=無 modal、1=校準中、2=校準結果、3=PID已存
  static unsigned long resultUntil = 0;
  const Telemetry& tele = link_.telemetry();

  // --- PID 已存確認(讀飛機回傳的實際 PID 顯示)---
  if (millis() < pidSaveUntil_) {
    if (shown != 3) {
      bool live = link_.linkFresh();
      char b[44];
      tft_.fillScreen(TFT_DARKGREEN);
      tft_.setFont(&fonts::efontTW_24);
      tft_.setTextColor(TFT_WHITE, TFT_DARKGREEN);
      tft_.setCursor(25, 25); tft_.print("PID 已存飛機");
      tft_.setFont(&fonts::efontTW_16);
      if (live) {
        snprintf(b, sizeof(b), "Kp  %.3f", tele.kp_rp_m / 1000.0f); tft_.setCursor(20, 85);  tft_.print(b);
        snprintf(b, sizeof(b), "Ki  %.3f", tele.ki_rp_m / 1000.0f); tft_.setCursor(20, 115); tft_.print(b);
        snprintf(b, sizeof(b), "Kd  %.3f", tele.kd_rp_m / 1000.0f); tft_.setCursor(20, 145); tft_.print(b);
        snprintf(b, sizeof(b), "KpY %.3f", tele.kp_y_m  / 1000.0f); tft_.setCursor(20, 175); tft_.print(b);
        snprintf(b, sizeof(b), "KiY %.3f", tele.ki_y_m  / 1000.0f); tft_.setCursor(20, 205); tft_.print(b);
      } else {
        tft_.setCursor(20, 120); tft_.print("飛機未連線,無法確認值");
      }
      shown = 3;
    }
    return true;
  }

  // --- 校準中(由手把觸發時刻 + 預估阻塞時間驅動)+ 進度條 ---
  if (millis() < calBusyUntil_) {
    if (shown != 1) {
      tft_.fillScreen(TFT_NAVY);
      tft_.setFont(&fonts::efontTW_24);
      tft_.setTextColor(TFT_YELLOW, TFT_NAVY);
      tft_.setCursor(55, 70); tft_.print("校準中…");
      tft_.setFont(&fonts::efontTW_16);
      tft_.setTextColor(TFT_WHITE, TFT_NAVY);
      tft_.setCursor(20, 115); tft_.print(calBusyName_);
      tft_.setFont(&fonts::efontTW_14);
      tft_.setCursor(20, 150); tft_.print("機體放平、保持不動");
      tft_.drawRect(20, 195, 200, 24, TFT_WHITE);   // 進度條外框
      shown = 1;
    }
    // 進度條填充:每次重繪更新(觸發→結束的時間比例),只增不減免清背景
    unsigned long total = calBusyUntil_ - calBusyStart_;
    long w = total ? (long)(196 * (millis() - calBusyStart_) / total) : 196;
    if (w < 0) w = 0; if (w > 196) w = 196;
    tft_.fillRect(22, 197, (int)w, 20, TFT_GREEN);
    return true;
  }

  // --- 校準中剛結束 → 讀 telemetry 顯示一次結果 ---
  // 三態:飛機沒連線(收不到遙測)→ 不能說完成;有連線再分成功/失敗
  if (calResultPending_) {
    calResultPending_ = false;
    bool linkAlive = link_.linkFresh();   // 校準視窗結束時應已有新鮮遙測
    bool failed    = tele.status & STATUS_CAL_FAILED;
    tft_.setFont(&fonts::efontTW_24);
    tft_.setTextColor(TFT_WHITE);
    if (!linkAlive) {
      tft_.fillScreen(TFT_DARKGREY);
      tft_.setCursor(30, 95);  tft_.print("飛機未連線");
      tft_.setFont(&fonts::efontTW_14);
      tft_.setCursor(15, 145); tft_.print("沒收到回應,校準未確認");
    } else if (failed) {
      tft_.fillScreen(TFT_MAROON);
      tft_.setCursor(45, 95);  tft_.print("校準失敗");
      tft_.setFont(&fonts::efontTW_14);
      tft_.setCursor(15, 145); tft_.print("偵測到晃動,放平後重試");
    } else {
      tft_.fillScreen(TFT_DARKGREEN);
      tft_.setCursor(45, 95);  tft_.print("校準完成");
    }
    shown = 2;
    resultUntil = millis() + 2500;
    return true;
  }
  if (shown == 2 && millis() < resultUntil) return true;

  // --- 沒有 modal 該顯示:若剛從 modal 退出,還原底層畫面 ---
  if (shown != 0) {
    if (ui_ == UI_FLIGHT) drawFlightStatic(); else drawMenuStatic();
    shown = 0;
  }
  return false;
}
