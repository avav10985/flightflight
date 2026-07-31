#pragma once
#include <Arduino.h>
#include "LGFX_S3.h"
#include "packets.h"
#include "FlightLink.h"
#include "Inputs.h"
// ============================================================
// MainUi:飛行畫面 + PID 選單 + 校準/存檔 modal(直立 240×320)
// static 一次畫底色 + dynamic 文字覆寫,只重畫有變的部分避免閃爍
// ============================================================

// PID 本地副本(開機與飛機預設一致)
// isAction = true → 動作項目(校準/存檔),OK 觸發、不顯示數值
// 動作項目用 paramVal 當 pulse counter,每按一次 +1,飛機 dedup 看 (id,val) 變化才動作
struct Param { const char* name; float val; float step; uint8_t id; bool isAction; };

class MainUi {
public:
  MainUi(LGFX& tft, FlightLink& link) : tft_(tft), link_(link) {}

  bool inMenu() const { return ui_ == UI_MENU; }

  void showFlight();   // 回飛行畫面(從選單/其他 mode 返回時)
  void showMenu();     // 進 PID 選單

  // UI 狀態機(每圈呼叫):選單開關、游標、按鈕。
  // 回傳本地動作 id(200 = 音樂切換,由 .ino 處理),0 = 無。
  int update(uint8_t mode, int btnEdge, Inputs& in, Signal& data);

  // 校準提示 modal(手把觸發時刻 + 預估阻塞時間驅動,見 .cpp 註解)
  void showCalPrompt(const char* name, unsigned long durMs);

  // 開機收到第一筆有效遙測後,把選單值同步成飛機的實際 PID(一次性)
  void syncPidOnce();

  // 過了參數重送窗口才清 paramID(期間每包連送,抗丟包)
  void maintainParam(Signal& data) {
    if (millis() > paramHoldUntil_) data.paramID = 0;
  }

  // 畫一幀(呼叫端控制 150ms 節流):modal 優先,否則飛行/選單
  void draw(uint8_t mode, const Signal& data);

private:
  enum UiState { UI_FLIGHT, UI_MENU };

  void handleMenuCursor(Inputs& in);
  int  handleMenuButton(int btn, Signal& data);
  void drawFlightStatic();
  void drawFlightDynamic(uint8_t mode, const Signal& data);
  void drawMenuStatic();
  void drawMenuDynamic();
  bool drawCalOverlay();   // true = modal 正佔畫面,跳過一般繪製

  LGFX&       tft_;
  FlightLink& link_;

  UiState ui_ = UI_FLIGHT;
  int     menuCursor_ = 0;

  // 2026-06-09 跟飛機 Drone_FC_Full(commit fe466f0)同步降到保守值
  // 避免一進選單就送舊高值給飛機 → 飛機 PID 被覆蓋成激進值
  static const int N_PARAM = 9;
  Param params_[N_PARAM] = {
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

  // 全 redraw flag:drawXxxStatic 設 true,dynamic 第一次跑會強制重畫所有
  // (從 PID 選單返回後 mode 沒變但畫面被清掉,沒這個會空一片)
  bool flightDirty_ = false;
  bool menuDirty_   = false;

  // ---- 校準畫面提示 ----
  // FC 校準時 sampleIMU() 會阻塞 0.3~1.5 秒,期間不回 telemetry,所以「校準中」
  // 不能等 telemetry 的 STATUS_CALIBRATING(手把根本收不到那個瞬間),改由手把
  // 「自己送出校準觸發」的時刻驅動;校準中結束後再讀 STATUS_CAL_FAILED 判成敗。
  unsigned long calBusyStart_     = 0;    // 校準開始 millis(算進度條用)
  unsigned long calBusyUntil_     = 0;    // 顯示「校準中」到此 millis
  const char*   calBusyName_      = "";   // "快速校準" / "完整校準"
  bool          calResultPending_ = false; // 校準中結束後,要顯示一次成敗

  // 參數重送窗口:調 PID / 觸發動作後,持續送同一個 paramID 一小段時間,
  // 避免 nRF24 偶爾丟一包就漏掉(FC 端 lastID/lastVal 去重,重送只會套用一次)
  unsigned long paramHoldUntil_ = 0;

  // PID:開機收到第一筆有效遙測後同步一次,之後手把端為主
  bool          pidSynced_    = false;
  unsigned long pidSaveUntil_ = 0;   // 顯示「PID 已存」確認屏到此 millis
};
