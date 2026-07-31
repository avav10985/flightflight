#pragma once
#include "config.h"
#if ENABLE_VOICE_MODE10
#include <Arduino.h>
#include <driver/i2s_std.h>
#include "LGFX_S3.h"
#include "packets.h"
// ============================================================
// VoiceAssistant:Mode 10 AI 語音控制(2026-06-12 Mode10_Test 管線移植)
//
// 架構:語音管線(錄音→WiFi→Groq Whisper→Llama→action)跑在
//   核心 0 的獨立 task;主迴圈(核心 1)50Hz 控制完全不中斷。
//   WiFi(≤2.472GHz)與 NRF CH88(2.488GHz)頻率錯開可共存。
//
// 虛擬搖桿:action 在主迴圈 applyOverride() 轉成 Signal 覆寫。
// 安全:搖桿一動立刻搶回;move 限時;模式開關最高優先。
//
// 需要 secrets.h(WIFI_SSID / WIFI_PASS / GROQ_API_KEY,gitignore 擋住)
// ============================================================
class VoiceAssistant {
public:
  VoiceAssistant(LGFX& tft) : tft_(tft) {}

  // 錄音緩衝(PSRAM)+ 語音 task 釘在核心 0。false = PSRAM 配置失敗。
  // micRx 來自 MediaPlayer::micHandle()(同 I²S port 全雙工的 RX 通道)。
  bool begin(i2s_chan_handle_t micRx);

  void enter();               // 進 mode 10(呼叫端先 musicStop)
  void exit();
  void setPTT(bool on) { ptt_ = on; }   // 左肩鈕 = PTT(主迴圈每圈更新)

  // 語音動作覆寫虛擬搖桿(主迴圈端,fillSignal 之後呼叫)
  void applyOverride(Signal& data);

  // 取走待播語音回應 id(1起飛 2降落 3停 4~7馬達1-4 8聽不懂,0=無)
  // 呼叫端用音樂管線非阻塞播 SD 上的預錄句
  uint8_t takeSpeech() { uint8_t id = speak_; speak_ = 0; return id; }

  // Mode 10 畫面更新(狀態字串 / 虛擬搖桿程式 / 2Hz 遙測)
  void updateUi(const Telemetry& tele, bool teleOK, uint8_t throttle);

private:
  enum VProg { VP_NONE, VP_TAKEOFF, VP_HOVER, VP_MOVE, VP_LAND };

  static void taskEntry(void* arg);
  void taskLoop();            // 語音 task 主體(核心 0)
  void drawVoiceStatic();
  const char* progName();
  void setStatus(const char* s);
  String transcribe();        // PCM → WAV → Groq Whisper STT
  String parseLlama(const String& userText);   // 文字 → Llama → action JSON
  void emitAction(const String& json, const String& transcript);

  LGFX& tft_;
  i2s_chan_handle_t micRx_ = nullptr;

  // ---- 主迴圈(核心1)↔ 語音 task(核心0)共享變數,單寫者原則 ----
  volatile bool    mode10_ = false;  // 主迴圈寫:目前在 mode 10
  volatile bool    ptt_    = false;  // 主迴圈寫:左肩鈕按住中
  volatile uint8_t action_ = 0;      // task 寫:0無 1takeoff 2land 3stop 4move 5spin
  volatile int8_t  dirX_ = 0, dirY_ = 0, dirZ_ = 0;   // move 方向(roll/pitch/throttle)
  volatile uint8_t dur_   = 0;       // move 秒數
  volatile uint8_t motor_ = 1;       // spin 動作:轉哪顆馬達(1~4)
  volatile uint8_t speak_ = 0;       // 語音回應 id(見 takeSpeech)
  volatile bool    fresh_ = false;   // task 寫 true → 主迴圈消化後寫 false
  char             status_[48] = "開機中";   // task 寫狀態字串(主迴圈顯示)

  int16_t* recBuf_   = nullptr;      // PSRAM 錄音緩衝(begin 配置)
  uint32_t recCount_ = 0;

  // ---- 虛擬搖桿程式(主迴圈端)----
  VProg         prog_    = VP_NONE;
  unsigned long progT0_  = 0;
  uint8_t       moveDur_ = 0;
  int8_t        mx_ = 0, my_ = 0, mz_ = 0;
  uint8_t       baseThr_ = 0, basePitch_ = 127, baseRoll_ = 127;
};
#endif  // ENABLE_VOICE_MODE10
