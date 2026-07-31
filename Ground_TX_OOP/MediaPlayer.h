#pragma once
#include "config.h"
#if ENABLE_MUSIC
#include <Arduino.h>
#include <SD.h>
#include <driver/i2s_std.h>   // 音樂播放 I²S TX
#include "LGFX_S3.h"
#include "Inputs.h"
#if ENABLE_VIDEO_MODE12
#include <JPEGDEC.h>          // Library Manager 搜 JPEGDEC(Larry Bank)
#endif
// ============================================================
// MediaPlayer:音樂播放(SD /music/*.WAV → I²S → MAX98357A)
//              + Mode 12 媒體模式(SD /video/*.mjp MJPEG 影片)
//
// I²S 用 i2s_std 直驅(沿用 Play_Test / Video_Test 驗證過的設定);
// ENABLE_VOICE_MODE10 時同 port 開全雙工,micHandle() 給語音錄音用。
// 優先級:警示音 > Mode 10 語音回應 > 背景音樂
// ============================================================
#define MUSIC_SAMPLE_RATE 32000  // WAV 需轉成 32kHz / 單聲道 / 16-bit PCM

#if ENABLE_VIDEO_MODE12
#define VID_FPS        15.0f
#define VID_FRAME_MAX  (160 * 1024)
#define VID_MAX_FILES  16
#endif

class MediaPlayer {
public:
  MediaPlayer(LGFX& tft);

  void beginI2S();                     // setup() 呼叫(SD init 之後)
  void setSdOk(bool ok) { sdOk_ = ok; }

  // ---- 音樂 ----
  void musicPlay(const char* path);
  void musicStop();
  void musicUpdate();                  // 每迴圈呼叫:非阻塞餵 I²S,DMA 滿了立刻返回
  void musicToggle();                  // 選單「音樂」項目:播放 ⇄ 停止
  bool isMusicPlaying() const { return musicPlaying_; }
  int  musicListFiles(String list[], int maxN);   // 列 SD /music/ 的 .WAV

#if ENABLE_VOICE_MODE10
  i2s_chan_handle_t micHandle() { return micRx_; }   // INMP441 RX(語音錄音用)
#endif

#if ENABLE_VIDEO_MODE12
  // ---- Mode 12 媒體模式 ----
  // 檔案:/video/*.mjp(MJPEG)+ 同名 .wav 聲音(可無 → 無聲播放)
  // 轉檔:ffmpeg -i in.mp4 -vf "scale=240:320:force_original_aspect_ratio=decrease:force_divisible_by=2,fps=15" -q:v 8 -f mjpeg xxx.mjp
  //       ffmpeg -i in.mp4 -ar 32000 -ac 1 -sample_fmt s16 xxx.wav
  // 操作:右搖桿選檔 → OK 播放 → 返回/OK 停止 → 模式開關撥走立刻退出
  // 同步:聲音當時鐘,解碼落後 >2 幀跳幀
  void enterVideoMode();
  void exitVideoMode();
  void videoModeLoop(int btnEdge, Inputs& in);
#endif

private:
  LGFX& tft_;
  bool  sdOk_ = false;

  // ---- 音樂 ----
  File     musicFile_;
  bool     musicPlaying_ = false;
  uint8_t  musicVolume_  = 60;     // 0~100
  i2s_chan_handle_t musicTx_ = nullptr;
#if ENABLE_VOICE_MODE10
  i2s_chan_handle_t micRx_   = nullptr;   // INMP441,跟喇叭全雙工共 BCLK/WS
#endif

#if ENABLE_VIDEO_MODE12
  static int jpegDrawCb(JPEGDRAW* d);     // JPEGDEC C callback → s_self->tft_
  void   vidScanFiles();
  void   drawVideoMenu();
  void   drawVolOverlay();
  void   drawMusicScreen(const String& name);
  void   drawPauseState();
  void   vidStop();
  void   vidStart(const String& name);
  void   vidFeedAudio();
  size_t vidReadFrame();
  void   vidUpdate();
  void   mediaVolume(int delta);
  void   mediaTogglePause(bool hasAudioNow);

  JPEGDEC  vidJpeg_;
  File     vidFile_, vidAud_;
  uint8_t* vidFrameBuf_  = nullptr;
  bool     vidPlaying_   = false;
  bool     vidHasAudio_  = false;
  uint64_t vidAudioFed_  = 0;
  uint32_t vidFrameShown_ = 0;
  unsigned long vidStartMs_ = 0;
  String   vidList_[VID_MAX_FILES];
  bool     vidIsVideo_[VID_MAX_FILES];   // true=/video/*.mjp,false=/music/*.wav
  int      vidCount_  = 0;
  int      vidCursor_ = 0;
  int      vidScroll_ = 0;   // 清單捲動視窗起點(一頁 7 列)
  bool     mediaPaused_ = false;
  unsigned long vidPauseStart_ = 0;
#endif
};
#endif  // ENABLE_MUSIC
