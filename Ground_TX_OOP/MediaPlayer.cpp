#include "MediaPlayer.h"
#if ENABLE_MUSIC

#if ENABLE_VIDEO_MODE12
// JPEGDEC 的 draw callback 是 C 函式指標,拿不到 this → 用單例指標轉接
static MediaPlayer* s_self = nullptr;
#endif

MediaPlayer::MediaPlayer(LGFX& tft) : tft_(tft) {
#if ENABLE_VIDEO_MODE12
  s_self = this;
#endif
}

// ============================================================
// I²S TX 初始化(沿用 Play_Test 驗證過的設定)
// ============================================================
void MediaPlayer::beginI2S() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  // 主程式 loop 比 Play_Test 忙(NRF24 重試 + TFT 重繪),緩衝拉到 ~190ms
  // 6 × 1023 frames × 4B ≈ 24KB 內部 RAM,換 6138 samples 餘裕
  chan_cfg.dma_desc_num  = 6;
  chan_cfg.dma_frame_num = 1023;
#if ENABLE_VOICE_MODE10
  i2s_new_channel(&chan_cfg, &musicTx_, &micRx_);   // 同 port 全雙工(共 BCLK/WS)
#else
  i2s_new_channel(&chan_cfg, &musicTx_, NULL);
#endif

  i2s_std_config_t std_cfg = {};
  std_cfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(MUSIC_SAMPLE_RATE);
  std_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
  std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  std_cfg.gpio_cfg.bclk = (gpio_num_t)PIN_I2S_BCLK;
  std_cfg.gpio_cfg.ws   = (gpio_num_t)PIN_I2S_WS;
  std_cfg.gpio_cfg.dout = (gpio_num_t)PIN_I2S_DOUT;
  std_cfg.gpio_cfg.din  = I2S_GPIO_UNUSED;
  i2s_channel_init_std_mode(musicTx_, &std_cfg);
  i2s_channel_enable(musicTx_);

#if ENABLE_VOICE_MODE10
  // 麥克風 RX:同時脈(32kHz / 32-bit),DIN = INMP441 SD
  i2s_std_config_t rx_cfg = std_cfg;
  rx_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
  rx_cfg.gpio_cfg.din  = (gpio_num_t)PIN_I2S_DIN;
  // 關鍵:TX 是雙 slot(喇叭 L+R),RX 必須只收 LEFT(INMP441 L/R 接地)。
  // 繼承 BOTH 會讓錄音 sample 之間穿插垃圾 → Whisper 聽到雜訊亂辨識
  rx_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
  i2s_channel_init_std_mode(micRx_, &rx_cfg);
  // 不在這裡 enable:錄音開始才 enable、結束 disable(避免 DMA 堆舊資料)
#endif

  // 預灌 silence 讓 MAX98357A 醒來看到穩定 0 訊號(boot pop 消除)
  int32_t silence[256] = {0};
  size_t w = 0;
  for (int i = 0; i < 4; i++)
    i2s_channel_write(musicTx_, silence, sizeof(silence), &w, 50 / portTICK_PERIOD_MS);
}

// ============================================================
// 音樂
// ============================================================
void MediaPlayer::musicPlay(const char* path) {
  if (musicPlaying_) { musicFile_.close(); }
  musicFile_ = SD.open(path, FILE_READ);
  if (!musicFile_) { Serial.printf("[!] 開不了 %s\n", path); return; }
  musicFile_.seek(44);   // 跳 WAV header
  musicPlaying_ = true;
  digitalWrite(PIN_AMP_SD, HIGH);   // 開擴大機
  Serial.printf("[+] 播放 %s\n", path);
}

void MediaPlayer::musicStop() {
  if (musicPlaying_) { musicFile_.close(); musicPlaying_ = false; }
  digitalWrite(PIN_AMP_SD, LOW);    // 關擴大機(靜音 + 消雜訊)
}

// 每迴圈呼叫:非阻塞地把 SD 資料餵進 I²S DMA。
// timeout 0:DMA 滿了就立刻返回,絕不卡住 50Hz 控制迴圈。
void MediaPlayer::musicUpdate() {
  if (!musicPlaying_) return;
  static uint8_t pcm[512];
  static int32_t tx[256];
  for (int round = 0; round < 16; round++) {    // 每迴圈最多餵 4096 samples(128ms 音訊);
                                                 // DMA 滿會提前 return,實際讀 SD 時間只 2~4ms
    int n = musicFile_.read(pcm, sizeof(pcm));
    if (n <= 0) {                                // 播完
      Serial.println("[+] 音樂播畢");
      musicStop();
      return;
    }
    int samples = n / 2;
    int16_t* s = (int16_t*)pcm;
    for (int i = 0; i < samples; i++)
      tx[i] = ((int32_t)(s[i] * musicVolume_ / 100)) << 16;
    size_t written = 0;
    i2s_channel_write(musicTx_, tx, samples * 4, &written, 0);   // 非阻塞
    size_t consumedPcm = written / 2;            // tx bytes 是 pcm bytes 的 2 倍
    if ((int)consumedPcm < n) {
      // DMA 滿:把沒寫進去的部分倒帶,下迴圈再餵
      musicFile_.seek(musicFile_.position() - (n - consumedPcm));
      return;
    }
  }
}

// 選單「音樂」項目:播放 ⇄ 停止
void MediaPlayer::musicToggle() {
  if (musicPlaying_) { musicStop(); return; }
  if (!sdOk_) { Serial.println("[!] SD 沒掛載,不能放音樂"); return; }
  // 先找 /music/ 第一首,沒有就退回根目錄 REC_001.WAV
  String list[1];
  if (musicListFiles(list, 1) > 0) {
    String path = String("/music/") + list[0];
    musicPlay(path.c_str());
  } else {
    musicPlay("/REC_001.WAV");
  }
}

// 列出 SD /music/ 資料夾的所有 .WAV 檔
int MediaPlayer::musicListFiles(String list[], int maxN) {
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

// ============================================================
// Mode 12:媒體模式(SD 影片播放,Video_Test 2026-06-12 移植)
// ============================================================
#if ENABLE_VIDEO_MODE12
int MediaPlayer::jpegDrawCb(JPEGDRAW* d) {
  s_self->tft_.pushImage(d->x, d->y, d->iWidth, d->iHeight, (uint16_t*)d->pPixels);
  return 1;
}

void MediaPlayer::vidScanFiles() {
  vidCount_ = 0;
  // 影片:/video/*.mjp
  File dir = SD.open("/video");
  if (dir) {
    while (vidCount_ < VID_MAX_FILES) {
      File f = dir.openNextFile();
      if (!f) break;
      String name = f.name();
      if (name.endsWith(".mjp") || name.endsWith(".MJP")) {
        vidIsVideo_[vidCount_] = true;
        vidList_[vidCount_++]  = name;
      }
      f.close();
    }
    dir.close();
  }
  // 音樂:/music/*.wav
  dir = SD.open("/music");
  if (dir) {
    while (vidCount_ < VID_MAX_FILES) {
      File f = dir.openNextFile();
      if (!f) break;
      String name = f.name();
      if (name.endsWith(".wav") || name.endsWith(".WAV")) {
        vidIsVideo_[vidCount_] = false;
        vidList_[vidCount_++]  = name;
      }
      f.close();
    }
    dir.close();
  }
}

void MediaPlayer::drawVideoMenu() {
  tft_.fillScreen(TFT_BLACK);
  tft_.fillRect(0, 0, 240, 32, TFT_NAVY);
  tft_.setFont(&fonts::efontTW_24);
  tft_.setTextColor(TFT_WHITE, TFT_NAVY);
  tft_.setCursor(30, 4);
  tft_.print("Mode 12 媒體");
  if (vidCount_ == 0) {
    tft_.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft_.setCursor(10, 100);
    tft_.print("SD /video /music 都空");
    return;
  }
  // 捲動視窗:一頁 7 列,游標超出視窗就捲
  const int VIS = 7;
  if (vidCursor_ < vidScroll_)           vidScroll_ = vidCursor_;
  if (vidCursor_ >= vidScroll_ + VIS)    vidScroll_ = vidCursor_ - VIS + 1;
  for (int r = 0; r < VIS && vidScroll_ + r < vidCount_; r++) {
    int i = vidScroll_ + r;
    bool sel = (i == vidCursor_);
    int y = 44 + r * 30;
    tft_.fillRect(0, y, 240, 30, sel ? TFT_DARKGREEN : TFT_BLACK);
    tft_.setTextColor(sel ? TFT_YELLOW : TFT_WHITE, sel ? TFT_DARKGREEN : TFT_BLACK);
    tft_.setCursor(10, y + 3);
    tft_.print(sel ? ">" : " ");
    tft_.print(vidIsVideo_[i] ? "[影]" : "[樂]");
    tft_.print(vidList_[i]);
  }
  // 超過一頁的提示
  if (vidCount_ > VIS) {
    tft_.setFont(&fonts::efontTW_14);
    tft_.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft_.setCursor(200, 260);
    tft_.printf("%d/%d", vidCursor_ + 1, vidCount_);
  }
  tft_.setFont(&fonts::efontTW_14);
  tft_.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft_.setCursor(5, 290);
  tft_.print("搖桿:選 OK:播 ↑↓音量 ←退出");
}

// 播放中的音量提示(畫在頂部,影片下一幀若蓋到會自己消失)
void MediaPlayer::drawVolOverlay() {
  char buf[20];
  snprintf(buf, sizeof(buf), " 音量 %d%% ", musicVolume_);
  tft_.setFont(&fonts::efontTW_16);
  tft_.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft_.setCursor(6, 4);
  tft_.print(buf);
}

// 音樂播放畫面
void MediaPlayer::drawMusicScreen(const String& name) {
  tft_.fillScreen(TFT_BLACK);
  tft_.fillRect(0, 0, 240, 32, TFT_NAVY);
  tft_.setFont(&fonts::efontTW_24);
  tft_.setTextColor(TFT_WHITE, TFT_NAVY);
  tft_.setCursor(30, 4);
  tft_.print("音樂播放中");
  tft_.setTextColor(TFT_CYAN, TFT_BLACK);
  tft_.setCursor(10, 110);
  tft_.println(name);
  tft_.setFont(&fonts::efontTW_14);
  tft_.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft_.setCursor(5, 290);
  tft_.print("↑↓音量 →暫停/繼續 ←退出");
}

// 暫停/繼續狀態顯示
void MediaPlayer::drawPauseState() {
  tft_.setFont(&fonts::efontTW_24);
  tft_.setTextColor(mediaPaused_ ? TFT_ORANGE : TFT_GREEN, TFT_BLACK);
  tft_.setCursor(90, 160);
  tft_.print(mediaPaused_ ? "[暫停]" : "[播放]");
}

void MediaPlayer::vidStop() {
  if (vidFile_) vidFile_.close();
  if (vidAud_)  vidAud_.close();
  vidPlaying_  = false;
  mediaPaused_ = false;
  digitalWrite(PIN_AMP_SD, LOW);
}

void MediaPlayer::vidStart(const String& name) {
  vidStop();
  String base = String("/video/") + name;
  vidFile_ = SD.open(base, FILE_READ);
  if (!vidFile_) return;
  String wav = base;                       // 同名 .wav 當聲音
  wav.replace(".mjp", ".wav");
  wav.replace(".MJP", ".WAV");
  vidAud_ = SD.open(wav, FILE_READ);
  vidHasAudio_ = (bool)vidAud_;
  if (vidHasAudio_) { vidAud_.seek(44); digitalWrite(PIN_AMP_SD, HIGH); }
  vidAudioFed_   = 0;
  vidFrameShown_ = 0;
  vidStartMs_    = millis();
  vidPlaying_    = true;
  tft_.fillScreen(TFT_BLACK);
}

void MediaPlayer::vidFeedAudio() {
  if (!vidHasAudio_) return;
  static uint8_t pcm[512];
  static int32_t txb[256];
  for (int round = 0; round < 16; round++) {
    int n = vidAud_.read(pcm, sizeof(pcm));
    if (n <= 0) {
      // 聲音先播完:把時鐘無縫切到 millis 繼續推影像
      vidHasAudio_ = false;
      vidStartMs_  = millis() - (unsigned long)((vidAudioFed_ / (float)(MUSIC_SAMPLE_RATE * 2)) * 1000);
      vidAudioFed_ = 0;
      return;
    }
    int samples = n / 2;
    int16_t* s = (int16_t*)pcm;
    for (int i = 0; i < samples; i++)
      txb[i] = ((int32_t)(s[i] * musicVolume_ / 100)) << 16;
    size_t written = 0;
    i2s_channel_write(musicTx_, txb, samples * 4, &written, 0);
    size_t used = written / 2;
    vidAudioFed_ += used;
    if ((int)used < n) { vidAud_.seek(vidAud_.position() - (n - used)); return; }
  }
}

size_t MediaPlayer::vidReadFrame() {
  static uint8_t chunk[1024];
  int len = 0, prev = -1;
  bool inFrame = false;
  while (true) {
    int n = vidFile_.read(chunk, sizeof(chunk));
    if (n <= 0) return 0;
    for (int i = 0; i < n; i++) {
      int c = chunk[i];
      if (!inFrame) {
        if (prev == 0xFF && c == 0xD8) { inFrame = true; vidFrameBuf_[0] = 0xFF; vidFrameBuf_[1] = 0xD8; len = 2; }
        prev = c;
        continue;
      }
      if (len < VID_FRAME_MAX) vidFrameBuf_[len++] = (uint8_t)c;
      if (prev == 0xFF && c == 0xD9) { vidFile_.seek(vidFile_.position() - (n - i - 1)); return len; }
      prev = c;
    }
  }
}

void MediaPlayer::vidUpdate() {
  vidFeedAudio();
  uint32_t target = vidHasAudio_
      ? (uint32_t)((vidAudioFed_ / (float)(MUSIC_SAMPLE_RATE * 2)) * VID_FPS)
      : (uint32_t)((millis() - vidStartMs_) / 1000.0f * VID_FPS);
  if (vidFrameShown_ >= target) return;
  size_t len = vidReadFrame();
  if (len == 0) { vidStop(); drawVideoMenu(); return; }   // 播完回選單
  vidFrameShown_++;
  if (target - vidFrameShown_ > 2) return;                 // 跳幀追時鐘
  if (vidJpeg_.openRAM(vidFrameBuf_, len, jpegDrawCb)) {
    int xoff = (240 - vidJpeg_.getWidth())  / 2;
    int yoff = (320 - vidJpeg_.getHeight()) / 2;
    if (xoff < 0) xoff = 0;
    if (yoff < 0) yoff = 0;
    vidJpeg_.decode(xoff, yoff, 0);
    vidJpeg_.close();
  }
}

void MediaPlayer::enterVideoMode() {
  musicStop();   // 影片聲音跟背景音樂共用 I²S,先停音樂
  if (!vidFrameBuf_)
    vidFrameBuf_ = (uint8_t*)heap_caps_malloc(VID_FRAME_MAX, MALLOC_CAP_SPIRAM);
  tft_.setSwapBytes(true);   // JPEGDEC 輸出 RGB565 little-endian,
                             // 沒這行畫面變紅綠雜訊(2026-06-12 實測)
  vidCursor_ = 0;
  vidScroll_ = 0;
  vidScanFiles();
  drawVideoMenu();
}

void MediaPlayer::exitVideoMode() {
  vidStop();
  tft_.setSwapBytes(false);  // 還原,避免影響其他畫面
  // 注意:音樂不停 — 音樂任意 mode 都能繼續放(原版行為)
}

// 音量調整(播放中 ↑↓ 鍵)
void MediaPlayer::mediaVolume(int delta) {
  int v = (int)musicVolume_ + delta;
  if (v < 0)   v = 0;
  if (v > 100) v = 100;
  musicVolume_ = (uint8_t)v;
  drawVolOverlay();
}

// 暫停/繼續(→ 鍵)
void MediaPlayer::mediaTogglePause(bool hasAudioNow) {
  mediaPaused_ = !mediaPaused_;
  if (mediaPaused_) {
    vidPauseStart_ = millis();
    digitalWrite(PIN_AMP_SD, LOW);             // 靜音
  } else {
    vidStartMs_ += millis() - vidPauseStart_;  // millis 時鐘補償暫停時間
    if (hasAudioNow) digitalWrite(PIN_AMP_SD, HIGH);
  }
}

void MediaPlayer::videoModeLoop(int btnEdge, Inputs& in) {
  // --- 影片播放中 ---
  if (vidPlaying_) {
    if (btnEdge == BTN_BACK)  { vidStop(); drawVideoMenu(); return; }     // ← 退出
    if (btnEdge == BTN_OK)    { mediaTogglePause(vidHasAudio_); drawPauseState(); }  // → 暫停/繼續
    if (btnEdge == BTN_PLUS)  mediaVolume(+10);                            // ↑ 音量
    if (btnEdge == BTN_MINUS) mediaVolume(-10);                            // ↓ 音量
    if (!mediaPaused_) vidUpdate();
    return;
  }
  // --- 音樂播放中(共用 musicPlay/musicUpdate)---
  if (musicPlaying_) {
    if (btnEdge == BTN_BACK)  { musicStop(); mediaPaused_ = false; drawVideoMenu(); return; }
    if (btnEdge == BTN_OK)    { mediaTogglePause(true); drawPauseState(); }
    if (btnEdge == BTN_PLUS)  mediaVolume(+10);
    if (btnEdge == BTN_MINUS) mediaVolume(-10);
    if (!mediaPaused_) {
      musicUpdate();
      if (!musicPlaying_) { mediaPaused_ = false; drawVideoMenu(); }  // 播完回選單
    }
    return;
  }
  // --- 選單瀏覽 ---
  static unsigned long lastMove = 0;
  if (vidCount_ > 0 && millis() - lastMove > 250) {
    int p = in.pitchPos();
    if (p > 240) { vidCursor_ = (vidCursor_ + 1) % vidCount_;             drawVideoMenu(); lastMove = millis(); }
    if (p < 15)  { vidCursor_ = (vidCursor_ - 1 + vidCount_) % vidCount_; drawVideoMenu(); lastMove = millis(); }
  }
  if (btnEdge == BTN_OK && vidCount_ > 0) {
    if (vidIsVideo_[vidCursor_]) {
      vidStart(vidList_[vidCursor_]);
    } else {
      String path = String("/music/") + vidList_[vidCursor_];
      musicPlay(path.c_str());
      if (musicPlaying_) drawMusicScreen(vidList_[vidCursor_]);
    }
  }
}
#endif  // ENABLE_VIDEO_MODE12
#endif  // ENABLE_MUSIC
