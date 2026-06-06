// ============================================================
// Voice_Test — 錄音 + 播放測試(I²S 麥克風 + 放大器 + SD + TFT)
//
// 一次驗證 V2-B 語音模組(INMP441 + MAX98357A + 喇叭)+ 手把現有
// SD + TFT 共 4 個元件能不能正常運作。
//
// === 操作 ===
//   肩鍵 L     → 開始錄音 / 停止錄音
//   + / −      → 在檔案清單上下移動游標
//   OK         → 播放選中的檔
//   返回       → 停止播放 / 退出 / 重新掃描 SD
//
// === 接線(同 [[handle-v2-plan]] V2-B 設計)===
//   INMP441:
//     VIN   → ESP32 3V3
//     GND   → GND(L/R 跟 GND 短接後同一條)
//     WS    → GPIO 12
//     SCK   → GPIO 11
//     SD    → GPIO 14
//   MAX98357A:
//     VIN   → 5V 軌
//     GND   → GND
//     LRC   → GPIO 12(跟 INMP441 共用)
//     BCLK  → GPIO 11(跟 INMP441 共用)
//     DIN   → GPIO 13
//     GAIN  → 浮空(預設 9dB)
//     SD    → 浮空(預設 mono)
//     OUT+/− → 喇叭 +/−
//
// === 錄音格式 ===
//   16 kHz / 單聲道 / 16-bit PCM WAV
//   檔名:/REC_001.WAV ~ /REC_999.WAV
//   單檔上限 30 秒
//
// 需要 Arduino-ESP32 core v3.0+(新 I2S API)
// ============================================================

#include <LovyanGFX.hpp>
#include <SD.h>
#include <SPI.h>
#include <driver/i2s_std.h>

// === LGFX 配置(同 Ground_TX_ESP32.ino 的設定)===
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
      cfg.pin_sclk    = 38;
      cfg.pin_mosi    = 39;
      cfg.pin_miso    = 40;   // 跟 SD 共用 SPI 時必須設,不然 SPI.begin 重新註冊 MISO 會失敗
      cfg.pin_dc      = 21;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs        = 48;
      cfg.pin_rst       = 16;
      cfg.pin_busy      = -1;
      cfg.memory_width  = 240;
      cfg.memory_height = 320;
      cfg.panel_width   = 240;
      cfg.panel_height  = 320;
      cfg.readable      = false;
      cfg.invert        = false;
      cfg.rgb_order     = false;
      cfg.dlen_16bit    = false;
      cfg.bus_shared    = true;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};
LGFX tft;

// === 腳位 ===
#define PIN_I2S_BCLK    11
#define PIN_I2S_WS      12
#define PIN_I2S_DOUT    13      // ESP32 → MAX98357A DIN
#define PIN_I2S_DIN     14      // INMP441 SD → ESP32
#define PIN_SD_CS       47   // 右側自由腳,不用上拉電阻,跟 38/39/40 SPI 軌靠近
#define PIN_SPI_SCK     38
#define PIN_SPI_MOSI    39
#define PIN_SPI_MISO    40
#define PIN_MENU_BTN     7
#define PIN_SHOULDER_L   8

// === 錄音參數 ===
// 16 kHz 會讓 BCLK = 1.024 MHz,低於 INMP441 規格最小 1.45 MHz 出怪雜訊
// 32 kHz → BCLK = 2.048 MHz,在 INMP441 規格內(1.45~3.27 MHz)
#define SAMPLE_RATE     32000
#define BUF_SAMPLES     512               // 每次 I²S 讀寫 sample 數(32 kHz 下約 16ms)
#define MAX_REC_SECS    30
#define MAX_FILES       20                // TFT 上最多顯示這麼多檔

// === Menu button enum(同 Ground_TX_ESP32)===
enum { BTN_NONE, BTN_PLUS, BTN_MINUS, BTN_OK, BTN_BACK };

// === 狀態機 ===
enum AppState { ST_MENU, ST_REC, ST_PLAY };
AppState state     = ST_MENU;
AppState lastState = (AppState)-1;

// === I²S handles ===
i2s_chan_handle_t i2s_tx = NULL;
i2s_chan_handle_t i2s_rx = NULL;

// === SD 狀態 ===
bool   sdOK = false;
String fileList[MAX_FILES];
int    fileCount = 0;
int    cursor    = 0;
int    viewTop   = 0;

// === 錄音狀態 ===
File          recFile;
uint32_t      recBytes   = 0;
unsigned long recStartMs = 0;

// === 播放狀態 ===
File          playFile;
uint32_t      playBytes  = 0;
uint32_t      playTotal  = 0;

// ============================================================
// 按鈕讀取(電阻階梯,同 Ground_TX_ESP32 的門檻)
int readMenuBtn() {
  int v = analogRead(PIN_MENU_BTN);
  if (v > 3250) return BTN_PLUS;
  if (v > 2415) return BTN_MINUS;
  if (v > 1665) return BTN_OK;
  if (v >  640) return BTN_BACK;
  return BTN_NONE;
}

// ============================================================
// WAV 檔頭(44 bytes,寫到 file 開頭)
void writeWavHeader(File &f, uint32_t dataLen) {
  uint16_t numChannels   = 1;
  uint16_t bitsPerSample = 16;
  uint32_t sampleRate    = SAMPLE_RATE;
  uint32_t byteRate      = sampleRate * numChannels * bitsPerSample / 8;
  uint16_t blockAlign    = numChannels * bitsPerSample / 8;
  uint32_t totalLen      = dataLen + 36;
  uint32_t fmtSize       = 16;
  uint16_t fmtPCM        = 1;

  f.seek(0);
  f.write((const uint8_t*)"RIFF", 4);
  f.write((const uint8_t*)&totalLen, 4);
  f.write((const uint8_t*)"WAVE", 4);
  f.write((const uint8_t*)"fmt ", 4);
  f.write((const uint8_t*)&fmtSize, 4);
  f.write((const uint8_t*)&fmtPCM, 2);
  f.write((const uint8_t*)&numChannels, 2);
  f.write((const uint8_t*)&sampleRate, 4);
  f.write((const uint8_t*)&byteRate, 4);
  f.write((const uint8_t*)&blockAlign, 2);
  f.write((const uint8_t*)&bitsPerSample, 2);
  f.write((const uint8_t*)"data", 4);
  f.write((const uint8_t*)&dataLen, 4);
}

// ============================================================
// I²S 初始化(full-duplex,共用 BCLK/WS,RX/TX 不同 slot_mask)
void initI2S() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, &i2s_tx, &i2s_rx);

  // 共用 GPIO 配置
  i2s_std_gpio_config_t gpio_cfg = {};
  gpio_cfg.mclk = I2S_GPIO_UNUSED;
  gpio_cfg.bclk = (gpio_num_t)PIN_I2S_BCLK;
  gpio_cfg.ws   = (gpio_num_t)PIN_I2S_WS;
  gpio_cfg.dout = (gpio_num_t)PIN_I2S_DOUT;
  gpio_cfg.din  = (gpio_num_t)PIN_I2S_DIN;
  gpio_cfg.invert_flags.mclk_inv = false;
  gpio_cfg.invert_flags.bclk_inv = false;
  gpio_cfg.invert_flags.ws_inv   = false;

  // RX(INMP441):32-bit slot,只讀左聲道(L/R 接 GND)
  i2s_std_config_t rx_cfg = {};
  rx_cfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  rx_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
  rx_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
  rx_cfg.gpio_cfg = gpio_cfg;
  i2s_channel_init_std_mode(i2s_rx, &rx_cfg);

  // TX(MAX98357A):32-bit slot,送 L+R(模組 SD 浮空 = mono mix)
  i2s_std_config_t tx_cfg = {};
  tx_cfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  tx_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
  tx_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
  tx_cfg.gpio_cfg = gpio_cfg;
  i2s_channel_init_std_mode(i2s_tx, &tx_cfg);

  i2s_channel_enable(i2s_rx);
  i2s_channel_enable(i2s_tx);
}

// ============================================================
// 掃描 SD 上所有 REC_*.WAV
void scanFiles() {
  fileCount = 0;
  if (!sdOK) return;

  File root = SD.open("/");
  if (!root) return;

  while (fileCount < MAX_FILES) {
    File f = root.openNextFile();
    if (!f) break;
    if (!f.isDirectory()) {
      String name = f.name();
      if (name.startsWith("/")) name = name.substring(1);
      if (name.startsWith("REC_") && name.endsWith(".WAV")) {
        fileList[fileCount++] = name;
      }
    }
    f.close();
  }
  root.close();

  // 簡單排序(字串遞增)
  for (int i = 0; i < fileCount - 1; i++) {
    for (int j = 0; j < fileCount - 1 - i; j++) {
      if (fileList[j] > fileList[j + 1]) {
        String t = fileList[j];
        fileList[j] = fileList[j + 1];
        fileList[j + 1] = t;
      }
    }
  }
}

// 找下一個可用檔名 /REC_NNN.WAV
String nextRecFilename() {
  for (int n = 1; n < 1000; n++) {
    char buf[16];
    snprintf(buf, sizeof(buf), "/REC_%03d.WAV", n);
    if (!SD.exists(buf)) return String(buf);
  }
  return "";
}

// ============================================================
// === 主選單畫面 ===
void drawMenuStatic() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0,   0, 240, 32, TFT_NAVY);
  tft.fillRect(0, 280, 240, 40, TFT_DARKGREY);

  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.setCursor(30, 4);
  tft.print("錄音播放測試");

  tft.setFont(&fonts::efontTW_14);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setCursor(5, 285);
  tft.print("肩鍵L:錄音  +/-:選  OK:放");
  tft.setCursor(5, 302);
  tft.print("返回:重新掃描");
}

void drawMenuDynamic() {
  tft.fillRect(0, 32, 240, 248, TFT_BLACK);
  tft.setFont(&fonts::efontTW_14);

  if (fileCount == 0) {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(10, 100);
    tft.print("(SD 卡上沒有錄音)");
    tft.setCursor(10, 130);
    tft.print("按肩鍵 L 開始");
    tft.setCursor(10, 150);
    tft.print("錄第一段");
    return;
  }

  const int rows = 10;
  if (cursor < viewTop)               viewTop = cursor;
  if (cursor >= viewTop + rows)       viewTop = cursor - rows + 1;
  if (viewTop > fileCount - rows)     viewTop = max(0, fileCount - rows);
  if (viewTop < 0)                    viewTop = 0;

  for (int i = 0; i < rows && (viewTop + i) < fileCount; i++) {
    int idx     = viewTop + i;
    int y       = 38 + i * 24;
    bool sel    = (idx == cursor);
    uint16_t bg = sel ? TFT_DARKGREEN : TFT_BLACK;
    uint16_t fg = sel ? TFT_YELLOW    : TFT_WHITE;
    tft.fillRect(0, y, 240, 24, bg);
    tft.setTextColor(fg, bg);
    tft.setCursor(10, y + 5);
    tft.print(sel ? "> " : "  ");
    tft.print(fileList[idx]);
  }
}

// === 錄音畫面 ===
void drawRecStatic() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0,   0, 240, 32, TFT_RED);
  tft.fillRect(0, 280, 240, 40, TFT_DARKGREY);

  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(TFT_WHITE, TFT_RED);
  tft.setCursor(60, 4);
  tft.print("● 錄音中");

  tft.setFont(&fonts::efontTW_14);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setCursor(10, 285);
  tft.print("再按肩鍵 L 停止");
  tft.setCursor(10, 302);
  tft.print("(自動 30 秒上限)");
}

void drawRecDynamic() {
  uint32_t s = (millis() - recStartMs) / 1000;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu : %02lu", s / 60, s % 60);

  tft.fillRect(0, 100, 240, 70, TFT_BLACK);
  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(60, 130);
  tft.print(buf);
}

// === 播放畫面 ===
void drawPlayStatic() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0,   0, 240, 32, TFT_DARKGREEN);
  tft.fillRect(0, 280, 240, 40, TFT_DARKGREY);

  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREEN);
  tft.setCursor(75, 4);
  tft.print("▶ 播放中");

  tft.setFont(&fonts::efontTW_14);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 60);
  tft.print("檔名:");
  tft.setCursor(10, 85);
  tft.print(fileList[cursor]);

  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setCursor(10, 285);
  tft.print("按返回鈕停止");
}

void drawPlayDynamic() {
  uint32_t playSecs  = playBytes / (SAMPLE_RATE * 2);
  uint32_t totalSecs = playTotal / (SAMPLE_RATE * 2);
  char buf[24];
  snprintf(buf, sizeof(buf), "%02lu:%02lu / %02lu:%02lu",
           playSecs / 60, playSecs % 60,
           totalSecs / 60, totalSecs % 60);

  tft.fillRect(0, 130, 240, 40, TFT_BLACK);
  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(30, 140);
  tft.print(buf);
}

// ============================================================
// 開始錄音
void startRec() {
  String name = nextRecFilename();
  if (name.length() == 0) {
    Serial.println("[!] 沒有可用檔名(REC_999 已用完)");
    return;
  }
  recFile = SD.open(name.c_str(), FILE_WRITE);
  if (!recFile) {
    Serial.println("[!] 無法開啟錄音檔");
    return;
  }
  uint8_t zeros[44] = {0};
  recFile.write(zeros, 44);
  recBytes   = 0;
  recStartMs = millis();
  state      = ST_REC;
  Serial.printf("[+] 開始錄音:%s\n", name.c_str());
}

// 錄音中:讀 I²S → 轉 16-bit → 寫 SD
void doRecChunk() {
  int32_t  raw[BUF_SAMPLES];
  size_t   bytesRead = 0;
  esp_err_t e = i2s_channel_read(i2s_rx, raw, sizeof(raw), &bytesRead,
                                  100 / portTICK_PERIOD_MS);
  if (e != ESP_OK || bytesRead == 0) return;

  int samples = bytesRead / 4;
  int16_t pcm[BUF_SAMPLES];
  for (int i = 0; i < samples; i++) {
    // INMP441 24-bit MSB-aligned 在 32-bit slot,右移 14 提升輕聲音量
    int32_t v = raw[i] >> 14;
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    pcm[i] = (int16_t)v;
  }
  recFile.write((uint8_t*)pcm, samples * 2);
  recBytes += samples * 2;

  if (millis() - recStartMs > MAX_REC_SECS * 1000UL) stopRec();
}

void stopRec() {
  writeWavHeader(recFile, recBytes);
  recFile.close();
  Serial.printf("[+] 錄音結束,寫入 %lu bytes\n", recBytes);
  scanFiles();
  cursor = fileCount - 1;
  state  = ST_MENU;
}

// ============================================================
// 開始播放
void startPlay() {
  if (cursor < 0 || cursor >= fileCount) return;
  String path = "/" + fileList[cursor];
  playFile = SD.open(path.c_str(), FILE_READ);
  if (!playFile) {
    Serial.printf("[!] 開檔失敗:%s\n", path.c_str());
    return;
  }
  uint32_t fsize = playFile.size();
  if (fsize <= 44) {
    Serial.println("[!] 檔案太小");
    playFile.close();
    return;
  }
  playFile.seek(44);
  playBytes = 0;
  playTotal = fsize - 44;
  state     = ST_PLAY;
  Serial.printf("[+] 播放:%s (%lu bytes 音訊)\n", path.c_str(), playTotal);
}

// 播放中:讀 SD 16-bit → 擴展 32-bit → 寫 I²S
void doPlayChunk() {
  uint8_t  pcmBuf[BUF_SAMPLES * 2];
  size_t   toRead = sizeof(pcmBuf);
  if (playBytes + toRead > playTotal) toRead = playTotal - playBytes;
  if (toRead == 0) { stopPlay(); return; }

  int n = playFile.read(pcmBuf, toRead);
  if (n <= 0) { stopPlay(); return; }

  int samples = n / 2;
  int32_t tx[BUF_SAMPLES];
  int16_t *pcm = (int16_t*)pcmBuf;
  for (int i = 0; i < samples; i++) {
    tx[i] = (int32_t)pcm[i] << 16;        // 16-bit → 32-bit MSB-aligned
  }
  size_t bytesWritten = 0;
  i2s_channel_write(i2s_tx, tx, samples * 4, &bytesWritten,
                    100 / portTICK_PERIOD_MS);
  playBytes += n;
}

void stopPlay() {
  playFile.close();
  Serial.println("[+] 播放結束");
  state = ST_MENU;
}

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== 錄音播放測試 ===");

  // 關 ESP32-S3 RGB LED
  neopixelWrite(48, 0, 0, 0);

  pinMode(PIN_SHOULDER_L, INPUT_PULLUP);
  analogReadResolution(12);

  // === 重要:SPI + SD 先 init,再讓 LovyanGFX 加入共用 ===
  // 順序顛倒(TFT 先)會讓 SD 拿不到 MISO,掛載失敗。
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  if (SD.begin(PIN_SD_CS, SPI)) {
    sdOK = true;
    Serial.println("[+] SD 掛載成功");
  } else {
    Serial.println("[!] SD 掛載失敗");
    // 注意:TFT 還沒 init,先用 Serial 印,後面 tft.init() 後再上紅字
  }

  // TFT(SD 之後才 init,讓 SD 先佔到正確的 SPI 設定)
  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::efontTW_16);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(5, 5);
  tft.print("錄音播放測試");
  tft.setCursor(5, 28);
  tft.print("初始化中...");

  // 如果 SD 失敗,TFT 上補紅字然後停在這
  if (!sdOK) {
    tft.setCursor(5, 55);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("SD 掛載失敗!");
    tft.setCursor(5, 80);
    tft.print("檢查 SD 模組接線");
    while (1) delay(1000);
  }

  // I²S
  initI2S();
  Serial.println("[+] I²S 初始化完成");

  scanFiles();
  Serial.printf("[+] 找到 %d 個錄音\n", fileCount);

  drawMenuStatic();
  drawMenuDynamic();
  Serial.println("[+] 就緒,按肩鍵 L 開始錄音");
}

// ============================================================
void loop() {
  // 狀態切換 → 重畫整個畫面
  if (state != lastState) {
    if (state == ST_MENU) { drawMenuStatic(); drawMenuDynamic(); }
    else if (state == ST_REC)  drawRecStatic();
    else if (state == ST_PLAY) drawPlayStatic();
    lastState = state;
  }

  // 按鈕 edge 偵測
  int btn = readMenuBtn();
  static int  lastBtn = BTN_NONE;
  bool btnEdge = (btn != lastBtn && btn != BTN_NONE);
  lastBtn = btn;

  bool shldL = (digitalRead(PIN_SHOULDER_L) == LOW);
  static bool lastShldL = false;
  bool shldLEdge = shldL && !lastShldL;
  lastShldL = shldL;

  // === 主選單 ===
  if (state == ST_MENU) {
    static int  lastCursor = -1;
    if (cursor != lastCursor) {
      drawMenuDynamic();
      lastCursor = cursor;
    }
    if (shldLEdge) { startRec(); return; }
    if (btnEdge) {
      if (btn == BTN_PLUS  && cursor > 0)               cursor--;
      if (btn == BTN_MINUS && cursor < fileCount - 1)   cursor++;
      if (btn == BTN_OK    && fileCount > 0)            startPlay();
      if (btn == BTN_BACK) {
        Serial.println("[+] 重新掃描 SD");
        scanFiles();
        drawMenuDynamic();
      }
    }
  }
  // === 錄音中 ===
  else if (state == ST_REC) {
    static unsigned long lastDraw = 0;
    if (millis() - lastDraw > 250) {
      drawRecDynamic();
      lastDraw = millis();
    }
    doRecChunk();
    if (shldLEdge) stopRec();
  }
  // === 播放中 ===
  else if (state == ST_PLAY) {
    static unsigned long lastDraw = 0;
    if (millis() - lastDraw > 250) {
      drawPlayDynamic();
      lastDraw = millis();
    }
    doPlayChunk();
    if (btnEdge && btn == BTN_BACK) stopPlay();
  }
}
