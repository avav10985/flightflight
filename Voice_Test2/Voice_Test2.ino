// ============================================================
// Voice_Test2 — 錄音 + 播放測試(用 ESP_I2S.h 新 C++ 包裝)
//
// 跟 Voice_Test 同硬體,改用 Arduino-ESP32 3.x 官方的 I2SClass。
// 雙工切換策略:模式切換時用 i2s.end() + i2s.begin() 重新 init,
// 切到「只 TX」或「只 RX」對應的 pin 配置(另一邊腳設 -1),
// 確保播放時 INMP441 沒接到 I²S DMA,錄音時 MAX 沒接到。
//
// 操作跟 Voice_Test 一樣:肩鍵L 錄音、+/- 選、OK 放、返回 重掃。
//
// 接線:
//   INMP441 VIN=3V3、GND=GND、SCK=11、WS=12、SD=14
//   MAX98357A VIN=5V、GND=GND、BCLK=11、LRC=12、DIN=13、SD=18(休眠控制)
//   SD 卡:CS=47、SCK=38、MOSI=39、MISO=40
//
// 需要 Arduino-ESP32 core v3.0+,內建 ESP_I2S.h。
// ============================================================

#include <LovyanGFX.hpp>
#include <SD.h>
#include <SPI.h>
#include <ESP_I2S.h>

// === LGFX 配置 ===
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
      cfg.pin_miso    = 40;
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

// === 引腳定義(共用時脈 + 分開資料線)===
const int PIN_I2S_BCLK   = 11;   // INMP441 SCK / MAX98357A BCLK 共用
const int PIN_I2S_WS     = 12;   // INMP441 WS  / MAX98357A LRC  共用
const int PIN_I2S_DOUT   = 13;   // ESP32 → MAX98357A DIN(獨立輸出)
const int PIN_I2S_DIN    = 14;   // INMP441 SD → ESP32(獨立輸入)
const int PIN_AMP_SD     = 18;   // MAX98357A shutdown 控制:LOW=休眠、HIGH=放音
const int PIN_SD_CS      = 47;
const int PIN_SPI_SCK    = 38;
const int PIN_SPI_MOSI   = 39;
const int PIN_SPI_MISO   = 40;
const int PIN_MENU_BTN   = 7;
const int PIN_SHOULDER_L = 8;
const int PIN_JOY_PITCH  = 4;    // 右搖桿 Y(選單上下用)

const uint32_t SAMPLE_RATE = 32000;
const int      BUF_SAMPLES = 512;
const int      MAX_REC_SECS = 30;
const int      MAX_FILES   = 20;

enum { BTN_NONE, BTN_PLUS, BTN_MINUS, BTN_OK, BTN_BACK };
enum AppState { ST_MENU, ST_REC, ST_PLAY };

// === 全域狀態 ===
I2SClass  i2s;
AppState  state     = ST_MENU;
AppState  lastState = (AppState)-1;
bool      sdOK = false;
String    fileList[MAX_FILES];
int       fileCount = 0;
int       cursor    = 0;
int       viewTop   = 0;

File          recFile;
uint32_t      recBytes   = 0;
unsigned long recStartMs = 0;

File          playFile;
uint32_t      playBytes  = 0;
uint32_t      playTotal  = 0;

// I²S 一次性 init(duplex 模式,開機 init 一次永遠不關)
// 防雜訊改成 由 MAX98357A SD 腳控制(GPIO 18 拉低 = MAX 休眠)
bool i2sReady = false;
void i2sInitOnce() {
  if (i2sReady) return;
  // BCLK + WS + DOUT + DIN 都接,duplex 一次 init
  i2s.setPins(PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT, -1, PIN_I2S_DIN);
  bool ok = i2s.begin(I2S_MODE_STD, SAMPLE_RATE,
                      I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
  if (!ok) {
    Serial.println("[!] i2s.begin (duplex) 失敗");
    return;
  }
  i2sReady = true;
  Serial.printf("[+] I²S duplex 啟動 32-bit MONO @%dHz\n", SAMPLE_RATE);
}

// ============================================================
// 階梯按鈕:4 樣本平均 + 50ms 時間穩定去抖
// 之前 3-vote 還是會在「按 + 過程中 ADC 短暫停在 OK 範圍 3 次」誤觸發 OK。
// 改成「同一個值要連續穩定 50ms 才當真」,徹底擋掉 transient 誤觸發。
int readMenuBtn() {
  static int  candidate     = BTN_NONE;
  static unsigned long candidateStart = 0;
  static int  stableReading = BTN_NONE;

  // 4 樣本平均消除 ADC 雜訊
  int v = 0;
  for (int i = 0; i < 4; i++) v += analogRead(PIN_MENU_BTN);
  v /= 4;

  int raw;
  // 2026-06-09 OK 跟 BACK threshold 互換時的中間值
  // 原本 OK > 1550 太近,OK 按鈕 ADC 飄一點(1400 左右)就會被當成 BACK
  // → 觸發「重新掃描 SD」+ cursor=0 → 自動跳回 REC_001
  // 把 OK 邊界拉低到 1200,BACK 拉低到 400,給 OK 更多 margin
  if      (v > 3000) raw = BTN_PLUS;
  else if (v > 2100) raw = BTN_MINUS;
  else if (v > 1200) raw = BTN_OK;
  else if (v >  400) raw = BTN_BACK;
  else               raw = BTN_NONE;

  // 候選變了 → 重置計時;沒變 → 累計穩定時間
  if (raw != candidate) {
    candidate      = raw;
    candidateStart = millis();
  }
  // 候選穩定 50ms 後才成為正式回報值
  if (millis() - candidateStart >= 50) {
    stableReading = candidate;
  }
  return stableReading;
}

// ============================================================
// WAV 檔頭(44 bytes,16kHz/Mono/16-bit)
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
// SD 掃描 + 檔名產生
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
  // 字串遞增排序
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

String nextRecFilename() {
  for (int n = 1; n <= 999; n++) {
    char buf[16];
    snprintf(buf, sizeof(buf), "/REC_%03d.WAV", n);
    if (!SD.exists(buf)) return String(buf);
  }
  return "";
}

// ============================================================
// TFT 畫面
void drawMenuStatic() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0,   0, 240, 32, TFT_NAVY);
  tft.fillRect(0, 280, 240, 40, TFT_DARKGREY);
  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.setCursor(30, 4);
  tft.print("錄音播放測試2");
  tft.setFont(&fonts::efontTW_14);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setCursor(5, 285);
  tft.print("肩L:錄  右搖桿:選  OK:放");
  tft.setCursor(5, 302);
  tft.print("返回:重新掃描");
}

// 上次畫到 TFT 的狀態(partial redraw 用)
int g_drawnCursor    = -1;
int g_drawnViewTop   = -1;
int g_drawnFileCount = -1;

// 畫單一列(row_idx = 0~9,file_idx = 該列對應的檔案編號)
void drawMenuRow(int row_idx, int file_idx) {
  int y       = 38 + row_idx * 24;
  bool sel    = (file_idx == cursor);
  uint16_t bg = sel ? TFT_DARKGREEN : TFT_BLACK;
  uint16_t fg = sel ? TFT_YELLOW    : TFT_WHITE;
  tft.fillRect(0, y, 240, 24, bg);
  tft.setTextColor(fg, bg);
  tft.setCursor(10, y + 5);
  tft.print(sel ? "> " : "  ");
  tft.print(fileList[file_idx]);
}

void drawMenuDynamic() {
  // 沒檔案的空狀態(只畫一次)
  if (fileCount == 0) {
    if (g_drawnFileCount == 0) return;   // 已經畫過,跳過
    tft.fillRect(0, 32, 240, 248, TFT_BLACK);
    tft.setFont(&fonts::efontTW_14);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(10, 100);
    tft.print("(SD 沒錄音檔)");
    tft.setCursor(10, 130);
    tft.print("按肩鍵 L 開始錄第一段");
    g_drawnFileCount = 0;
    g_drawnCursor    = -1;
    g_drawnViewTop   = -1;
    return;
  }

  const int rows = 10;
  if (cursor < viewTop)            viewTop = cursor;
  if (cursor >= viewTop + rows)    viewTop = cursor - rows + 1;
  if (viewTop > fileCount - rows)  viewTop = max(0, fileCount - rows);
  if (viewTop < 0)                 viewTop = 0;

  tft.setFont(&fonts::efontTW_14);

  // 視窗位置變了 / 檔案數變了 → 全部重畫
  if (viewTop != g_drawnViewTop || fileCount != g_drawnFileCount) {
    tft.fillRect(0, 32, 240, 248, TFT_BLACK);
    for (int i = 0; i < rows && (viewTop + i) < fileCount; i++) {
      drawMenuRow(i, viewTop + i);
    }
  }
  // 只游標移動 → 只重畫舊位置 + 新位置 2 列
  else if (cursor != g_drawnCursor) {
    if (g_drawnCursor >= viewTop && g_drawnCursor < viewTop + rows) {
      drawMenuRow(g_drawnCursor - viewTop, g_drawnCursor);
    }
    if (cursor >= viewTop && cursor < viewTop + rows) {
      drawMenuRow(cursor - viewTop, cursor);
    }
  }

  g_drawnCursor    = cursor;
  g_drawnViewTop   = viewTop;
  g_drawnFileCount = fileCount;
}

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
  tft.print("(30 秒上限)");
}

void drawRecDynamic() {
  uint32_t s = (millis() - recStartMs) / 1000;
  static uint32_t lastRecSec = 99999;
  if (s == lastRecSec) return;
  lastRecSec = s;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu : %02lu", s / 60, s % 60);
  tft.fillRect(0, 100, 240, 70, TFT_BLACK);
  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(60, 130);
  tft.print(buf);
}

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
  static uint32_t lastPlaySec = 99999;
  if (playSecs == lastPlaySec) return;
  lastPlaySec = playSecs;
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
// 錄音流程
void startRec() {
  if (!sdOK) {
    Serial.println("[!] SD 沒接,不能錄音");
    tft.fillRect(0, 280, 240, 40, TFT_RED);
    tft.setFont(&fonts::efontTW_16);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setCursor(10, 290);
    tft.print("先插 SD 卡");
    delay(1500);
    drawMenuStatic();
    drawMenuDynamic();
    return;
  }
  String name = nextRecFilename();
  if (name.length() == 0) {
    Serial.println("[!] REC_999 已用完");
    return;
  }
  recFile = SD.open(name.c_str(), FILE_WRITE);
  if (!recFile) {
    Serial.println("[!] 無法開啟錄音檔");
    return;
  }
  digitalWrite(PIN_AMP_SD, LOW);     // MAX 休眠(不影響錄音)
  uint8_t zeros[44] = {0};
  recFile.write(zeros, 44);
  recBytes   = 0;
  recStartMs = millis();
  state      = ST_REC;
  Serial.printf("[+] 開始錄音:%s\n", name.c_str());
}

void doRecChunk() {
  int32_t raw[BUF_SAMPLES];
  // 用 i2s.read 而不是 readBytes:read 是 ESP_I2S.h 自己的方法,
  // 直接從 DMA buffer 拿,readBytes 是 Stream 介面會 block 1秒等資料
  size_t bytesRead = i2s.read((uint8_t*)raw, sizeof(raw));
  if (bytesRead == 0) return;
  int samples = bytesRead / 4;
  int16_t pcm[BUF_SAMPLES];
  for (int i = 0; i < samples; i++) {
    int32_t v = raw[i] >> 16;
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
// 播放流程
void startPlay() {
  if (!sdOK) {
    Serial.println("[!] SD 沒接,不能播放");
    tft.fillRect(0, 280, 240, 40, TFT_RED);
    tft.setFont(&fonts::efontTW_16);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setCursor(10, 290);
    tft.print("先插 SD 卡");
    delay(1500);
    drawMenuStatic();
    drawMenuDynamic();
    return;
  }
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
  digitalWrite(PIN_AMP_SD, HIGH);     // 解除 MAX 休眠
  Serial.printf("[+] 播放:%s (%lu bytes),AMP_SD=HIGH\n", path.c_str(), playTotal);
}

void doPlayChunk() {
  uint8_t  pcmBuf[BUF_SAMPLES * 2];
  size_t   toRead = sizeof(pcmBuf);
  if (playBytes + toRead > playTotal) toRead = playTotal - playBytes;
  if (toRead == 0) { stopPlay(); return; }
  int n = playFile.read(pcmBuf, toRead);
  if (n <= 0) { stopPlay(); return; }
  // 16-bit PCM → 32-bit MSB-aligned(<< 16)寫進 I²S
  int samples = n / 2;
  int32_t tx[BUF_SAMPLES];
  int16_t *pcm = (int16_t*)pcmBuf;
  for (int i = 0; i < samples; i++) {
    tx[i] = (int32_t)pcm[i] << 16;
  }
  size_t written = i2s.write((uint8_t*)tx, samples * 4);
  playBytes += n;
  // 第一次寫入時 debug 確認有寫進去
  static bool firstWrite = true;
  if (firstWrite) {
    Serial.printf("[i2s] 第一次 write: 給 %d bytes,實際寫入 %u bytes\n",
                  samples * 4, (unsigned)written);
    firstWrite = false;
  }
}

void stopPlay() {
  playFile.close();
  digitalWrite(PIN_AMP_SD, LOW);      // MAX 休眠
  Serial.println("[+] 播放結束");
  state = ST_MENU;
}

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Voice_Test2(ESP_I2S.h C++ 包裝版)===");

  neopixelWrite(48, 0, 0, 0);  // 關 RGB LED

  pinMode(PIN_SHOULDER_L, INPUT_PULLUP);

  // MAX98357A SD 預設拉低 → 休眠靜音
  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, LOW);

  analogReadResolution(12);

  // SPI + SD(先 init SD,再讓 LovyanGFX 加入共用)
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  if (SD.begin(PIN_SD_CS, SPI)) {
    sdOK = true;
    Serial.println("[+] SD 掛載成功");
  } else {
    Serial.println("[!] SD 掛載失敗(可繼續用,只是不能錄/放)");
  }

  // TFT
  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::efontTW_16);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(5, 5);
  tft.print("錄音播放測試2");
  tft.setCursor(5, 28);
  tft.print("初始化中...");

  if (!sdOK) {
    tft.setCursor(5, 55);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("SD 未接(可繼續用)");
    tft.setCursor(5, 80);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("錄/放時要插 SD");
    delay(1500);
  }

  // I²S duplex 一次性啟動,以後不切換(避免 end/begin 切模式造成讀寫失敗)
  i2sInitOnce();

  scanFiles();
  Serial.printf("[+] 找到 %d 個錄音\n", fileCount);

  drawMenuStatic();
  drawMenuDynamic();
  Serial.println("[+] 就緒,按肩鍵 L 開始");
}

// ============================================================
void loop() {
  if (state != lastState) {
    if      (state == ST_MENU) {
      // 重置 partial-redraw 狀態,進選單時整個重畫
      g_drawnCursor = -1; g_drawnViewTop = -1; g_drawnFileCount = -1;
      drawMenuStatic();
      drawMenuDynamic();
    }
    else if (state == ST_REC)    drawRecStatic();
    else if (state == ST_PLAY)   drawPlayStatic();
    lastState = state;
  }

  int btn = readMenuBtn();
  static int  lastBtn = BTN_NONE;
  // 只在「鬆開 → 按下」瞬間 fire,鄰近 button 互跳不會誤觸發
  bool btnEdge = (lastBtn == BTN_NONE && btn != BTN_NONE);
  lastBtn = btn;
  // Debug:btnEdge 觸發時印一行,看實際是哪個 button + ADC 多少
  if (btnEdge) {
    Serial.printf("[btn] edge btn=%d (1=+ 2=- 3=OK 4=back) v=%d\n",
                  btn, analogRead(PIN_MENU_BTN));
  }

  bool shldL = (digitalRead(PIN_SHOULDER_L) == LOW);
  static bool lastShldL = false;
  bool shldLEdge = shldL && !lastShldL;
  lastShldL = shldL;

  if (state == ST_MENU) {
    drawMenuDynamic();   // 內部會自動判斷要不要 redraw(partial / full / 跳過)
    if (shldLEdge) { startRec(); return; }

    // 右搖桿 Y 軸控制游標(連續推會加速捲動)
    // 推上(ADC > 3000)→ 游標上;推下(ADC < 1000)→ 游標下
    static unsigned long lastJoyMove = 0;
    static unsigned long joyHoldSince = 0;
    int joyY = analogRead(PIN_JOY_PITCH);
    bool joyUp   = (joyY > 3000);
    bool joyDown = (joyY < 1000);
    if (!joyUp && !joyDown) {
      joyHoldSince = 0;   // 回中位重置加速計時
    } else {
      if (joyHoldSince == 0) joyHoldSince = millis();
      // 按住越久越快:前 500ms 一次,500ms~1.5s 250ms 一次,1.5s 後 100ms 一次
      unsigned long held = millis() - joyHoldSince;
      unsigned long rate = (held > 1500) ? 80 : (held > 500) ? 200 : 350;
      if (millis() - lastJoyMove >= rate) {
        if (joyUp   && cursor > 0)               cursor--;
        if (joyDown && cursor < fileCount - 1)   cursor++;
        lastJoyMove = millis();
      }
    }

    // 階梯按鈕:邊緣觸發 + 對 +/- 加自動連發(按住捲動長 list)
    static unsigned long btnHoldStart   = 0;
    static unsigned long lastBtnRepeat  = 0;
    bool fireNav = false;
    if (btnEdge && (btn == BTN_PLUS || btn == BTN_MINUS)) {
      fireNav = true;
      btnHoldStart = millis();
    } else if (btn == BTN_NONE) {
      btnHoldStart = 0;
    } else if ((btn == BTN_PLUS || btn == BTN_MINUS) && btnHoldStart > 0) {
      // 按住超過 500ms 後,每 200ms 自動連發一次
      if (millis() - btnHoldStart > 500 && millis() - lastBtnRepeat > 200) {
        fireNav = true;
        lastBtnRepeat = millis();
      }
    }
    if (fireNav) {
      if (btn == BTN_PLUS  && cursor > 0)             cursor--;
      if (btn == BTN_MINUS && cursor < fileCount - 1) cursor++;
    }
    if (btnEdge) {
      if (btn == BTN_OK   && fileCount > 0) startPlay();
      if (btn == BTN_BACK) {
        Serial.println("[+] 重新掃描 SD");
        scanFiles();
        cursor = 0; viewTop = 0;
        drawMenuDynamic();
      }
    }
  }
  else if (state == ST_REC) {
    drawRecDynamic();
    doRecChunk();
    if (shldLEdge) stopRec();
  }
  else if (state == ST_PLAY) {
    drawPlayDynamic();
    doPlayChunk();
    if (btnEdge && btn == BTN_BACK) stopPlay();
  }
}
