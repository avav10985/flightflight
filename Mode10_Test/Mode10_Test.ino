// ============================================================
// Mode10_Test — Phase 2:PTT 錄音 → WiFi → Groq Whisper STT
//
// 流程:按住肩鈕錄音 → 放開 → WiFi 上傳到 Groq Whisper → Serial 印辨識文字
//
// === 操作 ===
//   左肩鈕(GPIO 8)按住 → 開始錄音
//   放開              → 停錄 + 自動上傳辨識
//
// === 需要 ===
//   - PSRAM=opi 開啟(handle V2-A 板已開)
//   - secrets.h(從 secrets.h.example 複製改名,填 WiFi + GROQ_API_KEY)
//   - WiFi 熱點(2.4GHz,ESP32-S3 不支援 5GHz)
//
// === 注意 ===
//   - WiFi 跟 NRF24 都在 2.4GHz,這隻 sketch 沒開 NRF24,沒衝突
//   - 之後合進主程式要做 WiFi on/off 切換
// ============================================================

#include <driver/i2s_std.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "secrets.h"

#define PIN_I2S_BCLK     11
#define PIN_I2S_WS       12
#define PIN_I2S_DIN      14
#define PIN_SHOULDER_L    8

#define SAMPLE_RATE     16000
#define MAX_REC_SECS       10
#define MAX_SAMPLES     (SAMPLE_RATE * MAX_REC_SECS)
#define I2S_CHUNK         512

// Groq API endpoint
#define GROQ_HOST       "api.groq.com"
#define GROQ_PORT       443
#define GROQ_PATH       "/openai/v1/audio/transcriptions"
#define GROQ_MODEL      "whisper-large-v3-turbo"   // 最快、夠準
#define GROQ_LANGUAGE   "zh"                       // 中文,跳過自動偵測加速

i2s_chan_handle_t i2s_rx = nullptr;
int16_t* recBuf = nullptr;
volatile uint32_t recCount = 0;
bool recording = false;
unsigned long recStartMs = 0;

void initI2S() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num  = 8;
  chan_cfg.dma_frame_num = 512;
  i2s_new_channel(&chan_cfg, nullptr, &i2s_rx);

  i2s_std_gpio_config_t gpio_cfg = {};
  gpio_cfg.mclk = I2S_GPIO_UNUSED;
  gpio_cfg.bclk = (gpio_num_t)PIN_I2S_BCLK;
  gpio_cfg.ws   = (gpio_num_t)PIN_I2S_WS;
  gpio_cfg.dout = I2S_GPIO_UNUSED;
  gpio_cfg.din  = (gpio_num_t)PIN_I2S_DIN;

  i2s_std_config_t rx_cfg = {};
  rx_cfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  rx_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
  rx_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
  rx_cfg.gpio_cfg = gpio_cfg;
  i2s_channel_init_std_mode(i2s_rx, &rx_cfg);
  i2s_channel_enable(i2s_rx);
}

// 構造 44-byte PCM WAV header
void buildWavHeader(uint8_t* hdr, uint32_t dataLen) {
  uint32_t fileLen   = dataLen + 36;
  uint32_t byteRate  = SAMPLE_RATE * 1 * 16 / 8;
  uint16_t blockAlign = 1 * 16 / 8;
  memcpy(hdr,      "RIFF", 4);
  memcpy(hdr +  4, &fileLen, 4);
  memcpy(hdr +  8, "WAVEfmt ", 8);
  uint32_t fmtLen = 16;       memcpy(hdr + 16, &fmtLen, 4);
  uint16_t fmt   = 1;          memcpy(hdr + 20, &fmt, 2);            // PCM
  uint16_t chan  = 1;          memcpy(hdr + 22, &chan, 2);
  uint32_t sr    = SAMPLE_RATE;memcpy(hdr + 24, &sr, 4);
  memcpy(hdr + 28, &byteRate, 4);
  memcpy(hdr + 32, &blockAlign, 2);
  uint16_t bits  = 16;         memcpy(hdr + 34, &bits, 2);
  memcpy(hdr + 36, "data", 4);
  memcpy(hdr + 40, &dataLen, 4);
}

void startRec() {
  recCount   = 0;
  recording  = true;
  recStartMs = millis();
  Serial.println("[REC] 開始");
}

void stopRec();

void doRecChunk() {
  if (!recording) return;
  if (recCount >= MAX_SAMPLES) {
    Serial.println("[REC] 達上限 10 秒,自動停");
    stopRec();
    return;
  }
  int32_t raw[I2S_CHUNK];
  size_t bytesRead = 0;
  esp_err_t e = i2s_channel_read(i2s_rx, raw, sizeof(raw), &bytesRead,
                                 50 / portTICK_PERIOD_MS);
  if (e != ESP_OK || bytesRead == 0) return;
  int samples = bytesRead / 4;
  for (int i = 0; i < samples && recCount < MAX_SAMPLES; i++) {
    int32_t v = raw[i] >> 16;
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    recBuf[recCount++] = (int16_t)v;
  }
}

// 送 PCM 上 Groq Whisper,回傳辨識文字(失敗回空字串)
String transcribeWithGroq() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[GROQ] WiFi 沒連,跳過");
    return "";
  }
  uint32_t audioBytes = recCount * 2;
  if (audioBytes == 0) return "";

  Serial.printf("[GROQ] 連線 %s ...\n", GROQ_HOST);
  WiFiClientSecure client;
  client.setInsecure();   // 跳過憑證驗證(嵌入式裝不下 CA),簡單可用
  client.setTimeout(15000);
  if (!client.connect(GROQ_HOST, GROQ_PORT)) {
    Serial.println("[GROQ] TLS 連線失敗");
    return "";
  }

  // multipart body 組裝(streaming 寫出,不全部存 RAM)
  const char* boundary = "----Mode10TestBoundary";
  String head1 = String("--") + boundary + "\r\n"
               + "Content-Disposition: form-data; name=\"model\"\r\n\r\n"
               + GROQ_MODEL + "\r\n";
  String head2 = String("--") + boundary + "\r\n"
               + "Content-Disposition: form-data; name=\"language\"\r\n\r\n"
               + GROQ_LANGUAGE + "\r\n";
  String head3 = String("--") + boundary + "\r\n"
               + "Content-Disposition: form-data; name=\"response_format\"\r\n\r\n"
               + "json\r\n";
  String head4 = String("--") + boundary + "\r\n"
               + "Content-Disposition: form-data; name=\"file\"; filename=\"a.wav\"\r\n"
               + "Content-Type: audio/wav\r\n\r\n";
  String tail  = String("\r\n--") + boundary + "--\r\n";

  uint32_t wavLen = 44 + audioBytes;
  uint32_t bodyLen = head1.length() + head2.length() + head3.length()
                   + head4.length() + wavLen + tail.length();

  // HTTP request line + headers
  client.printf("POST %s HTTP/1.1\r\n", GROQ_PATH);
  client.printf("Host: %s\r\n", GROQ_HOST);
  client.printf("Authorization: Bearer %s\r\n", GROQ_API_KEY);
  client.printf("Content-Type: multipart/form-data; boundary=%s\r\n", boundary);
  client.printf("Content-Length: %u\r\n", bodyLen);
  client.print("Connection: close\r\n\r\n");

  // body
  client.print(head1);
  client.print(head2);
  client.print(head3);
  client.print(head4);
  uint8_t wavHdr[44];
  buildWavHeader(wavHdr, audioBytes);
  client.write(wavHdr, 44);
  // 音檔分塊送(避免一次 write 太大),2KB/chunk
  const size_t CHUNK = 2048;
  uint8_t* p = (uint8_t*)recBuf;
  uint32_t remain = audioBytes;
  while (remain > 0) {
    size_t n = remain > CHUNK ? CHUNK : remain;
    size_t sent = client.write(p, n);
    if (sent == 0) { Serial.println("[GROQ] write 失敗"); client.stop(); return ""; }
    p += sent; remain -= sent;
  }
  client.print(tail);

  Serial.println("[GROQ] 上傳完,等回應 ...");

  // 讀 HTTP response 全部
  unsigned long t0 = millis();
  while (!client.available() && millis() - t0 < 15000) delay(10);
  String full = "";
  while (client.available()) {
    full += client.readString();
  }
  client.stop();

  if (full.length() == 0) { Serial.println("[GROQ] 沒回應"); return ""; }
  // 印 HTTP status
  int firstLine = full.indexOf("\r\n");
  if (firstLine > 0) Serial.printf("[GROQ] %s\n", full.substring(0, firstLine).c_str());

  // 找 JSON body(第一個 { 開始)
  int jsonStart = full.indexOf('{');
  if (jsonStart < 0) {
    Serial.println("[GROQ] 沒看到 JSON,完整回應:");
    Serial.println(full);
    return "";
  }
  String body = full.substring(jsonStart);
  // 粗糙 parse:找 "text":"..." 內容
  int textKey = body.indexOf("\"text\":\"");
  if (textKey < 0) {
    Serial.println("[GROQ] JSON 無 text 欄位,body:");
    Serial.println(body);
    return "";
  }
  int textStart = textKey + 8;
  int textEnd = body.indexOf("\"", textStart);
  // 跳過跳脫的 \"
  while (textEnd > 0 && body[textEnd - 1] == '\\') {
    textEnd = body.indexOf("\"", textEnd + 1);
  }
  if (textEnd < 0) return "";
  return body.substring(textStart, textEnd);
}

void stopRec() {
  recording = false;
  unsigned long dur = millis() - recStartMs;
  int32_t maxAmp = 0;
  for (uint32_t i = 0; i < recCount; i++) {
    int32_t v = recBuf[i]; if (v < 0) v = -v;
    if (v > maxAmp) maxAmp = v;
  }
  Serial.printf("[REC] 停 | samples=%u | 時長=%lu ms | max=%ld\n",
                recCount, dur, (long)maxAmp);
  if (maxAmp < 100) {
    Serial.println("      ⚠ 訊號太弱,不上傳");
    return;
  }
  // 上傳
  unsigned long t0 = millis();
  String text = transcribeWithGroq();
  unsigned long t1 = millis() - t0;
  if (text.length() > 0) {
    Serial.printf("[STT] (%lu ms)「%s」\n", t1, text.c_str());
  } else {
    Serial.printf("[STT] 失敗 (%lu ms)\n", t1);
  }
}

void setup() {
  // 暫時關 brown-out detector(LDO 不夠時測試用,正式版要硬體解掉)
  // 危險:電壓真的掉時不會自動 reset,可能跑出怪行為。穩了之後拿掉。
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(200);
  Serial.println("\n=== Mode10_Test Phase 2:錄音 + Groq Whisper ===");
  Serial.println("⚠ brown-out 偵測已關(暫時),硬體穩定後要拿掉");

  pinMode(PIN_SHOULDER_L, INPUT_PULLUP);

  recBuf = (int16_t*)heap_caps_malloc(MAX_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM);
  if (!recBuf) {
    Serial.println("[X] PSRAM alloc 失敗,檢查 sketch.yaml 是否 PSRAM=opi");
    while (1) delay(1000);
  }
  Serial.printf("[+] PSRAM buffer:%d KB\n", (int)(MAX_SAMPLES * 2 / 1024));

  initI2S();
  Serial.println("[+] I2S 啟動");

  Serial.printf("[*] WiFi 連線 %s ...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  // 手把 3V3 LDO 規格不夠,降到 2dBm(最低)避免 brown-out。
  // 桌面距離手機熱點 1~3m 完全夠用,RSSI 大概還 -65dBm。
  WiFi.setTxPower(WIFI_POWER_2dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long wt0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wt0 < 15000) {
    delay(250); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[+] WiFi OK,IP=%s,RSSI=%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("[X] WiFi 連線失敗,錄音還能用但不會上傳");
  }

  Serial.println("[+] 按住左肩鈕(GPIO 8)講話、放開上傳");
}

void loop() {
  static bool lastBtn = false;
  bool btn = (digitalRead(PIN_SHOULDER_L) == LOW);
  if (btn && !lastBtn) startRec();
  if (!btn && lastBtn && recording) stopRec();
  lastBtn = btn;

  doRecChunk();
}
