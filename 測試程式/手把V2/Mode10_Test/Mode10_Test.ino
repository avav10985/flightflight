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
#include <LovyanGFX.hpp>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "secrets.h"

#define PIN_I2S_BCLK     11
#define PIN_I2S_WS       12
#define PIN_I2S_DIN      14
#define PIN_AMP_SD       18    // MAX98357A SD(shutdown):LOW=靜音,HIGH=啟用
#define PIN_SHOULDER_L    8

// ====== TFT(沿用 Ground_TX_ESP32 配置)======
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

// TFT 顯示 helper
void tftStatus(const char* line, uint16_t color = TFT_YELLOW) {
  tft.fillRect(0, 60, 240, 80, TFT_BLACK);
  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(10, 80);
  tft.print(line);
}

void tftTranscript(const char* text) {
  tft.fillRect(0, 160, 240, 110, TFT_BLACK);
  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, 170);
  tft.print("辨識:");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 200);
  tft.println(text);
}

void tftClearTranscript() {
  tft.fillRect(0, 160, 240, 110, TFT_BLACK);
}

void tftHint(const char* text) {
  tft.fillRect(0, 280, 240, 40, TFT_BLACK);
  tft.setFont(&fonts::efontTW_14);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(10, 290);
  tft.print(text);
}

#define SAMPLE_RATE     16000
#define MAX_REC_SECS       10
#define MAX_SAMPLES     (SAMPLE_RATE * MAX_REC_SECS)
#define I2S_CHUNK         512

// Groq API:Whisper STT + Llama 解析
#define GROQ_HOST       "api.groq.com"
#define GROQ_PORT       443
#define GROQ_STT_PATH   "/openai/v1/audio/transcriptions"
#define GROQ_CHAT_PATH  "/openai/v1/chat/completions"
#define GROQ_STT_MODEL  "whisper-large-v3-turbo"
#define GROQ_LLM_MODEL  "llama-3.3-70b-versatile"
#define GROQ_LANGUAGE   "zh"

// Llama prompt(放在 system message)
#define LLM_SYSTEM_PROMPT "你是無人機語音指令解析器。把使用者的中文輸入翻成 JSON。可用 action: takeoff(起飛)、land(降落)、move(移動,要 direction: up/down/left/right/forward/back 跟 duration_sec 1-10)、stop(停止/懸停)、mode(切模式,要 mode: 0/1/2)。聽不懂回 {\\\"action\\\":\\\"unknown\\\"}。只回 JSON,不要其他文字。"

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
  tftStatus("● 錄音中...", TFT_RED);
  tftClearTranscript();   // 清掉上次結果
  tftHint("放開肩鈕送出");
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

// 確認 WiFi 連著,沒連就重連(最多等 8 秒)
bool ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  Serial.println("[WiFi] 失聯,重連 ...");
  tftStatus("WiFi 重連 ...", TFT_YELLOW);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(200);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] 重連成功 RSSI=%d\n", WiFi.RSSI());
    return true;
  }
  Serial.println("[WiFi] 重連失敗");
  return false;
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
               + GROQ_STT_MODEL + "\r\n";
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
  client.printf("POST %s HTTP/1.1\r\n", GROQ_STT_PATH);
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

// JSON 字串需要 escape 的字元(quote / backslash / control chars)
String jsonEscape(const String& s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else out += c;
  }
  return out;
}

// 反向:把 JSON escape 過的字串解開(把 \" 變回 ")
String jsonUnescape(const String& s) {
  String out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    if (s[i] == '\\' && i + 1 < s.length()) {
      char n = s[i + 1];
      switch (n) {
        case '"':  out += '"'; break;
        case '\\': out += '\\'; break;
        case '/':  out += '/'; break;
        case 'n':  out += '\n'; break;
        case 'r':  out += '\r'; break;
        case 't':  out += '\t'; break;
        default:   out += n; break;
      }
      i++;
    } else {
      out += s[i];
    }
  }
  return out;
}

// Base64 編碼(輸入 inLen bytes,寫出 ceil(inLen/3)*4 chars 到 out)
static const char B64_CHARSET[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
void base64Encode(const uint8_t* in, size_t inLen, char* out) {
  size_t i = 0, j = 0;
  while (i + 2 < inLen) {
    uint32_t v = ((uint32_t)in[i] << 16) | ((uint32_t)in[i+1] << 8) | in[i+2];
    out[j++] = B64_CHARSET[(v >> 18) & 0x3F];
    out[j++] = B64_CHARSET[(v >> 12) & 0x3F];
    out[j++] = B64_CHARSET[(v >> 6)  & 0x3F];
    out[j++] = B64_CHARSET[v & 0x3F];
    i += 3;
  }
  if (i < inLen) {
    uint32_t v = (uint32_t)in[i] << 16;
    if (i + 1 < inLen) v |= (uint32_t)in[i+1] << 8;
    out[j++] = B64_CHARSET[(v >> 18) & 0x3F];
    out[j++] = B64_CHARSET[(v >> 12) & 0x3F];
    out[j++] = (i + 1 < inLen) ? B64_CHARSET[(v >> 6) & 0x3F] : '=';
    out[j++] = '=';
  }
}

// 失敗時填這個,讓 TFT 顯示具體錯誤
// 留錯誤訊息給 TFT 顯示
String g_lastErr = "";

// 文字 → Groq Llama → JSON 指令字串(失敗回空字串)
String parseCommandWithLlama(const String& userText) {
  g_lastErr = "";
  if (WiFi.status() != WL_CONNECTED) { g_lastErr = "WiFi 斷"; return ""; }
  if (userText.length() == 0) return "";

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(10000);
  if (!client.connect(GROQ_HOST, GROQ_PORT)) {
    g_lastErr = "Llama TLS 失敗";
    return "";
  }

  String userEsc = jsonEscape(userText);
  String body = String("{\"model\":\"") + GROQ_LLM_MODEL + "\","
              + "\"messages\":["
              + "{\"role\":\"system\",\"content\":\"" + LLM_SYSTEM_PROMPT + "\"},"
              + "{\"role\":\"user\",\"content\":\"起飛\"},"
              + "{\"role\":\"assistant\",\"content\":\"{\\\"action\\\":\\\"takeoff\\\"}\"},"
              + "{\"role\":\"user\",\"content\":\"降落\"},"
              + "{\"role\":\"assistant\",\"content\":\"{\\\"action\\\":\\\"land\\\"}\"},"
              + "{\"role\":\"user\",\"content\":\"停下來\"},"
              + "{\"role\":\"assistant\",\"content\":\"{\\\"action\\\":\\\"stop\\\"}\"},"
              + "{\"role\":\"user\",\"content\":\"向前飛三秒\"},"
              + "{\"role\":\"assistant\",\"content\":\"{\\\"action\\\":\\\"move\\\",\\\"direction\\\":\\\"forward\\\",\\\"duration_sec\\\":3}\"},"
              + "{\"role\":\"user\",\"content\":\"切到 GPS 模式\"},"
              + "{\"role\":\"assistant\",\"content\":\"{\\\"action\\\":\\\"mode\\\",\\\"mode\\\":2}\"},"
              + "{\"role\":\"user\",\"content\":\"" + userEsc + "\"}"
              + "],"
              + "\"response_format\":{\"type\":\"json_object\"},"
              + "\"temperature\":0.1}";

  client.printf("POST %s HTTP/1.1\r\n", GROQ_CHAT_PATH);
  client.printf("Host: %s\r\n", GROQ_HOST);
  client.printf("Authorization: Bearer %s\r\n", GROQ_API_KEY);
  client.print("Content-Type: application/json\r\n");
  client.printf("Content-Length: %d\r\n", body.length());
  client.print("Connection: close\r\n\r\n");
  client.print(body);

  unsigned long t0 = millis();
  while (!client.available() && millis() - t0 < 10000) delay(10);
  String full = "";
  while (client.available()) full += client.readString();
  client.stop();

  if (full.length() == 0) { g_lastErr = "Llama 無回應"; return ""; }
  int first = full.indexOf("\r\n");
  String statusLine = (first > 0) ? full.substring(0, first) : "??";
  Serial.printf("[LLM] %s\n", statusLine.c_str());
  if (statusLine.indexOf("200") < 0) {
    g_lastErr = statusLine.substring(0, 24);
    Serial.println(full);
    return "";
  }

  int jsonStart = full.indexOf('{');
  if (jsonStart < 0) { g_lastErr = "Llama 無 JSON"; return ""; }
  String resBody = full.substring(jsonStart);
  int contentKey = resBody.indexOf("\"content\":\"");
  if (contentKey < 0) { g_lastErr = "無 content"; return ""; }
  int cs = contentKey + 11;
  int ce = cs;
  while (ce < (int)resBody.length()) {
    if (resBody[ce] == '\\' && ce + 1 < (int)resBody.length()) { ce += 2; continue; }
    if (resBody[ce] == '"') break;
    ce++;
  }
  if (ce >= (int)resBody.length()) return "";
  return jsonUnescape(resBody.substring(cs, ce));
}

#if 0   // Gemini 版本暫時封存,撞 429 暫不用,留著之後可能恢復
String processAudioWithGemini() {
  g_geminiErr = "";
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[GEM] WiFi 沒連");
    return "";
  }
  uint32_t pcmBytes = recCount * 2;
  if (pcmBytes == 0) return "";
  uint32_t wavBytes = 44 + pcmBytes;
  uint32_t b64Bytes = ((wavBytes + 2) / 3) * 4;

  // PSRAM 配置 WAV + base64 buffer(臨時)
  uint8_t* wav = (uint8_t*)heap_caps_malloc(wavBytes, MALLOC_CAP_SPIRAM);
  if (!wav) { Serial.println("[GEM] PSRAM wav alloc 失敗"); return ""; }
  buildWavHeader(wav, pcmBytes);
  memcpy(wav + 44, recBuf, pcmBytes);

  char* b64 = (char*)heap_caps_malloc(b64Bytes + 1, MALLOC_CAP_SPIRAM);
  if (!b64) { heap_caps_free(wav); Serial.println("[GEM] PSRAM b64 alloc 失敗"); return ""; }
  base64Encode(wav, wavBytes, b64);
  b64[b64Bytes] = 0;
  heap_caps_free(wav);

  Serial.printf("[GEM] 連線 Gemini ... (audio %u → base64 %u bytes)\n", wavBytes, b64Bytes);
  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(15000);
  if (!client.connect(GEMINI_HOST, GEMINI_PORT)) {
    Serial.println("[GEM] TLS 連線失敗");
    g_geminiErr = "TLS 失敗";
    heap_caps_free(b64);
    return "";
  }

  // JSON body 前後段(中間塞 base64)
  String head = String("{\"contents\":[{\"parts\":["
              "{\"text\":\"") + GEMINI_PROMPT + "\"},"
              "{\"inline_data\":{\"mime_type\":\"audio/wav\",\"data\":\"";
  String tail = "\"}}]}],\"generationConfig\":{"
                "\"responseMimeType\":\"application/json\","
                "\"temperature\":0.1,\"maxOutputTokens\":256}}";
  uint32_t bodyLen = head.length() + b64Bytes + tail.length();

  String path = String("/v1beta/models/") + GEMINI_MODEL
              + ":generateContent?key=" + GEMINI_API_KEY;

  client.printf("POST %s HTTP/1.1\r\n", path.c_str());
  client.printf("Host: %s\r\n", GEMINI_HOST);
  client.print("Content-Type: application/json\r\n");
  client.printf("Content-Length: %u\r\n", bodyLen);
  client.print("Connection: close\r\n\r\n");
  client.print(head);
  // base64 分塊送(避免一次 write 太多)
  const size_t CHUNK = 4096;
  uint32_t remain = b64Bytes;
  char* p = b64;
  while (remain > 0) {
    size_t n = remain > CHUNK ? CHUNK : remain;
    client.write((const uint8_t*)p, n);
    p += n; remain -= n;
  }
  client.print(tail);
  heap_caps_free(b64);

  Serial.println("[GEM] 上傳完,等回應 ...");
  unsigned long t0 = millis();
  while (!client.available() && millis() - t0 < 15000) delay(10);
  String full = "";
  while (client.available()) full += client.readString();
  client.stop();

  if (full.length() == 0) {
    Serial.println("[GEM] 沒回應");
    g_geminiErr = "無回應";
    return "";
  }
  int first = full.indexOf("\r\n");
  String statusLine = (first > 0) ? full.substring(0, first) : "??";
  Serial.printf("[GEM] %s\n", statusLine.c_str());

  // 檢查 HTTP 狀態
  if (statusLine.indexOf("200") < 0) {
    // 不是 200,把狀態碼跟 body 顯示
    g_geminiErr = statusLine.substring(0, 24);
    Serial.println("[GEM] HTTP 錯誤,完整回應:");
    Serial.println(full);
    return "";
  }

  // 抽 candidates[0].content.parts[0].text(裡面又是 JSON 字串)
  int jsonStart = full.indexOf('{');
  if (jsonStart < 0) { g_geminiErr = "無 JSON body"; return ""; }
  String resBody = full.substring(jsonStart);
  int textKey = resBody.indexOf("\"text\":\"");
  if (textKey < 0) {
    Serial.println("[GEM] 無 text 欄位:");
    Serial.println(resBody);
    g_geminiErr = "無 text 欄位";
    // 看看是不是 Gemini 回了錯誤(例如 promptFeedback)
    int errKey = resBody.indexOf("\"message\":\"");
    if (errKey > 0) {
      int es = errKey + 11;
      int ee = resBody.indexOf("\"", es);
      if (ee > 0) g_geminiErr = resBody.substring(es, min(ee, es + 30));
    }
    return "";
  }
  int ts = textKey + 8;
  int te = ts;
  while (te < (int)resBody.length()) {
    if (resBody[te] == '\\' && te + 1 < (int)resBody.length()) { te += 2; continue; }
    if (resBody[te] == '"') break;
    te++;
  }
  if (te >= (int)resBody.length()) return "";
  String textEscaped = resBody.substring(ts, te);
  return jsonUnescape(textEscaped);
}
#endif   // Gemini 程式碼結束(撞 429 暫停用)

// 從回傳 JSON 抽 transcript 欄位
String extractTranscript(const String& json) {
  int idx = json.indexOf("\"transcript\":\"");
  if (idx < 0) return "";
  int s = idx + 14;
  int e = json.indexOf("\"", s);
  if (e < 0) return "";
  return json.substring(s, e);
}

// 從 LLM 回傳的 JSON 字串抽 action 欄位
String extractAction(const String& jsonContent) {
  int idx = jsonContent.indexOf("\"action\":\"");
  if (idx < 0) return "unknown";
  int s = idx + 10;
  int e = jsonContent.indexOf("\"", s);
  if (e < 0) return "unknown";
  return jsonContent.substring(s, e);
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
    tftStatus("訊號太弱", TFT_ORANGE);
    tftHint("按肩鈕重試,講大聲一點");
    return;
  }
  // Phase 2:Groq Whisper STT
  if (!ensureWiFi()) {
    tftStatus("WiFi 失聯", TFT_RED);
    tftHint("確認熱點開著再試");
    return;
  }
  tftStatus("上傳中...", TFT_YELLOW);
  unsigned long t0 = millis();
  String text = transcribeWithGroq();
  unsigned long t1 = millis() - t0;
  if (text.length() == 0) {
    char ebuf[64];
    snprintf(ebuf, sizeof(ebuf), "STT 失敗");
    tftStatus(ebuf, TFT_RED);
    tftHint("按肩鈕重試");
    return;
  }
  Serial.printf("[STT] (%lu ms)「%s」\n", t1, text.c_str());
  tftTranscript(text.c_str());

  // 清掉 Whisper 常加的標點 / 空白,避免 Llama 認不出來
  String cleanText = text;
  cleanText.trim();
  // 移除常見中英文標點
  String punct = "。,!?,.!?;:、 \"'";
  while (cleanText.length() > 0 && punct.indexOf(cleanText[cleanText.length()-1]) >= 0) {
    cleanText.remove(cleanText.length() - 1);
  }
  Serial.printf("[STT] 清理後「%s」\n", cleanText.c_str());

  // Phase 3:Groq Llama 解析
  tftStatus("解析中...", TFT_YELLOW);
  unsigned long t2 = millis();
  String llmJson = parseCommandWithLlama(cleanText);
  unsigned long t3 = millis() - t2;
  if (llmJson.length() == 0) {
    char ebuf[64];
    snprintf(ebuf, sizeof(ebuf), "失敗:%s",
             g_lastErr.length() > 0 ? g_lastErr.c_str() : "Llama");
    tftStatus(ebuf, TFT_RED);
    tftHint("按肩鈕重試");
    return;
  }
  Serial.printf("[LLM] (%lu ms) %s\n", t3, llmJson.c_str());
  String action = extractAction(llmJson);
  char buf[64];
  snprintf(buf, sizeof(buf), "→ %s", action.c_str());
  tftStatus(buf, (action == "unknown") ? TFT_ORANGE : TFT_GREEN);
  // unknown 時 hint 印 Llama 回來的真實 JSON,直接看哪邊出問題
  if (action == "unknown") {
    // 截斷太長的 JSON,只顯示前 40 字
    String shortJson = llmJson;
    if (shortJson.length() > 40) shortJson = shortJson.substring(0, 40);
    tftHint(shortJson.c_str());
  } else {
    tftHint("按肩鈕說下一句");
  }
}

void setup() {
  // 暫時關 brown-out detector(S3 要清 ENA bit,不能整個 reg 寫 0)
  // 危險:電壓真的掉時不會自動 reset,可能跑出怪行為。穩了之後拿掉。
  REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_INT_ENA);
  REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(200);
  Serial.println("\n=== Mode10_Test Phase 2:錄音 + Groq Whisper ===");
  Serial.println("⚠ brown-out 偵測已關(暫時),硬體穩定後要拿掉");

  // TFT 啟動
  tft.init();
  tft.setRotation(2);   // 直立 240×320,跟 Ground_TX_ESP32 / Voice_Test 同方向
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, 240, 50, TFT_NAVY);
  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.setCursor(20, 12);
  tft.print("Mode 10 語音");
  tftStatus("開機中...", TFT_YELLOW);

  // 立刻把 MAX98357A 拉 LOW(shutdown)避免開機/I2S 啟動時放大雜訊
  // Phase 5 接 TTS 才會在播放前拉 HIGH、播完拉回 LOW
  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, LOW);

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
  tftStatus("WiFi 連線中...", TFT_YELLOW);
  WiFi.mode(WIFI_STA);
  // 8.5dBm 折衷:範圍夠用,WiFi TX 尖峰電流不會打掛 buck 5V 軌
  // (預設 19.5dBm 在 2S buck 架構下會偶爾重啟 / 連線失敗)
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long wt0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wt0 < 15000) {
    delay(250); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[+] WiFi OK,IP=%s,RSSI=%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    tftStatus("就緒", TFT_GREEN);
    tftHint("按住肩鈕說話");
  } else {
    Serial.println("[X] WiFi 連線失敗");
    tftStatus("WiFi 失敗", TFT_RED);
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
