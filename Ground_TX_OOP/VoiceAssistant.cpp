#include "VoiceAssistant.h"
#if ENABLE_VOICE_MODE10
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "secrets.h"                // WIFI_SSID / WIFI_PASS / GROQ_API_KEY(gitignore 擋住)

// --- Demo 旗標:還不能穩飛之前,語音動作一律轉成「單馬達輕轉」 ---
// 1 = demo(起飛→轉1號、轉N號馬達→轉N號、降落/停→停轉;不碰虛擬搖桿)
// 0 = 完整飛行映射(虛擬搖桿 takeoff/hover/move/land)
#define VOICE_DEMO_SPIN  1

// --- 可調參數(拆槳地面測試後修正)---
#define VOICE_THR_HOVER  135   // 懸停虛擬油門(0~255);實測懸停點後改
#define VOICE_TILT        50   // move 搖桿偏移(50 ≈ 7.9°,MAX_ANGLE 20° 時)
#define VOICE_THR_VERT    10   // 上升/下降的油門增減
#define VOICE_PITCH_FWD   -1   // 「前進」的 pitch 偏移正負(拆槳測試確認)
#define VOICE_ROLL_RIGHT  +1   // 「右」的 roll 偏移正負(拆槳測試確認)

#define VREC_RATE     32000          // I²S 實際取樣(跟音樂共用時脈,不能改)
#define VREC_WAV_RATE 16000          // 上傳取樣率:2:1 降採樣,檔小一半傳更快
#define VREC_MAXSEC   8
#define VREC_MAXSAMP  (VREC_WAV_RATE * VREC_MAXSEC)

// ---- JSON 工具(檔內共用)----
static String vJsonEscape(const String& s) {
  String out; out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else out += c;
  }
  return out;
}

static String vJsonUnescape(const String& s) {
  String out; out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    if (s[i] == '\\' && i + 1 < s.length()) {
      char n = s[i + 1];
      switch (n) {
        case '"': out += '"'; break;  case '\\': out += '\\'; break;
        case '/': out += '/'; break;  case 'n': out += '\n'; break;
        case 'r': out += '\r'; break; case 't': out += '\t'; break;
        default:  out += n; break;
      }
      i++;
    } else out += s[i];
  }
  return out;
}

// ============================================================
// 生命週期
// ============================================================
bool VoiceAssistant::begin(i2s_chan_handle_t micRx) {
  micRx_ = micRx;
  // 關閉 task watchdog:語音 task 的 TLS 加密/上傳即使逐塊讓出 CPU,
  // 仍可能讓核心 0 IDLE 超過 5 秒沒跑滿觸發重啟(2026-06-12 兩度實測)。
  // 手把非飛行安全件,關掉換穩定;之後 ESP-SR 本地辨識取代雲端就能開回來。
  esp_task_wdt_deinit();
  recBuf_ = (int16_t*)heap_caps_malloc(VREC_MAXSAMP * 2, MALLOC_CAP_SPIRAM);
  if (!recBuf_) return false;
  xTaskCreatePinnedToCore(taskEntry, "voice", 16384, this, 1, NULL, 0);
  return true;
}

void VoiceAssistant::enter() {
  prog_ = VP_NONE;
  drawVoiceStatic();
  mode10_ = true;
}

void VoiceAssistant::exit() {
  mode10_ = false;
  ptt_    = false;
  prog_   = VP_NONE;
  // WiFi 保持連線(下次進來免等);要省電再說
}

void VoiceAssistant::setStatus(const char* s) {
  strncpy(status_, s, sizeof(status_) - 1);
  status_[sizeof(status_) - 1] = 0;
}

// ============================================================
// 虛擬搖桿覆寫(主迴圈端)
// ============================================================
void VoiceAssistant::applyOverride(Signal& data) {
#if VOICE_DEMO_SPIN
  // Demo:語音動作 → 單馬達輕轉指令(paramID 110/111),不碰虛擬搖桿。
  // 飛機端只在「未武裝 + 油門收底」才執行,轉 3 秒自動停。
  if (fresh_) {
    fresh_ = false;
    static float spinPulse = 0;       // 每次 +10,飛機 dedup 看值變化;%10 取馬達編號
    spinPulse += 10;
    if (action_ == 1)      { data.paramID = 110; data.paramVal = 1      + spinPulse; }  // 起飛→轉1號
    else if (action_ == 5) { data.paramID = 110; data.paramVal = motor_ + spinPulse; }  // 轉N號馬達
    else if (action_ == 2 || action_ == 3) { data.paramID = 111; data.paramVal = spinPulse; }  // 降落/停→停轉
  }
  return;
#endif
  // 收到新動作 → 啟動程式,記錄搖桿基準
  if (fresh_) {
    fresh_ = false;
    switch (action_) {
      case 1: prog_ = VP_TAKEOFF; break;
      case 2: prog_ = VP_LAND;    break;
      case 3: prog_ = VP_HOVER;   break;
      case 4: prog_ = VP_MOVE;
              mx_ = dirX_; my_ = dirY_; mz_ = dirZ_;
              moveDur_ = dur_ ? dur_ : 2;
              if (moveDur_ > 10) moveDur_ = 10;
              break;
      default: return;   // unknown → 不動作
    }
    progT0_ = millis();
    baseThr_ = data.throttle; basePitch_ = data.pitch; baseRoll_ = data.roll;
  }
  if (prog_ == VP_NONE) return;

  // 安全 1:搖桿任何軸偏離「程式啟動時的位置」> 25 → 立刻搶回手動
  if (abs((int)data.throttle - baseThr_)   > 25 ||
      abs((int)data.pitch    - basePitch_) > 25 ||
      abs((int)data.roll     - baseRoll_)  > 25) {
    prog_ = VP_NONE;
    return;
  }

  float t = (millis() - progT0_) / 1000.0f;
  int thr = VOICE_THR_HOVER, pit = 127, rol = 127;

  switch (prog_) {
    case VP_TAKEOFF:               // 1.5 秒線性升到懸停油門
      thr = (int)(VOICE_THR_HOVER * (t / 1.5f));
      if (t >= 1.5f) { prog_ = VP_HOVER; progT0_ = millis(); thr = VOICE_THR_HOVER; }
      break;
    case VP_HOVER:                 // 維持到搖桿介入 / 新指令 / 切模式
      break;
    case VP_MOVE:
      thr = VOICE_THR_HOVER + mz_ * VOICE_THR_VERT;
      pit = 127 + my_ * VOICE_TILT;
      rol = 127 + mx_ * VOICE_TILT;
      if (t >= moveDur_) { prog_ = VP_HOVER; progT0_ = millis(); }
      break;
    case VP_LAND: {                // 3 秒線性收油門到 0
      float k = 1.0f - t / 3.0f;
      if (k <= 0) { thr = 0; prog_ = VP_NONE; }
      else        thr = (int)(VOICE_THR_HOVER * k);
      break;
    }
    default: break;
  }
  data.throttle = (uint8_t)constrain(thr, 0, 255);
  data.pitch    = (uint8_t)constrain(pit, 0, 255);
  data.roll     = (uint8_t)constrain(rol, 0, 255);
  data.yaw      = 127;
}

// ============================================================
// 語音 task 端(核心 0):HTTP 工具(Mode10_Test 移植)
// ============================================================
// PCM → WAV → Groq Whisper STT(在 task 上跑,阻塞沒關係)
String VoiceAssistant::transcribe() {
  uint32_t audioBytes = recCount_ * 2;
  if (audioBytes == 0) return "";
  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(12000);
  if (!client.connect("api.groq.com", 443)) { setStatus("STT 連線失敗"); return ""; }

  const char* boundary = "----GTXVoiceBoundary";
  String head1 = String("--") + boundary + "\r\nContent-Disposition: form-data; name=\"model\"\r\n\r\nwhisper-large-v3-turbo\r\n";
  String head2 = String("--") + boundary + "\r\nContent-Disposition: form-data; name=\"language\"\r\n\r\nzh\r\n";
  String head3 = String("--") + boundary + "\r\nContent-Disposition: form-data; name=\"response_format\"\r\n\r\njson\r\n";
  String head4 = String("--") + boundary + "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"a.wav\"\r\nContent-Type: audio/wav\r\n\r\n";
  String tail  = String("\r\n--") + boundary + "--\r\n";
  uint32_t wavLen  = 44 + audioBytes;
  uint32_t bodyLen = head1.length() + head2.length() + head3.length() + head4.length() + wavLen + tail.length();

  client.printf("POST /openai/v1/audio/transcriptions HTTP/1.1\r\n");
  client.print("Host: api.groq.com\r\n");
  client.printf("Authorization: Bearer %s\r\n", GROQ_API_KEY);
  client.printf("Content-Type: multipart/form-data; boundary=%s\r\n", boundary);
  client.printf("Content-Length: %u\r\n", bodyLen);
  client.print("Connection: close\r\n\r\n");
  client.print(head1); client.print(head2); client.print(head3); client.print(head4);

  // 44-byte WAV header
  uint8_t h[44];
  memcpy(h, "RIFF", 4); uint32_t v = 36 + audioBytes; memcpy(h + 4, &v, 4);
  memcpy(h + 8, "WAVEfmt ", 8); v = 16; memcpy(h + 16, &v, 4);
  uint16_t u = 1; memcpy(h + 20, &u, 2); memcpy(h + 22, &u, 2);
  v = VREC_WAV_RATE;     memcpy(h + 24, &v, 4);
  v = VREC_WAV_RATE * 2; memcpy(h + 28, &v, 4);
  u = 2; memcpy(h + 32, &u, 2); u = 16; memcpy(h + 34, &u, 2);
  memcpy(h + 36, "data", 4); memcpy(h + 40, &audioBytes, 4);
  client.write(h, 44);

  // PCM 分塊送
  uint8_t* p = (uint8_t*)recBuf_;
  uint32_t remain = audioBytes;
  while (remain > 0) {
    size_t n = remain > 4096 ? 4096 : remain;
    client.write(p, n);
    p += n; remain -= n;
    vTaskDelay(1);   // 每塊讓出 CPU:TLS 加密+上傳連續佔核心 0 會餓死
                     // IDLE0 觸發 task WDT 重啟(2026-06-12 實測)
  }
  client.print(tail);

  unsigned long t0 = millis();
  while (!client.available() && millis() - t0 < 12000) vTaskDelay(10 / portTICK_PERIOD_MS);
  String full = "";
  while (client.available()) { full += client.readString(); vTaskDelay(1); }
  client.stop();
  if (full.length() == 0) { setStatus("STT 無回應"); return ""; }
  if (full.indexOf("200") < 0 && full.indexOf("\r\n") > 0) {
    setStatus("STT HTTP 錯誤"); return "";
  }
  int k = full.indexOf("\"text\":\"");
  if (k < 0) { setStatus("STT 無結果"); return ""; }
  int s = k + 8, e = s;
  while (e < (int)full.length()) {
    if (full[e] == '\\' && e + 1 < (int)full.length()) { e += 2; continue; }
    if (full[e] == '"') break;
    e++;
  }
  String text = vJsonUnescape(full.substring(s, e));
  // 清標點(Mode10_Test 老坑)
  text.trim();
  String punct = "。,!?,.!?;:、 \"'";
  while (text.length() > 0 && punct.indexOf(text[text.length() - 1]) >= 0)
    text.remove(text.length() - 1);
  return text;
}

// 文字 → Llama → action JSON
String VoiceAssistant::parseLlama(const String& userText) {
  if (userText.length() == 0) return "";
  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(10000);
  if (!client.connect("api.groq.com", 443)) { setStatus("LLM 連線失敗"); return ""; }

  String body = String("{\"model\":\"llama-3.3-70b-versatile\",\"messages\":["
    "{\"role\":\"system\",\"content\":\"你是無人機語音指令解析器。把使用者的中文輸入翻成 JSON。可用 action: takeoff(起飛)、land(降落)、move(移動,要 direction: up/down/left/right/forward/back 跟 duration_sec 1-10)、stop(停止/懸停)、spin(轉馬達,要 motor: 1-4)。聽不懂回 {\\\"action\\\":\\\"unknown\\\"}。只回 JSON。\"},"
    "{\"role\":\"user\",\"content\":\"起飛\"},{\"role\":\"assistant\",\"content\":\"{\\\"action\\\":\\\"takeoff\\\"}\"},"
    "{\"role\":\"user\",\"content\":\"降落\"},{\"role\":\"assistant\",\"content\":\"{\\\"action\\\":\\\"land\\\"}\"},"
    "{\"role\":\"user\",\"content\":\"停下來\"},{\"role\":\"assistant\",\"content\":\"{\\\"action\\\":\\\"stop\\\"}\"},"
    "{\"role\":\"user\",\"content\":\"向前飛三秒\"},{\"role\":\"assistant\",\"content\":\"{\\\"action\\\":\\\"move\\\",\\\"direction\\\":\\\"forward\\\",\\\"duration_sec\\\":3}\"},"
    "{\"role\":\"user\",\"content\":\"轉一號馬達\"},{\"role\":\"assistant\",\"content\":\"{\\\"action\\\":\\\"spin\\\",\\\"motor\\\":1}\"},"
    "{\"role\":\"user\",\"content\":\"") + vJsonEscape(userText) + "\"}],"
    "\"response_format\":{\"type\":\"json_object\"},\"temperature\":0.1}";

  client.print("POST /openai/v1/chat/completions HTTP/1.1\r\n");
  client.print("Host: api.groq.com\r\n");
  client.printf("Authorization: Bearer %s\r\n", GROQ_API_KEY);
  client.print("Content-Type: application/json\r\n");
  client.printf("Content-Length: %d\r\n", body.length());
  client.print("Connection: close\r\n\r\n");
  client.print(body);

  unsigned long t0 = millis();
  while (!client.available() && millis() - t0 < 10000) vTaskDelay(10 / portTICK_PERIOD_MS);
  String full = "";
  while (client.available()) { full += client.readString(); vTaskDelay(1); }
  client.stop();
  if (full.length() == 0) { setStatus("LLM 無回應"); return ""; }
  int ck = full.indexOf("\"content\":\"");
  if (ck < 0) { setStatus("LLM 無結果"); return ""; }
  int s = ck + 11, e = s;
  while (e < (int)full.length()) {
    if (full[e] == '\\' && e + 1 < (int)full.length()) { e += 2; continue; }
    if (full[e] == '"') break;
    e++;
  }
  return vJsonUnescape(full.substring(s, e));
}

// 從 action JSON 抽欄位 → 寫共享變數
void VoiceAssistant::emitAction(const String& json, const String& transcript) {
  // 容錯解析:Llama 有時回美化 JSON(冒號後有空格/換行),
  // 不能假設 "key":"value" 緊貼 → 找 key 後再找冒號、再找下一對引號
  auto field = [&](const char* key) -> String {
    String pat = String("\"") + key + "\"";
    int i = json.indexOf(pat);
    if (i < 0) return "";
    int c = json.indexOf(":", i + pat.length());
    if (c < 0) return "";
    int q1 = json.indexOf("\"", c + 1);
    if (q1 < 0) return "";
    int q2 = json.indexOf("\"", q1 + 1);
    if (q2 < 0) return "";
    return json.substring(q1 + 1, q2);
  };
  String act = field("action");
  char buf[48];
  if (act == "takeoff")      { action_ = 1; speak_ = 1; snprintf(buf, sizeof(buf), "「%s」→起飛", transcript.c_str()); }
  else if (act == "land")    { action_ = 2; speak_ = 2; snprintf(buf, sizeof(buf), "「%s」→降落", transcript.c_str()); }
  else if (act == "stop")    { action_ = 3; speak_ = 3; snprintf(buf, sizeof(buf), "「%s」→懸停", transcript.c_str()); }
  else if (act == "move") {
    action_ = 4;
    String dir = field("direction");
    dirX_ = dirY_ = dirZ_ = 0;
    if (dir == "forward") dirY_ = VOICE_PITCH_FWD;
    if (dir == "back")    dirY_ = -VOICE_PITCH_FWD;
    if (dir == "right")   dirX_ = VOICE_ROLL_RIGHT;
    if (dir == "left")    dirX_ = -VOICE_ROLL_RIGHT;
    if (dir == "up")      dirZ_ = +1;
    if (dir == "down")    dirZ_ = -1;
    int di = json.indexOf("\"duration_sec\":");
    dur_ = (di > 0) ? (uint8_t)json.substring(di + 15).toInt() : 2;
    snprintf(buf, sizeof(buf), "「%s」→移動 %ds", transcript.c_str(), dur_);
  }
  else if (act == "spin") {
    action_ = 5;
    int mi = json.indexOf("\"motor\":");
    int m = (mi > 0) ? json.substring(mi + 8).toInt() : 1;
    if (m < 1 || m > 4) m = 1;
    motor_ = (uint8_t)m;
    speak_ = (uint8_t)(3 + m);   // 4~7 = motor1~4
    snprintf(buf, sizeof(buf), "「%s」→轉%d號馬達", transcript.c_str(), m);
  }
  else { action_ = 0; speak_ = 8; snprintf(buf, sizeof(buf), "「%s」→聽不懂", transcript.c_str()); setStatus(buf); return; }
  setStatus(buf);
  fresh_ = true;
}

// ---- 語音 task 主體(核心 0)----
void VoiceAssistant::taskEntry(void* arg) {
  ((VoiceAssistant*)arg)->taskLoop();
}

void VoiceAssistant::taskLoop() {
  bool wifiStarted = false;
  for (;;) {
    if (!mode10_) { vTaskDelay(100 / portTICK_PERIOD_MS); continue; }

    // WiFi 連線(第一次進 mode 10 才連,之後保持)
    if (!wifiStarted) {
      setStatus("WiFi 連線中...");
      WiFi.mode(WIFI_STA);
      WiFi.setTxPower(WIFI_POWER_8_5dBm);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      wifiStarted = true;
    }
    if (WiFi.status() != WL_CONNECTED) {
      static unsigned long lastTry = 0;
      if (millis() - lastTry > 8000) { WiFi.disconnect(); WiFi.begin(WIFI_SSID, WIFI_PASS); lastTry = millis(); }
      setStatus("等待 WiFi...");
      vTaskDelay(300 / portTICK_PERIOD_MS);
      continue;
    }

    if (!ptt_) { setStatus("就緒:按住左肩鈕說話"); vTaskDelay(50 / portTICK_PERIOD_MS); continue; }

    // --- 錄音(按住期間)---
    setStatus("錄音中...");
    recCount_ = 0;
    i2s_channel_enable(micRx_);
    static int32_t raw[256];
    while (ptt_ && mode10_ && recCount_ < VREC_MAXSAMP) {
      size_t br = 0;
      if (i2s_channel_read(micRx_, raw, sizeof(raw), &br, 80 / portTICK_PERIOD_MS) == ESP_OK) {
        int n = br / 4;
        for (int i = 0; i < n && recCount_ < VREC_MAXSAMP; i += 2) {   // 2:1 降採樣 32k→16k
          int32_t s = raw[i] >> 16;
          if (s > 32767) s = 32767;
          if (s < -32768) s = -32768;
          recBuf_[recCount_++] = (int16_t)s;
        }
      }
    }
    i2s_channel_disable(micRx_);
    if (!mode10_) continue;

    // 太短 / 太弱不上傳
    if (recCount_ < VREC_WAV_RATE / 4) { setStatus("太短,再試一次"); vTaskDelay(800 / portTICK_PERIOD_MS); continue; }
    int32_t maxAmp = 0;
    for (uint32_t i = 0; i < recCount_; i += 8) {
      int32_t a = recBuf_[i]; if (a < 0) a = -a;
      if (a > maxAmp) maxAmp = a;
    }
    if (maxAmp < 100) { setStatus("訊號太弱,再試一次"); vTaskDelay(800 / portTICK_PERIOD_MS); continue; }

    // --- STT + 解析 ---
    setStatus("辨識中...");
    String text = transcribe();
    if (text.length() == 0) { vTaskDelay(800 / portTICK_PERIOD_MS); continue; }
    setStatus("解析中...");
    String json = parseLlama(text);
    if (json.length() == 0) { vTaskDelay(800 / portTICK_PERIOD_MS); continue; }
    emitAction(json, text);
  }
}

// ============================================================
// 主迴圈端 UI
// ============================================================
const char* VoiceAssistant::progName() {
  switch (prog_) {
    case VP_TAKEOFF: return "起飛中";
    case VP_HOVER:   return "懸停保持";
    case VP_MOVE:    return "移動中";
    case VP_LAND:    return "降落中";
    default:         return "搖桿控制";
  }
}

void VoiceAssistant::drawVoiceStatic() {
  tft_.fillScreen(TFT_BLACK);
  tft_.fillRect(0, 0, 240, 32, TFT_MAROON);
  tft_.setFont(&fonts::efontTW_24);
  tft_.setTextColor(TFT_WHITE, TFT_MAROON);
  tft_.setCursor(14, 4);
  tft_.print("Mode 10 語音控制");
  tft_.setFont(&fonts::efontTW_14);
  tft_.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft_.setCursor(5, 296);
  tft_.print("按住左肩鈕說話,搖桿一動即手動");
}

void VoiceAssistant::updateUi(const Telemetry& tele, bool teleOK, uint8_t throttle) {
  static char lastStatus[48] = "";
  static VProg lastProg = VP_LAND;   // 故意不同,首次必畫
  static unsigned long lastTele = 0;

  if (strcmp(lastStatus, status_) != 0) {
    strncpy(lastStatus, status_, sizeof(lastStatus));
    tft_.fillRect(0, 50, 240, 60, TFT_BLACK);
    tft_.setFont(&fonts::efontTW_24);
    tft_.setTextColor(TFT_CYAN, TFT_BLACK);
    tft_.setCursor(8, 60);
    tft_.print(lastStatus);
  }
  if (prog_ != lastProg) {
    lastProg = prog_;
    tft_.fillRect(0, 130, 240, 40, TFT_BLACK);
    tft_.setFont(&fonts::efontTW_24);
    tft_.setTextColor(prog_ == VP_NONE ? TFT_WHITE : TFT_GREEN, TFT_BLACK);
    tft_.setCursor(8, 138);
    tft_.printf("%s  油門 %d", progName(), throttle);
  }
  if (millis() - lastTele > 500) {   // 遙測 2Hz 更新
    lastTele = millis();
    tft_.fillRect(0, 190, 240, 30, TFT_BLACK);
    tft_.setFont(&fonts::efontTW_16);
    tft_.setTextColor(teleOK ? TFT_LIGHTGREY : TFT_RED, TFT_BLACK);
    tft_.setCursor(8, 196);
    if (teleOK)
      tft_.printf("R%+.1f P%+.1f 高 %.1fm", tele.roll / 10.0f, tele.pitch / 10.0f, tele.altitude_dm / 10.0f);
    else
      tft_.print("飛機未連線");
  }
}
#endif  // ENABLE_VOICE_MODE10
