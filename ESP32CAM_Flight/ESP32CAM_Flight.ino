// ============================================================
// ESP32-CAM 飛行版（連到固定 WiFi）
// MCU: AI-Thinker ESP32-CAM
//
// 跟測試版的差別：
//   - Station 模式：飛機連到「電腦/手機建立的熱點」
//   - 失敗自動退回 AP 模式（保命，照樣可用）
//   - QVGA 320×240 預設（穩定流暢）
//   - LED 狀態指示：閃 = 連線中、亮 = 已連線
//
// 使用流程：
//   1. 電腦或手機建立 WiFi 熱點（SSID + 密碼）
//   2. 修改下面 WIFI_SSID 和 WIFI_PASS
//   3. 燒進 ESP32-CAM
//   4. ESP32-CAM 連到熱點 → 序列埠顯示分配到的 IP
//   5. 同熱點下的電腦瀏覽器開 http://<IP>
//
// 飛機端供電：
//   - 從 ESC1 BEC 或 PDB 取 5V → ESP32-CAM 5V
//   - 加 470µF 電解電容（5V ↔ GND）吸 ESC 雜訊
// ============================================================

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// ========== 你要改的兩行 ==========
const char* WIFI_SSID = "你的熱點名稱";
const char* WIFI_PASS = "你的熱點密碼";
// 例如：
// const char* WIFI_SSID = "MyPhone";
// const char* WIFI_PASS = "12345678";
// ===================================

// ---- AP 備援設定（連不上熱點時使用）----
const char* AP_SSID_FALLBACK = "DroneCAM_AP";
const char* AP_PASS_FALLBACK = "12345678";

// ---- AI-Thinker ESP32-CAM 相機腳位 ----
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define LED_PIN  33   // 板載紅 LED（active LOW）

WebServer server(80);
bool cameraOK = false;
bool wifiSTA  = false;   // true=連到熱點、false=AP 模式

// ============================================================
// 相機初始化（飛行用 QVGA 平衡畫質與流暢度）
// ============================================================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;    // 320×240 飛行用
  config.jpeg_quality = 12;                 // 12 平衡點
  config.fb_count     = psramFound() ? 2 : 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[X] 相機初始化失敗 0x%x\n", err);
    return false;
  }
  Serial.println("[+] 相機 QVGA 模式 OK");
  return true;
}

// ============================================================
// WiFi 連線（先 Station，失敗退回 AP）
// ============================================================
void connectWiFi() {
  Serial.printf("[*] 嘗試連到熱點 \"%s\"...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));   // LED 閃
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiSTA = true;
    digitalWrite(LED_PIN, LOW);   // 持續亮（active low）
    Serial.printf("[+] 已連線！IP = %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[+] 訊號強度 RSSI = %d dBm\n", WiFi.RSSI());
  } else {
    Serial.println("[!] 連不到熱點，退回 AP 模式");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID_FALLBACK, AP_PASS_FALLBACK);
    delay(500);
    digitalWrite(LED_PIN, HIGH);   // LED 滅（AP 模式）
    Serial.printf("[+] AP \"%s\" 啟動，IP = %s\n",
                  AP_SSID_FALLBACK,
                  WiFi.softAPIP().toString().c_str());
    Serial.printf("    密碼: %s\n", AP_PASS_FALLBACK);
  }
}

// ============================================================
// HTTP 路由
// ============================================================
void handleRoot() {
  String html =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<title>DroneCAM</title>"
    "<style>"
    "body{margin:0;background:#000;color:#fff;font-family:sans-serif;text-align:center;}"
    "img{max-width:100%;height:auto;}"
    "</style></head><body>"
    "<img src='/stream' />"
    "</body></html>";
  server.send(200, "text/html", html);
}

void handleStream() {
  WiFiClient client = server.client();
  String boundary = "frame";
  client.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=" + boundary + "\r\n\r\n"
  );

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      delay(50);
      continue;
    }
    client.printf("--%s\r\n", boundary.c_str());
    client.print("Content-Type: image/jpeg\r\n");
    client.printf("Content-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    esp_camera_fb_return(fb);
    delay(20);   // 約 50fps 上限
  }
}

void handleSnapshot() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");
  server.client().write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("  ESP32-CAM 飛行版");
  Serial.println("========================================");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);   // 一開始 LED 滅

  if (psramFound()) {
    Serial.printf("[+] PSRAM: %d bytes\n", ESP.getPsramSize());
  }

  // 相機
  cameraOK = initCamera();
  if (!cameraOK) {
    Serial.println("⚠️  相機 init 失敗，停止");
    while (1) delay(1000);
  }

  // WiFi
  connectWiFi();

  // HTTP server
  server.on("/",         HTTP_GET, handleRoot);
  server.on("/stream",   HTTP_GET, handleStream);
  server.on("/snapshot", HTTP_GET, handleSnapshot);
  server.begin();
  Serial.println("[+] HTTP server OK");

  Serial.println();
  Serial.println("========================================");
  Serial.println("  路徑：");
  Serial.println("    /         首頁（即時影像）");
  Serial.println("    /stream   純 MJPEG 串流（給 OpenCV / VLC）");
  Serial.println("    /snapshot 單張 JPEG（給 AI 抓幀）");
  Serial.println("========================================");
}

void loop() {
  server.handleClient();

  // 監控連線狀態
  static unsigned long lastT = 0;
  static unsigned long lastReconnect = 0;
  if (millis() - lastT > 5000) {
    lastT = millis();
    if (wifiSTA) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[!] WiFi 斷線，重連...");
        WiFi.reconnect();
        lastReconnect = millis();
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);   // 連線中持續亮
        Serial.printf("[狀態] RSSI=%d dBm  Heap=%d\n",
                      WiFi.RSSI(), ESP.getFreeHeap());
      }
    }
  }
}
