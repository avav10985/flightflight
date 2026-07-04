// ============================================================
// ESP32-CAM 獨立測試程式
// MCU: AI-Thinker ESP32-CAM（內建 OV2640 相機）
//
// 用途：驗證 ESP32-CAM、相機、WiFi 串流都正常
//
// ⚠️ 注意：這個程式是燒給 ESP32-CAM，**不是**燒給 LOLIN D32！
//          ESP32-CAM 跟飛機端的飛控板是兩塊不同的板子。
//
// 燒錄方式：
//   1. 用 FTDI USB 轉 TTL 接 ESP32-CAM：
//      FTDI 5V  → 5V
//      FTDI GND → GND
//      FTDI TX  → U0R
//      FTDI RX  → U0T
//   2. 燒錄前：IO0 短接 GND（進入燒錄模式）
//   3. 按一下 RESET 按鈕
//   4. Arduino IDE 上傳
//   5. 燒完：IO0 拔離 GND，按 RESET → 正常運行
//
// 開發板選擇：工具 → 開發板 → "AI Thinker ESP32-CAM"
//
// 使用方式：
//   1. 燒完之後序列埠 (115200) 顯示 WiFi 名稱 + IP
//   2. 手機 / 電腦連上 WiFi「DroneCAM_test」（密碼 12345678）
//   3. 瀏覽器開 http://192.168.4.1
//   4. 看到即時影像 → ✅ 一切正常
// ============================================================

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// ==== AI-Thinker ESP32-CAM 相機腳位（固定，別改） ====
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

// ==== WiFi AP 設定 ====
const char* AP_SSID = "DroneCAM_test";
const char* AP_PASS = "12345678";

WebServer server(80);

bool cameraOK = false;

// ============================================================
// 相機初始化
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

  // 有 PSRAM 用高畫質、沒有用低畫質
  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA;   // 640×480
    config.jpeg_quality = 10;              // 0~63，越小越好
    config.fb_count     = 2;
    Serial.println("[+] PSRAM 偵測到 → 高畫質模式");
  } else {
    config.frame_size   = FRAMESIZE_QVGA;  // 320×240
    config.jpeg_quality = 12;
    config.fb_count     = 1;
    Serial.println("[!] 無 PSRAM → 低畫質模式");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[X] 相機初始化失敗，錯誤碼 0x%x\n", err);
    return false;
  }
  Serial.println("[+] 相機初始化 OK");
  return true;
}

// ============================================================
// HTTP 路由
// ============================================================
void handleRoot() {
  String html =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<title>ESP32-CAM 測試</title>"
    "<style>"
    "body{margin:0;background:#222;color:#fff;font-family:sans-serif;text-align:center;}"
    "h2{margin:10px;}"
    "img{max-width:100%;height:auto;border:2px solid #4af;}"
    "</style></head><body>"
    "<h2>ESP32-CAM 即時影像測試</h2>"
    "<img src='/stream' />"
    "<p>看到畫面 = 相機 + WiFi 都正常 ✓</p>"
    "</body></html>";
  server.send(200, "text/html", html);
}

void handleStream() {
  WiFiClient client = server.client();
  String boundary = "frame";
  String headers =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=" + boundary + "\r\n"
    "\r\n";
  client.print(headers);

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[!] 取影像失敗");
      delay(100);
      continue;
    }

    client.printf("--%s\r\n", boundary.c_str());
    client.print("Content-Type: image/jpeg\r\n");
    client.printf("Content-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    esp_camera_fb_return(fb);

    if (!client.connected()) break;
    delay(30);  // 約 30fps 上限
  }
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("  ESP32-CAM 測試程式");
  Serial.println("========================================");

  // ----- PSRAM 檢查 -----
  if (psramFound()) {
    Serial.printf("[+] PSRAM size: %d bytes\n", ESP.getPsramSize());
  } else {
    Serial.println("[!] 沒偵測到 PSRAM（可能是無 PSRAM 版本或設定問題）");
  }

  // ----- 相機初始化 -----
  cameraOK = initCamera();
  if (!cameraOK) {
    Serial.println();
    Serial.println("⚠️  相機 init 失敗可能原因：");
    Serial.println("    1. 開發板選錯（應該選 AI Thinker ESP32-CAM）");
    Serial.println("    2. 相機鏡頭沒插好 / 排線斷");
    Serial.println("    3. 燒錄時供電不足（FTDI 5V 電流不夠 → 用獨立 5V）");
    Serial.println("    4. 鏡頭模組壞");
    while (1) delay(1000);
  }

  // ----- WiFi AP -----
  Serial.println("[+] 啟動 WiFi AP...");
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(500);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("[+] WiFi 名稱: %s\n", AP_SSID);
  Serial.printf("[+] 密碼:      %s\n", AP_PASS);
  Serial.printf("[+] IP 位址:   %s\n", ip.toString().c_str());

  // ----- HTTP server -----
  server.on("/",       HTTP_GET, handleRoot);
  server.on("/stream", HTTP_GET, handleStream);
  server.begin();
  Serial.println("[+] HTTP server started");

  Serial.println();
  Serial.println("========================================");
  Serial.println("  使用方式：");
  Serial.printf("  1. 手機/電腦 連上 WiFi: %s\n", AP_SSID);
  Serial.printf("  2. 密碼: %s\n", AP_PASS);
  Serial.printf("  3. 瀏覽器開: http://%s\n", ip.toString().c_str());
  Serial.println("========================================");
}

void loop() {
  server.handleClient();

  // 每 5 秒印一次狀態
  static unsigned long lastT = 0;
  if (millis() - lastT > 5000) {
    lastT = millis();
    int n = WiFi.softAPgetStationNum();
    Serial.printf("[狀態] 連線數=%d 可用記憶體=%d bytes\n",
                  n, ESP.getFreeHeap());
  }
}
