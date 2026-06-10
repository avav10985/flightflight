// ============================================================
// HeadTrack_Test — 手機 IMU 透過 WiFi 控制飛機端 servo(頭追原型)
// 燒到飛機 LOLIN D32(ESP32-WROOM-32)。**獨立 sketch,不影響 Drone_FC_Full**。
//
// 用法:
//   1. 燒這支到 LOLIN D32(arduino-cli FQBN esp32:esp32:lolin_d32)
//   2. ESP32 開機開啟 WiFi AP「HeadTrack」(密碼 "head1234")
//   3. 手機連 HeadTrack WiFi
//   4. 瀏覽器開 http://192.168.4.1
//   5. 網頁要求權限讀 IMU,允許
//   6. 動手機 → Serial 印 alpha/beta/gamma + servo 寫入值
//   7. 兩顆 SG90 servo 接 GPIO 16/17,5V/GND 從 buck → 看是不是跟著動
//
// 為什麼可以直接讀手機 IMU:
//   - 現代瀏覽器(Chrome/Safari)的 DeviceOrientation API
//   - 不用裝任何 App,純網頁 + JavaScript
//   - HTTPS 限制:iOS 14+ 需要 HTTPS 才能授權,Android 一般 OK
//     如果 iOS 不行,要外接 HTTPS proxy 或用其他方法
//
// 注意:
//   - WiFi 跟 NRF24 都 2.4GHz → 同時開會互擾
//   - 這支 sketch 沒 NRF24,純頭追測試
//   - 整合到 Drone_FC_Full 之後要處理 WiFi + NRF24 共存(可以,但要小心)
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ---- WiFi AP 設定(公開,非敏感)----
const char* AP_SSID = "HeadTrack";
const char* AP_PASS = "head1234";

// ---- Servo 腳位 ----
#define PIN_PAN_SERVO   16    // 左右(yaw)
#define PIN_TILT_SERVO  17    // 上下(pitch)

WebServer server(80);
Servo panServo, tiltServo;

// 從手機收到的角度
float lastAlpha = 0;   // 0~360,繞 Z 軸(yaw 方向,指南針)
float lastBeta  = 0;   // -180~180,繞 X 軸(pitch,前後傾)
float lastGamma = 0;   // -90~90,繞 Y 軸(roll,左右傾)

// 上次收到資料的時間(超過 1 秒沒資料 → 失聯,servo 回中位)
unsigned long lastDataMs = 0;

// 寫入 servo,把手機角度映射成 servo 角度
void writeServos() {
  // alpha 0~360 → pan servo 0~180(取一半範圍 ±90°,避免 servo 卡死)
  // 把 alpha 看成 -180~+180:把 alpha 大於 180 的減 360
  float yaw = lastAlpha;
  if (yaw > 180) yaw -= 360;
  // yaw 範圍 -180~+180,clamp 到 ±60 後映射到 servo 30~150
  yaw = constrain(yaw, -60.0f, 60.0f);
  int pan = map((int)yaw, -60, 60, 30, 150);

  // beta -90~+90 → tilt servo 30~150
  float pitch = constrain(lastBeta, -60.0f, 60.0f);
  int tilt = map((int)pitch, -60, 60, 30, 150);

  panServo.write(pan);
  tiltServo.write(tilt);
}

// HTTP 根路徑:回傳 HTML 頁面(手機開這個)
void handleRoot() {
  String html =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>HeadTrack</title>"
    "<style>"
    "body{font-family:sans-serif;margin:15px;background:#111;color:#0f0;}"
    "button{font-size:20px;padding:10px 20px;margin:8px 0;background:#0a0;color:#fff;border:0;border-radius:4px;}"
    "h1{font-size:18px;}"
    "p{font-size:14px;margin:4px 0;}"
    ".v{font-size:24px;font-family:monospace;color:#ff0;}"
    "#log{font-size:11px;color:#aaa;margin-top:10px;}"
    "</style></head><body>"
    "<h1>HeadTrack(手機 → 飛機 servo)</h1>"
    "<p id='status'>偵測中...</p>"
    "<hr><h2>方案 A:滑桿手動控制(不靠 sensor)</h2>"
    "<p>Pan(左右):<span class='v' id='ps'>0</span></p>"
    "<input type='range' min='-60' max='60' value='0' id='panS' "
    "oninput='ps.innerText=this.value;sendSlider()' style='width:100%;height:40px;'>"
    "<p>Tilt(上下):<span class='v' id='ts'>0</span></p>"
    "<input type='range' min='-60' max='60' value='0' id='tiltS' "
    "oninput='ts.innerText=this.value;sendSlider()' style='width:100%;height:40px;'>"
    "<hr><h2>方案 B:手機 IMU(自動)</h2>"
    "<button onclick='reqIOS()' id='btn'>iOS 點此授權</button>"
    "<p>α(yaw):<span class='v' id='a'>--</span></p>"
    "<p>β(pitch):<span class='v' id='b'>--</span></p>"
    "<p>γ(roll):<span class='v' id='g'>--</span></p>"
    "<p id='log'>等待事件...</p>"
    "<script>"
    "var lastSend = 0, count = 0, lastSliderSend = 0;"
    "function setStatus(s) { document.getElementById('status').innerText = s; }"
    "function setLog(s) { document.getElementById('log').innerText = s; }"
    "function sendSlider() {"
    "  var now = Date.now();"
    "  if (now - lastSliderSend < 30) return;"   // 限 30Hz 避免送太快
    "  lastSliderSend = now;"
    "  var pan = document.getElementById('panS').value;"
    "  var tilt = document.getElementById('tiltS').value;"
    "  fetch('/o?a=' + pan + '&b=' + tilt + '&g=0').catch(function(){});"
    "}"
    "function onOrient(e) {"
    "  count++;"
    "  document.getElementById('a').innerText = (e.alpha==null?0:e.alpha).toFixed(1);"
    "  document.getElementById('b').innerText = (e.beta==null?0:e.beta).toFixed(1);"
    "  document.getElementById('g').innerText = (e.gamma==null?0:e.gamma).toFixed(1);"
    "  setLog('收到事件: ' + count + ' 次');"
    "  var now = Date.now();"
    "  if (now - lastSend > 50) {"
    "    lastSend = now;"
    "    var a = e.alpha==null?0:e.alpha, b = e.beta==null?0:e.beta, g = e.gamma==null?0:e.gamma;"
    "    fetch('/o?a=' + a + '&b=' + b + '&g=' + g).catch(function(){});"
    "  }"
    "}"
    "function attach() {"
    "  window.addEventListener('deviceorientation', onOrient);"
    "  setStatus('已 attach,等事件...');"
    "}"
    "function reqIOS() {"
    "  setStatus('呼叫 iOS requestPermission...');"
    "  try {"
    "    DeviceOrientationEvent.requestPermission().then(function(p) {"
    "      if (p === 'granted') { attach(); document.getElementById('btn').style.display='none'; }"
    "      else { setStatus('iOS 拒絕: ' + p); }"
    "    }).catch(function(err) { setStatus('iOS 錯誤: ' + err.message); });"
    "  } catch(e) { setStatus('reqIOS 異常: ' + e.message); }"
    "}"
    // 立刻執行,不用 window.onload
    "var hasDOE = typeof DeviceOrientationEvent !== 'undefined';"
    "var hasReq = hasDOE && typeof DeviceOrientationEvent.requestPermission === 'function';"
    "setStatus('偵測:DOE=' + hasDOE + ' reqPerm=' + hasReq);"
    "if (hasReq) {"
    // iOS:等使用者按按鈕
    "  setStatus('iOS 偵測,點下方按鈕授權');"
    "} else if (hasDOE) {"
    // Android / 桌面:直接 attach
    "  attach();"
    "  document.getElementById('btn').style.display = 'none';"
    "} else {"
    "  setStatus('瀏覽器不支援 DeviceOrientationEvent');"
    "}"
    // 5 秒後如果還沒收到事件,提示可能 HTTP/HTTPS 問題
    "setTimeout(function() {"
    "  if (count === 0) {"
    "    setLog('5 秒沒收到事件 → 多半 Chrome Android 在 HTTP 環境封鎖了 sensor。'"
    "          + ' 試 Firefox 或 DuckDuckGo 瀏覽器,或 chrome://flags 開 #unsafely-treat-insecure-origin-as-secure');"
    "  }"
    "}, 5000);"
    "</script></body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

// HTTP /o?a=X&b=Y&g=Z:接收手機 IMU 資料
void handleOrient() {
  if (server.hasArg("a")) lastAlpha = server.arg("a").toFloat();
  if (server.hasArg("b")) lastBeta  = server.arg("b").toFloat();
  if (server.hasArg("g")) lastGamma = server.arg("g").toFloat();
  lastDataMs = millis();
  writeServos();
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== HeadTrack 原型啟動 ===");

  // Servo 初始化,中位
  panServo.attach(PIN_PAN_SERVO);
  tiltServo.attach(PIN_TILT_SERVO);
  panServo.write(90);
  tiltServo.write(90);
  Serial.printf("[+] Servo:Pan=GPIO%d Tilt=GPIO%d,先到中位 90°\n",
                PIN_PAN_SERVO, PIN_TILT_SERVO);

  // WiFi AP 模式
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);
  Serial.printf("[+] WiFi AP「%s」啟動,IP = %s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());
  Serial.printf("    密碼:%s\n", AP_PASS);
  Serial.println("    手機連 HeadTrack → 瀏覽器開 http://192.168.4.1");

  // HTTP 路由
  server.on("/", handleRoot);
  server.on("/o", handleOrient);
  server.begin();
  Serial.println("[+] HTTP server 啟動");
}

void loop() {
  server.handleClient();

  // 失聯保護:1 秒沒資料 → servo 回中位
  if (millis() - lastDataMs > 1000) {
    panServo.write(90);
    tiltServo.write(90);
  }

  // 每秒 Serial 印一次當前角度
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.printf("[head] α=%.1f β=%.1f γ=%.1f | pan=%d tilt=%d\n",
                  lastAlpha, lastBeta, lastGamma,
                  panServo.read(), tiltServo.read());
  }
}
