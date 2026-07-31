// ============================================================
// 地面站發射器 V2-A(OOP 重構版):ESP32-S3-N16R8 + MSP2806 2.8" SPI TFT(直立)
//
// 2026-07-04 從單檔 Ground_TX_ESP32.ino 重構成類別模組,功能/行為不變:
//   packets.h        nRF24 封包契約(平台無關,新手把 Pi 直接沿用)
//   FlightLink       nRF24 鏈路(平台無關,RF24 函式庫 Pi 同 API)
//   Inputs           搖桿/三段開關/選單鈕/肩鍵
//   MainUi           飛行畫面 + PID 選單 + 校準/存檔 modal
//   MediaPlayer      I²S + 音樂 + Mode 12 影片
//   VoiceAssistant   Mode 10 語音(核心 0 task + 虛擬搖桿)
//   WifiScanScreen   Mode 22 WiFi 環境掃描
//   PcBridge.h       Mode 11 骨架(未接線)、PersistentMenu.h 常駐選單骨架
//
// 功能:
//   - 油門(左搖桿上下,拆彈簧)/ pitch / roll
//   - yaw 由肩鍵 L / R(按著 = 固定 ±60°/s)
//   - 兩個 3 段開關組 9 模式,**mode = A×10 + B 兩位數編碼**:
//     00=安全/校準、01=手動、02=GPS、10=語音、11=PC、12/20/21/22=預留
//   - NRF24 雙向 ACK(送指令 + 收遙測)
//   - MSP2806 2.8" SPI ILI9341 TFT 240×320 直立顯示模式/姿態/狀態
//     (跟 NRF24/SD 共用 SPI 匯流排,只佔 CS=48 + DC=21 兩隻獨立腳)
//   - mode 0 按「OK」進選單,4 鈕電阻階梯調 PID
//
// 硬體接線見 手把接線總表_V2.md。
// 開發板:ESP32S3 Dev Module;USB CDC On Boot=Enabled;
//          Flash=16MB;PSRAM=OPI PSRAM;Partition=16M with OTA。
// 函式庫:LovyanGFX(per-sketch config,LGFX_S3.h)、RF24、SD、SPI、JPEGDEC。
//
// 飛機端 Drone_FC_Full 用同一份封包(packets.h 內容須逐欄一致),
// 兩端必須同時重燒,封包對不上會收到亂碼。
// ============================================================

#include <SPI.h>
#include <SD.h>
#include "config.h"
#include "packets.h"
#include "LGFX_S3.h"
#include "FlightLink.h"
#include "Inputs.h"
#include "MainUi.h"
#if ENABLE_MUSIC
#include "MediaPlayer.h"
#endif
#if ENABLE_VOICE_MODE10
#include "VoiceAssistant.h"
#endif
#if ENABLE_WIFI_SCAN_MODE22
#include "WifiScan.h"
#endif
#if ENABLE_PC_MODE11
#include "PcBridge.h"
#endif

// ---- 物件 ----
LGFX       tft;
FlightLink flightLink(PIN_NRF_CE, PIN_NRF_CSN, PIPE_OUT, NRF_CHANNEL);
Inputs     inputs;
MainUi     mainUi(tft, flightLink);
#if ENABLE_MUSIC
MediaPlayer media(tft);
#endif
#if ENABLE_VOICE_MODE10
VoiceAssistant voice(tft);
#endif
#if ENABLE_WIFI_SCAN_MODE22
WifiScanScreen scanScreen(tft);
#endif

Signal data;
bool   sdOK = false;
#if ENABLE_SD
SPIClass spiSD(HSPI);   // SD 專用第二組 SPI(S3 上 HSPI = SPI3)
#endif

unsigned long lastDrawTime = 0;

// ============================================================
void setup() {
  Serial.begin(115200);
  // 等 USB CDC 真的連上(最多 3 秒),沒 host 連接也照樣繼續
  // 比固定 delay(2000) 可靠:有 Serial Monitor 接著就立刻過,沒接就 3 秒後超時
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(200);   // 額外緩衝,讓 host 端 buffer 準備好
  Serial.println("\n=== 地面站 V2-A 啟動(OOP)===");
  Serial.flush();

  // 關掉 ESP32-S3 板上的 WS2812 RGB LED(常在 GPIO 48,跟我們 TFT CS 共腳)
  // 在 tft.init() 之前送一次「全 0 = 關燈」訊號,之後 GPIO 48 切換成 SPI CS
  // 用,LED 會保持上次的狀態(熄滅)。如果你的板 LED 不在 GPIO 48,改下面數字。
  neopixelWrite(48, 0, 0, 0);

  inputs.begin();   // 肩鍵 INPUT_PULLUP + ADC 12bit

  // MAX98357A SD 拉低 → 休眠,沒「沙沙」雜訊。Mode 10 / 音樂播放時程式再設 HIGH
  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, LOW);

  // TFT 背光:MSP2806 的 LED 腳直接接 B 板 3V3 軌常亮,不需要程式控制
  // (試燒驗證:LED 接 GPIO 15 PWM 不會亮,直接拉 3V3 才亮)

  // === SPI / NRF24 / SD 先 init,TFT 後加入共用 ===
  // 順序顛倒(TFT 先)會讓 SD 拿不到 MISO 掛載失敗(2026-06-06 試燒實證)
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);

  // NRF24
  Serial.print("[*] NRF24 radio.begin() ... ");
  bool nrfOK = flightLink.begin();
  Serial.println(nrfOK ? "OK" : "FAIL");
  Serial.printf("[*] NRF24 chip connected: %s\n",
                flightLink.chipConnected() ? "YES" : "NO ⚠️ 模組沒回應 SPI");
  if (!nrfOK || !flightLink.chipConnected()) {
    Serial.println("[!] 手把 NRF24 init 失敗,通常是:");
    Serial.println("    1. 100µF 電容沒焊");
    Serial.println("    2. CE/CSN/SCK/MISO/MOSI 接線錯");
    Serial.println("    3. 模組壞了");
    Serial.println("    4. 3V3 軌電壓不夠");
  }
  Serial.println("[+] NRF24 就緒");

#if ENABLE_SD
  // SD(選用,沒卡也繼續)— 用獨立 SPI3,不碰 NRF24/TFT 的 bus
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  spiSD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  // 20MHz 餵影片才夠(4MHz 只有 ~400KB/s 會卡),掛不上自動降速
  if (SD.begin(PIN_SD_CS, spiSD, 20000000) ||
      SD.begin(PIN_SD_CS, spiSD, 10000000) ||
      SD.begin(PIN_SD_CS, spiSD, 4000000)) {
    sdOK = true;
    Serial.println("[+] SD 卡掛載成功(獨立 SPI3)");
  } else {
    Serial.println("[*] SD 卡未接或初始化失敗(繼續)");
  }
#else
  // ENABLE_SD = 0:跳過 SD init,GPIO 47 不操作,留給未來其他用途
  Serial.println("[*] SD 停用(ENABLE_SD = 0)");
  sdOK = false;
#endif

  // MAX98357A SD 腳:開機立刻拉低靜音(浮空會放大 I²S 雜訊,memory 老坑)
  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, LOW);
#if ENABLE_MUSIC
  media.setSdOk(sdOK);
  media.beginI2S();
  Serial.println("[+] 音樂 I²S 就緒(選單→音樂 播放/停止)");
#endif

#if ENABLE_VOICE_MODE10
  // 錄音緩衝(PSRAM)+ 語音管線 task 釘在核心 0(主迴圈在核心 1)
  if (voice.begin(media.micHandle())) {
    Serial.println("[+] 語音 task 啟動(核心 0)");
  } else {
    Serial.println("[!] PSRAM 配置失敗,Mode 10 停用");
  }
#endif

  // TFT(SD 之後 init,共用 SPI bus_shared 模式)
  tft.init();
  tft.setRotation(2);   // 240×320 portrait 翻 180°
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::efontTW_16);

  // 開機狀態畫面(因 USB CDC 在 ESP32-S3 有問題,debug 訊息全部上 TFT)
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(5, 5);
  tft.print("地面站 V2-A 啟動");

  // NRF24 狀態:大字顯示 OK / FAIL,3 秒後才進飛行 UI
  tft.setCursor(5, 40);
  tft.setTextColor(nrfOK ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.print("NRF24 begin: ");
  tft.print(nrfOK ? "OK" : "FAIL");

  tft.setCursor(5, 70);
  bool chipOK = flightLink.chipConnected();
  tft.setTextColor(chipOK ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.print("chip connected: ");
  tft.print(chipOK ? "YES" : "NO");

  tft.setCursor(5, 100);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.printf("PIPE = 0x%04X%04X", (uint16_t)(PIPE_OUT >> 16), (uint16_t)PIPE_OUT);
  tft.setCursor(5, 130);
  tft.printf("CHAN = %d", NRF_CHANNEL);

  // 上次重啟原因:除錯語音辨識中途重啟用(電池模式看不到 Serial)
  {
    esp_reset_reason_t rr = esp_reset_reason();
    const char* rs = (rr == ESP_RST_POWERON) ? "正常上電" :
                     (rr == ESP_RST_BROWNOUT) ? "BROWNOUT 電壓不足!" :
                     (rr == ESP_RST_PANIC)    ? "PANIC 程式崩潰!" :
                     (rr == ESP_RST_TASK_WDT) ? "看門狗 WDT!" :
                     (rr == ESP_RST_INT_WDT)  ? "中斷 WDT!" :
                     (rr == ESP_RST_SW)       ? "軟體重啟" : "其他";
    tft.setCursor(5, 200);
    tft.setTextColor(rr == ESP_RST_POWERON ? TFT_DARKGREY : TFT_ORANGE, TFT_BLACK);
    tft.printf("上次重啟:%s", rs);
  }

  tft.setCursor(5, 170);
  if (sdOK) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print("SD: 已掛載");
  } else {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.print("SD: 未接");
  }

  tft.setCursor(5, 250);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.print("3 秒後進飛行 UI...");

  delay(3000);

  mainUi.showFlight();
  Serial.println("地面站 V2-A 就緒");
}

// ============================================================
void loop() {
  uint8_t mode = inputs.readModeSwitches();   // 兩位數編碼:A 十位 + B 個位
  int btnE = inputs.menuBtnEdge();            // 按鈕單一邊緣偵測

#if ENABLE_VOICE_MODE10
  // Mode 10 語音控制:50Hz 控制照跑(這是飛行模式!),語音管線在
  // 核心 0 背景跑,主迴圈只做 PTT 偵測 + 虛擬搖桿覆寫 + UI
  static bool inVoiceMode = false;
  if (mode == 10) {
    if (!inVoiceMode) {
      inVoiceMode = true;
#if ENABLE_MUSIC
      media.musicStop();   // 錄音時不放音樂(回授 + I²S 雙工單純化)
#endif
      voice.enter();
    }
    voice.setPTT(inputs.shoulderL());          // 左肩鈕 = PTT
    inputs.fillSignal(data, mode);             // data.mode = 10,yaw 中立
    voice.applyOverride(data);                 // 語音程式覆寫虛擬搖桿
    flightLink.send(data);
    voice.updateUi(flightLink.telemetry(), flightLink.teleOK(), data.throttle);
#if ENABLE_MUSIC
    // 語音回應:task 設 speak id,主迴圈用音樂管線非阻塞播 SD 上的預錄句
    uint8_t spk = voice.takeSpeech();
    if (spk) {
      static const char* SPK[] = { "", "/voice/takeoff.wav", "/voice/land.wav",
        "/voice/stop.wav", "/voice/motor1.wav", "/voice/motor2.wav",
        "/voice/motor3.wav", "/voice/motor4.wav", "/voice/unknown.wav" };
      if (spk <= 8 && sdOK) media.musicPlay(SPK[spk]);
    }
    media.musicUpdate();
#endif
    return;
  } else if (inVoiceMode) {
    inVoiceMode = false;
    voice.exit();
    mainUi.showFlight();
  }
#endif

#if ENABLE_VIDEO_MODE12
  // Mode 12 媒體模式:接管整個 loop。NRF24 照送(mode=12 → 飛機白名單
  // 擋掉自動 disarm),模式開關一撥走立刻退出
  static bool inVideoMode = false;
  if (mode == 12) {
    if (!inVideoMode) { inVideoMode = true; media.enterVideoMode(); }
    media.videoModeLoop(btnE, inputs);
    inputs.fillSignal(data, mode);
    flightLink.send(data);
    return;
  } else if (inVideoMode) {
    inVideoMode = false;
    media.exitVideoMode();
    mainUi.showFlight();
  }
#endif

#if ENABLE_WIFI_SCAN_MODE22
  // Mode 22 WiFi 掃描:被動列出附近 AP。接管整個 loop,不送 NRF
  //(飛機 1 秒沒收到會自動 failsafe disarm,安全)
  static bool inScanMode = false;
  if (mode == 22) {
    if (!inScanMode) {
      inScanMode = true;
#if ENABLE_MUSIC
      media.musicStop();
#endif
      scanScreen.enter();
    }
    scanScreen.loop(btnE);
    return;
  } else if (inScanMode) {
    inScanMode = false;
    scanScreen.exit();
    mainUi.showFlight();
  }
#endif

  // UI 狀態機(選單開關/游標/按鈕);回傳 200 = 本地音樂切換
  int localAct = mainUi.update(mode, btnE, inputs, data);
#if ENABLE_MUSIC
  if (localAct == 200) media.musicToggle();
#else
  (void)localAct;
#endif

  // 讀搖桿;剛切進 mode 0 → 顯示進場自動校準提示
  // (FC 進 mode 0 自動跑 quick+full,~1.8s 阻塞 + 餘裕)
  if (inputs.fillSignal(data, mode)) mainUi.showCalPrompt("進場自動校準", 2400);

#if ENABLE_MUSIC
  media.musicUpdate();   // 非阻塞餵 I²S,DMA 滿了立刻返回
#endif

  // 送指令 + 收遙測(ACK payload)
  // Debug:每秒印一次 TX 統計,看到底有沒有送出去 + 飛機有沒有 ACK 回來
  static uint32_t lastTxStat = 0;
  bool ok = flightLink.send(data);
  if (ok && flightLink.teleOK()) mainUi.syncPidOnce();   // 開機第一筆遙測 → 選單顯示飛機實際 PID
  if (millis() - lastTxStat > 1000) {
    lastTxStat = millis();
    Serial.printf("[NRF] TX ok=%lu fail=%lu (1 秒內) | mode=%02d throttle=%d\n",
                  flightLink.txOk, flightLink.txFail, data.mode, data.throttle);
    flightLink.txOk = 0; flightLink.txFail = 0;
  }
  mainUi.maintainParam(data);   // 過了重送窗口才清 paramID(期間每包連送,抗丟包)

  // TFT 更新（每 150ms）
  if (millis() - lastDrawTime > 150) {
    lastDrawTime = millis();
    mainUi.draw(mode, data);
  }

  delay(20);   // 約 50Hz
}
