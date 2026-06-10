// ============================================================
// Mode10_Test — Phase 1:PTT 錄音到 PSRAM
//
// 目標:按住左肩鈕錄音、放開印 buffer 資訊,確認音檔有抓到。
// 後續 Phase:WiFi → Groq Whisper → Llama → NRF24 override → VoiceRSS。
//
// === 操作 ===
//   左肩鈕(GPIO 8)按住    → 開始錄音
//   左肩鈕放開              → 停止錄音 + Serial 印 sample 數 / 時長 / 最大振幅
//
// === 接線 ===(沿用 Voice_Test V2-B 配線)
//   INMP441:
//     VIN  → 3V3、GND → GND(L/R 接地)
//     WS   → GPIO 12
//     SCK  → GPIO 11
//     SD   → GPIO 14
//
// === 錄音格式 ===
//   16 kHz / 單聲道 / int16 PCM(Whisper 標準)
//   PSRAM buffer 上限 10 秒 = 320 KB
//
// 需要 Arduino-ESP32 core v3.0+、PSRAM=opi
// ============================================================

#include <driver/i2s_std.h>

#define PIN_I2S_BCLK     11
#define PIN_I2S_WS       12
#define PIN_I2S_DIN      14
#define PIN_SHOULDER_L    8

#define SAMPLE_RATE     16000
#define MAX_REC_SECS       10
#define MAX_SAMPLES     (SAMPLE_RATE * MAX_REC_SECS)
#define I2S_CHUNK         512

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

void startRec() {
  recCount   = 0;
  recording  = true;
  recStartMs = millis();
  Serial.println("[REC] 開始");
}

// 每 loop 抓一塊 I2S 進 PSRAM
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
    int32_t v = raw[i] >> 16;   // 24-bit MSB-aligned → 取高 16 bit
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    recBuf[recCount++] = (int16_t)v;
  }
}

void stopRec() {
  recording = false;
  unsigned long dur = millis() - recStartMs;
  // 算 max 振幅 + RMS 看訊號強度
  int32_t maxAmp = 0;
  uint64_t sumSq = 0;
  for (uint32_t i = 0; i < recCount; i++) {
    int32_t v = recBuf[i];
    if (v < 0) v = -v;
    if (v > maxAmp) maxAmp = v;
    sumSq += (uint32_t)(recBuf[i] * recBuf[i]);
  }
  float rms = recCount ? sqrtf((float)(sumSq / recCount)) : 0;
  Serial.printf("[REC] 停 | samples=%u | 時長=%lu ms | max=%ld | rms=%.0f\n",
                recCount, dur, (long)maxAmp, rms);
  if (maxAmp < 100)       Serial.println("      ⚠ 訊號太弱,可能沒接好或太遠");
  else if (maxAmp > 30000) Serial.println("      ⚠ 訊號爆滿,小聲一點");
  else                     Serial.println("      ✓ 訊號 OK");
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(200);
  Serial.println("\n=== Mode10_Test Phase 1:PTT 錄音 ===");

  pinMode(PIN_SHOULDER_L, INPUT_PULLUP);

  // PSRAM buffer:10 秒 × 16kHz × 2 bytes = 320 KB
  recBuf = (int16_t*)heap_caps_malloc(MAX_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM);
  if (!recBuf) {
    Serial.println("[X] PSRAM alloc 失敗,檢查 sketch.yaml 是否開 PSRAM=opi");
    while (1) delay(1000);
  }
  Serial.printf("[+] PSRAM buffer 配置:%d 樣本 / %d KB\n",
                MAX_SAMPLES, (int)(MAX_SAMPLES * 2 / 1024));

  initI2S();
  Serial.println("[+] I2S 啟動,SR=16kHz mono int16");
  Serial.println("[+] 按住左肩鈕(GPIO 8)開始錄音、放開停止");
}

void loop() {
  // 按鈕 edge detection
  static bool lastBtn = false;
  bool btn = (digitalRead(PIN_SHOULDER_L) == LOW);
  if (btn && !lastBtn) startRec();
  if (!btn && lastBtn && recording) stopRec();
  lastBtn = btn;

  doRecChunk();
}
