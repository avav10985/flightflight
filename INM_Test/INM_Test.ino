// ============================================================
// INM_Test — 單獨測試 INMP441 麥克風是否還活著
//
// 跟 jmaker blog 同樣概念,把腳位改成我們手把用的:
//   SCK = GPIO 11 (blog 是 26)
//   WS  = GPIO 12 (blog 是 22)
//   SD  = GPIO 14 (blog 是 21)
//
// 只連 INMP441,其他一概不接,單獨測麥克風。
//
// === 接線(5 條線)===
//   INMP441 VDD → ESP32 3V3      ⚠️ 絕對不能 5V,會燒晶片
//   INMP441 GND → ESP32 GND
//   INMP441 L/R → GND(跟 GND 短接,選左聲道)
//   INMP441 SCK → ESP32 GPIO 11
//   INMP441 WS  → ESP32 GPIO 12
//   INMP441 SD  → ESP32 GPIO 14
//
// === 預期 ===
// Serial Monitor (115200) 印出聲音量(RMS + ASCII 條):
//   - 安靜 → RMS 接近 0,條短
//   - 對麥克風講話 / 吹氣 → RMS 上升,條變長
//
// 如果不管你做什麼 RMS 都是 0 → INMP441 死了
// 如果 RMS 數值亂跳沒規律 / 滿格不變 → INMP441 也壞了
// 如果一接電就 brown-out reset → INMP441 短路嚴重
// ============================================================

#include <driver/i2s_std.h>
#include <math.h>

#define PIN_SCK 11
#define PIN_WS 12
#define PIN_SD 14

#define SAMPLE_RATE 16000
#define BUF_SAMPLES 512

i2s_chan_handle_t rx_handle = NULL;

void initI2S() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, NULL, &rx_handle);  // 只要 RX

  i2s_std_config_t std_cfg = {};
  std_cfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  std_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
  std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  std_cfg.gpio_cfg.bclk = (gpio_num_t)PIN_SCK;
  std_cfg.gpio_cfg.ws = (gpio_num_t)PIN_WS;
  std_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
  std_cfg.gpio_cfg.din = (gpio_num_t)PIN_SD;
  std_cfg.gpio_cfg.invert_flags.mclk_inv = false;
  std_cfg.gpio_cfg.invert_flags.bclk_inv = false;
  std_cfg.gpio_cfg.invert_flags.ws_inv = false;

  i2s_channel_init_std_mode(rx_handle, &std_cfg);
  i2s_channel_enable(rx_handle);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("  INMP441 麥克風單獨測試");
  Serial.println("========================================");
  Serial.println("接線:");
  Serial.println("  VDD → 3V3   (不能 5V!)");
  Serial.println("  GND → GND");
  Serial.println("  L/R → GND   (選左聲道)");
  Serial.println("  SCK → GPIO 11");
  Serial.println("  WS  → GPIO 12");
  Serial.println("  SD  → GPIO 14");
  Serial.println("");

  // 關 RGB LED(避免干擾)
  neopixelWrite(48, 0, 0, 0);

  initI2S();
  Serial.println("[+] I2S 初始化完成");
  Serial.println("[+] 開始監測:對麥克風講話或吹氣,看 RMS / 音量條是否變化");
  Serial.println("");
  delay(500);
}

void loop() {
  static int32_t buf[BUF_SAMPLES];
  size_t bytesRead = 0;
  esp_err_t e = i2s_channel_read(rx_handle, buf, sizeof(buf), &bytesRead,
                                 100 / portTICK_PERIOD_MS);

  if (e != ESP_OK || bytesRead == 0) {
    Serial.println("[!] I2S 讀取失敗");
    delay(500);
    return;
  }

  int samples = bytesRead / 4;

  // 統計 RMS 跟最大絕對值
  int32_t maxAbs = 0;
  int64_t sumSq = 0;
  for (int i = 0; i < samples; i++) {
    int32_t v = buf[i] >> 14;  // 把 INMP441 32-bit 轉成有用的範圍
    int32_t av = v < 0 ? -v : v;
    if (av > maxAbs) maxAbs = av;
    sumSq += (int64_t)v * v;
  }
  int32_t rms = (int32_t)sqrt((double)sumSq / samples);

  // ASCII 音量條(0 ~ 5000 對應 0 ~ 50 格)
  int bar = (int)((float)rms / 5000.0f * 50.0f);
  if (bar > 50) bar = 50;
  if (bar < 0) bar = 0;

  Serial.printf("RMS=%5ld  MAX=%5ld  |", rms, maxAbs);
  for (int i = 0; i < bar; i++) Serial.print("#");
  for (int i = bar; i < 50; i++) Serial.print(" ");
  Serial.println("|");

  delay(50);  // 約 20 Hz 更新
}
