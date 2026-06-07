// ============================================================
// SPK_Test — 純粹單獨測 MAX98357A + 喇叭 是否還活著
//
// **不需要 SD / TFT / INMP441 / 任何其他元件**,只接 MAX98357A 跟喇叭。
//
// 程式直接產生 440 Hz(中央 A 音,「La~」)正弦波,送 I²S 出去。
// 接好如果有聽到「La~~」連續嗶聲 → MAX98357A 跟喇叭都活的!
//
// === 接線(5 條訊號 + 2 條喇叭)===
//   MAX98357A VIN  → ESP32 5V  ⚠️ **不是 3V3,這跟 INMP441 相反**
//   MAX98357A GND  → ESP32 GND
//   MAX98357A LRC  → ESP32 GPIO 12
//   MAX98357A BCLK → ESP32 GPIO 11
//   MAX98357A DIN  → ESP32 GPIO 13
//   MAX98357A GAIN → 不接(預設 9dB)
//   MAX98357A SD   → 不接(浮空 = mono mix)
//   MAX98357A OUT+ → 喇叭「+」焊盤
//   MAX98357A OUT− → 喇叭「−」焊盤
//
// === 預期 ===
// 接電後 1 秒內喇叭發出「La~~~~」連續中音(440 Hz)
//
// 如果完全沒聲 → MAX98357A / 喇叭 / 接線 其中之一壞了
// 如果有聲但破音很大 → 喇叭可能受傷,或 GAIN 接到 GND 太大聲
// 如果有聲但很小聲 → 喇叭OK,只是 GAIN 預設 9dB 較保守
// 如果一接電 3V3 掉壓 reset → MAX98357A 內部短路,跟 INMP441 同樣下場
// ============================================================

#include <driver/i2s_std.h>
#include <math.h>

#define PIN_BCLK    11
#define PIN_WS      12
#define PIN_DOUT    13
#define PIN_AMP_SD  17   // MAX98357A SD,HIGH=啟用、LOW=休眠靜音

#define SAMPLE_RATE 16000
#define TONE_FREQ   440.0f      // A4「La」音
#define BUF_SAMPLES 512
#define AMPLITUDE   16000       // 16-bit 最大 32767,用半幅避免吵到鄰居

i2s_chan_handle_t tx_handle = NULL;

void initI2S() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, &tx_handle, NULL);   // 只要 TX

  i2s_std_config_t std_cfg = {};
  std_cfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  std_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;   // 送 L+R(MAX98357A SD 浮空 = 混音)

  std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  std_cfg.gpio_cfg.bclk = (gpio_num_t)PIN_BCLK;
  std_cfg.gpio_cfg.ws   = (gpio_num_t)PIN_WS;
  std_cfg.gpio_cfg.dout = (gpio_num_t)PIN_DOUT;
  std_cfg.gpio_cfg.din  = I2S_GPIO_UNUSED;
  std_cfg.gpio_cfg.invert_flags.mclk_inv = false;
  std_cfg.gpio_cfg.invert_flags.bclk_inv = false;
  std_cfg.gpio_cfg.invert_flags.ws_inv   = false;

  i2s_channel_init_std_mode(tx_handle, &std_cfg);
  i2s_channel_enable(tx_handle);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("       MAX98357A + 喇叭 單獨測試");
  Serial.println("========================================");
  Serial.println("接線:");
  Serial.println("  VIN  → ESP32 5V   ⚠️ (不能 3V3)");
  Serial.println("  GND  → ESP32 GND");
  Serial.println("  LRC  → GPIO 12");
  Serial.println("  BCLK → GPIO 11");
  Serial.println("  DIN  → GPIO 13");
  Serial.println("  OUT+/− → 喇叭");
  Serial.println("");

  // 關 RGB LED
  neopixelWrite(48, 0, 0, 0);

  // MAX98357A SD 控制:HIGH 啟用(SPK_Test 整個跑都要播)
  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, HIGH);

  initI2S();
  Serial.println("[+] I²S 初始化完成");
  Serial.printf("[+] 開始播放 %.0f Hz 正弦波(中央 A 音 / La)\n", TONE_FREQ);
  Serial.println("[+] 應該聽到連續「嗶~~」音");
  Serial.println("");
}

void loop() {
  static float phase = 0.0f;
  const float phaseInc = 2.0f * M_PI * TONE_FREQ / SAMPLE_RATE;
  static int16_t buf[BUF_SAMPLES];

  // 產生一塊正弦波 samples
  for (int i = 0; i < BUF_SAMPLES; i++) {
    buf[i] = (int16_t)(sinf(phase) * AMPLITUDE);
    phase += phaseInc;
    if (phase >= 2.0f * M_PI) phase -= 2.0f * M_PI;
  }

  // 推給 I²S
  size_t bytesWritten = 0;
  i2s_channel_write(tx_handle, buf, sizeof(buf), &bytesWritten, portMAX_DELAY);

  // 每秒印一次,讓你知道程式在跑
  static uint32_t lastPrint = 0;
  static uint32_t sampleCount = 0;
  sampleCount += BUF_SAMPLES;
  if (millis() - lastPrint > 1000) {
    Serial.printf("[+] 已送 %lu samples(約 %.1f 秒)\n", sampleCount, sampleCount / (float)SAMPLE_RATE);
    lastPrint = millis();
  }
}
