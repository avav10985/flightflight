// ============================================================
// Play_Test — 純粹「SD → I²S → MAX98357A」播放測試
//
// 不接 TFT、不接 INMP441、不接任何其他東西。
// 只測試播放路徑,排除 Voice_Test 多元件互相干擾的問題。
//
// === 接線 ===
//   SD 模組:                   MAX98357A:
//     VCC  → 5V                VIN  → 5V
//     GND  → GND               GND  → GND
//     CS   → GPIO 47           LRC  → GPIO 12
//     SCK  → GPIO 38           BCLK → GPIO 11
//     MOSI → GPIO 39           DIN  → GPIO 13
//     MISO → GPIO 40           OUT± → 喇叭
//
// === SD 卡需要 ===
//   根目錄放一個 /REC_001.WAV(32 kHz / 單聲道 / 16-bit PCM)
//
// === 預期 ===
// 接電後立刻開始循環播放 REC_001.WAV,**乾淨無雜訊**
//
// 如果乾淨 → 純播放路徑沒問題,Voice_Test 雜訊是「TFT/搖桿/選單」干擾
// 如果還是雜訊 → I²S TX 或 MAX98357A 跟 SD 共用 SPI 本身就會雜
// ============================================================

#include <SPI.h>
#include <SD.h>
#include <driver/i2s_std.h>

#define PIN_SD_CS      47
#define PIN_SPI_SCK    38
#define PIN_SPI_MOSI   39
#define PIN_SPI_MISO   40

#define PIN_BCLK       11
#define PIN_WS         12
#define PIN_DOUT       13
#define PIN_AMP_SD     18    // MAX98357A SD(shutdown 控制),HIGH = 啟用、LOW = 休眠靜音

#define SAMPLE_RATE    32000
#define BUF_SAMPLES    512

i2s_chan_handle_t tx_handle = NULL;
File              playFile;
uint32_t          playBytes = 0;
uint32_t          playTotal = 0;
bool              sdOK = false;

void initI2S() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  // 大 DMA buffer 避免 SD 讀慢時 TX underrun(原始設定)
  chan_cfg.dma_desc_num  = 8;
  chan_cfg.dma_frame_num = 512;
  i2s_new_channel(&chan_cfg, &tx_handle, NULL);   // 只要 TX

  i2s_std_config_t std_cfg = {};
  std_cfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
  std_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;   // 同時送 L+R 給 MAX98357A mono mix
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

void openWav() {
  if (playFile) playFile.close();
  playFile = SD.open("/REC_001.WAV", FILE_READ);
  if (!playFile) {
    Serial.println("[!] 開不了 /REC_001.WAV");
    return;
  }
  uint32_t fsize = playFile.size();
  playFile.seek(44);   // 跳 WAV header
  playBytes = 0;
  playTotal = fsize - 44;
  Serial.printf("[+] 開始播放 REC_001.WAV (%lu 音訊 bytes,約 %.1f 秒)\n",
                playTotal, playTotal / (float)(SAMPLE_RATE * 2));
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("  Play_Test:SD → I²S 純播放(無 TFT/其他)");
  Serial.println("========================================");

  neopixelWrite(48, 0, 0, 0);   // 關 RGB LED

  // MAX98357A SD 預設拉低(保持 shutdown,避免 boot 時喇叭聽到雜訊)
  // I²S 跑起來後再拉高啟用
  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, LOW);

  // SPI + SD(不卡死,SD 失敗也讓 setup 跑完)
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  sdOK = SD.begin(PIN_SD_CS, SPI);
  Serial.println(sdOK ? "[+] SD 掛載成功" : "[!] SD 沒接,進入閒置(插卡後重啟才會跑)");

  // I²S
  initI2S();
  Serial.println("[+] I²S 初始化完成");

  // 預先灌幾批靜音 sample 進 I²S DMA buffer,讓 MAX98357A 醒來看到的是
  // 穩定的 0 訊號,而不是 startup 階段的隨機 DOUT(消除 boot pop)
  int32_t silence[BUF_SAMPLES] = {0};
  size_t bytesWritten = 0;
  for (int i = 0; i < 4; i++) {
    i2s_channel_write(tx_handle, silence, sizeof(silence), &bytesWritten,
                      100 / portTICK_PERIOD_MS);
  }
  delay(20);

  // I²S 穩定後才啟用 MAX98357A,不會聽到 boot 雜訊
  digitalWrite(PIN_AMP_SD, HIGH);
  Serial.println("[+] MAX98357A 啟用(I²S 已穩定)");

  if (sdOK) {
    openWav();
    Serial.println("[+] 進入循環播放,聽聽看雜訊還在嗎");
  }
}

void loop() {
  if (!sdOK) { delay(500); return; }   // SD 沒接就閒置,等使用者插卡 + 重啟
  static uint8_t  pcmBuf[BUF_SAMPLES * 2];
  static int32_t  tx[BUF_SAMPLES];

  // 讀 SD
  size_t toRead = sizeof(pcmBuf);
  if (playBytes + toRead > playTotal) toRead = playTotal - playBytes;
  if (toRead == 0) {
    // 播完一輪,重來
    Serial.println("[+] 播完一輪,重頭來");
    playFile.seek(44);
    playBytes = 0;
    return;
  }

  int n = playFile.read(pcmBuf, toRead);
  if (n <= 0) return;

  int samples = n / 2;
  int16_t *pcm = (int16_t*)pcmBuf;
  for (int i = 0; i < samples; i++) {
    tx[i] = (int32_t)pcm[i] << 16;        // 16-bit → 32-bit MSB-aligned
  }

  size_t bytesWritten = 0;
  i2s_channel_write(tx_handle, tx, samples * 4, &bytesWritten, portMAX_DELAY);
  playBytes += n;
}
