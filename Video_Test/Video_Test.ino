// ============================================================
// Video_Test — SD 卡 MJPEG 影片 + WAV 聲音,播在 2.8" TFT 上
//
// === 需要的函式庫 ===
//   JPEGDEC(Library Manager 搜「JPEGDEC」,作者 Larry Bank)
//
// === SD 卡放兩個檔(根目錄)===
//   /video.mjp  — MJPEG 影片(連續 JPEG 幀)
//   /video.wav  — 聲音(32kHz / 單聲道 / 16-bit PCM)
//
// === 電腦轉檔指令(ffmpeg,直式螢幕,寬 240 高度自動 @15fps)===
//   ffmpeg -i 影片.mp4 -vf "scale=240:320:force_original_aspect_ratio=decrease:force_divisible_by=2,fps=15" -q:v 8 -f mjpeg video.mjp
//   ffmpeg -i 影片.mp4 -ar 32000 -ac 1 -sample_fmt s16 video.wav
//   (橫式直式來源都會塞進 240×320 內,程式自動置中,黑邊不編碼)
//
// === 接線(全部沿用手把現有配置,不用改線)===
//   SD:SPI3(SCK15 / MOSI17 / MISO47 / CS0)
//   TFT:SPI2(38/39/40,CS48 DC21 RST16)
//   MAX98357A:BCLK11 / LRC12 / DIN13 / SD(shutdown)18
//
// === 同步機制 ===
//   聲音是時鐘:I²S 以固定 32kHz 消化資料,用「已餵進去的
//   聲音秒數」推算現在該播第幾幀;影像解碼太慢就跳幀追上。
// ============================================================

#include <SPI.h>
#include <SD.h>
#include <driver/i2s_std.h>
#include <LovyanGFX.hpp>
#include <JPEGDEC.h>

// ---- SD 獨立 SPI3 ----
#define PIN_SD_CS     0
#define PIN_SD_SCK   15
#define PIN_SD_MOSI  17
#define PIN_SD_MISO  47

// ---- I²S 喇叭 ----
#define PIN_BCLK     11
#define PIN_WS       12
#define PIN_DOUT     13
#define PIN_AMP_SD   18
#define AUDIO_RATE   32000

#define VIDEO_FPS    15.0f   // 配合轉檔 fps=15;解碼+SPI 推屏的舒適範圍
#define FRAME_BUF_SZ (160 * 1024)   // 單幀 JPEG 上限(PSRAM)

// ====== TFT(沿用 Ground_TX 配置)======
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
      cfg.bus_shared    = false;   // 這支 SD 在 SPI3,TFT 獨佔 SPI2,可關 shared 換速度
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};
LGFX tft;

SPIClass spiSD(HSPI);
JPEGDEC  jpeg;

File     vidFile, audFile;
uint8_t* frameBuf = nullptr;
i2s_chan_handle_t audioTx = nullptr;
uint64_t audioFed   = 0;     // 已餵進 I²S 的 PCM bytes(時鐘)
uint32_t frameShown = 0;
bool     hasAudio   = false;

// JPEGDEC 畫圖 callback:解碼出的區塊直接推上螢幕
int jpegDraw(JPEGDRAW* d) {
  tft.pushImage(d->x, d->y, d->iWidth, d->iHeight, (uint16_t*)d->pPixels);
  return 1;
}

void initAudio() {
  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  cc.dma_desc_num  = 8;
  cc.dma_frame_num = 512;
  i2s_new_channel(&cc, &audioTx, NULL);
  i2s_std_config_t sc = {};
  sc.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_RATE);
  sc.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
  sc.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
  sc.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  sc.gpio_cfg.bclk = (gpio_num_t)PIN_BCLK;
  sc.gpio_cfg.ws   = (gpio_num_t)PIN_WS;
  sc.gpio_cfg.dout = (gpio_num_t)PIN_DOUT;
  sc.gpio_cfg.din  = I2S_GPIO_UNUSED;
  i2s_channel_init_std_mode(audioTx, &sc);
  i2s_channel_enable(audioTx);
  int32_t silence[256] = {0};
  size_t w;
  for (int i = 0; i < 4; i++)
    i2s_channel_write(audioTx, silence, sizeof(silence), &w, 50 / portTICK_PERIOD_MS);
}

// 非阻塞餵聲音,回傳是否還有資料
void feedAudio() {
  if (!hasAudio) return;
  static uint8_t pcm[512];
  static int32_t tx[256];
  for (int round = 0; round < 4; round++) {
    int n = audFile.read(pcm, sizeof(pcm));
    if (n <= 0) { hasAudio = false; return; }
    int samples = n / 2;
    int16_t* s = (int16_t*)pcm;
    for (int i = 0; i < samples; i++) tx[i] = ((int32_t)s[i]) << 16;
    size_t written = 0;
    i2s_channel_write(audioTx, tx, samples * 4, &written, 0);
    size_t usedPcm = written / 2;
    audioFed += usedPcm;
    if ((int)usedPcm < n) {
      audFile.seek(audFile.position() - (n - usedPcm));
      return;   // DMA 滿
    }
  }
}

// 從 MJPEG 流讀下一幀(FFD8 ... FFD9)進 frameBuf,回傳長度(0=讀完)
// 1KB 塊狀讀取 + 逐 byte 掃描標記,標記跨塊也抓得到(prev 跨迴圈保留)
size_t readFrame() {
  static uint8_t chunk[1024];
  int len = 0, prev = -1;
  bool inFrame = false;
  while (true) {
    int n = vidFile.read(chunk, sizeof(chunk));
    if (n <= 0) return 0;                        // 檔案結束
    for (int i = 0; i < n; i++) {
      int c = chunk[i];
      if (!inFrame) {
        if (prev == 0xFF && c == 0xD8) {         // 幀開始
          inFrame = true;
          frameBuf[0] = 0xFF; frameBuf[1] = 0xD8;
          len = 2;
        }
        prev = c;
        continue;
      }
      if (len < FRAME_BUF_SZ) frameBuf[len++] = (uint8_t)c;
      if (prev == 0xFF && c == 0xD9) {           // 幀結束
        vidFile.seek(vidFile.position() - (n - i - 1));   // 多讀的倒帶
        return len;
      }
      prev = c;
    }
  }
}

void showMsg(const char* m, uint16_t color) {
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::efontTW_24);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(10, 100);
  tft.println(m);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, LOW);

  tft.init();
  tft.setRotation(2);          // 直立 240×320,跟手把其他程式同方向
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);      // JPEGDEC 輸出 RGB565 little-endian

  // SD:獨立 bus + 10cm 短線,拉高時脈餵影片(預設 4MHz 只有 ~400KB/s 不夠)
  spiSD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  pinMode(PIN_SD_CS, OUTPUT);
  bool sdMounted = SD.begin(PIN_SD_CS, spiSD, 20000000) ||
                   SD.begin(PIN_SD_CS, spiSD, 10000000) ||
                   SD.begin(PIN_SD_CS, spiSD, 4000000);
  if (!sdMounted) { showMsg("SD 掛載失敗", TFT_RED); while (1) delay(1000); }

  // 幀緩衝(PSRAM)
  frameBuf = (uint8_t*)heap_caps_malloc(FRAME_BUF_SZ, MALLOC_CAP_SPIRAM);
  if (!frameBuf) { showMsg("PSRAM 配置失敗", TFT_RED); while (1) delay(1000); }

  // 開檔
  vidFile = SD.open("/video.mjp", FILE_READ);
  if (!vidFile) { showMsg("找不到 /video.mjp", TFT_RED); while (1) delay(1000); }
  audFile = SD.open("/video.wav", FILE_READ);
  if (audFile) {
    audFile.seek(44);
    hasAudio = true;
    initAudio();
    digitalWrite(PIN_AMP_SD, HIGH);
  } else {
    Serial.println("[*] 沒有 /video.wav,無聲播放");
  }

  Serial.println("[+] 開始播放");
}

void loop() {
  feedAudio();

  // 目前該播到第幾幀(有聲音用聲音當時鐘,沒聲音用 millis)
  uint32_t target;
  if (audFile) {
    float sec = audioFed / (float)(AUDIO_RATE * 2);
    target = (uint32_t)(sec * VIDEO_FPS);
  } else {
    target = (uint32_t)(millis() / 1000.0f * VIDEO_FPS);
  }

  if (frameShown >= target) { delay(1); return; }   // 還不到下一幀時間

  size_t len = readFrame();
  if (len == 0) {   // 影片播完 → 從頭循環
    Serial.println("[+] 播完,重頭循環");
    vidFile.seek(0);
    if (audFile) { audFile.seek(44); hasAudio = true; }
    audioFed = 0;
    frameShown = 0;
    return;
  }
  frameShown++;

  // 落後太多就跳幀(只讀不解碼,追上聲音)
  if (target - frameShown > 2) return;

  if (jpeg.openRAM(frameBuf, len, jpegDraw)) {
    // 影片只編碼內容(寬 240、高自動),畫面上下置中
    int xoff = (240 - jpeg.getWidth())  / 2;
    int yoff = (320 - jpeg.getHeight()) / 2;
    if (xoff < 0) xoff = 0;
    if (yoff < 0) yoff = 0;
    jpeg.decode(xoff, yoff, 0);
    jpeg.close();
  }
}
