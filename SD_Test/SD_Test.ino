// ============================================================
// SD_Test — 單獨測 SD 卡(獨立 SPI3 + TFT 顯示版,2026-06-12)
//
// 結果全部顯示在 TFT,不依賴 Serial。
// 電池模式測試:SD VCC 留在 buck 5V 軌「不用改接線」。
//
// === SD 接線(6 條)===
//   VCC → buck 5V 軌(不動)
//   GND → 共地
//   SCK  → GPIO 15
//   MOSI → GPIO 17
//   MISO → GPIO 47(2026-06-12 改:GPIO 3 是接線總表標記「避開」的腳,我選錯了)
//   CS   → GPIO 0 (BOOT 腳,輸出閒置 HIGH 安全;C 板 R11 10k 上拉本來就是為它準備的)
//
// === 操作 ===
//   拔電池 → USB 燒錄 → 拔 USB → 接電池
//   TFT 會顯示三段速度的掛載結果 + 卡資訊 + 寫入測試
// ============================================================

#include <SPI.h>
#include <SD.h>
#include <LovyanGFX.hpp>

#define PIN_SD_CS     0
#define PIN_SD_SCK   15
#define PIN_SD_MOSI  17
#define PIN_SD_MISO  47

SPIClass spiSD(HSPI);   // S3 第二組 SPI(SPI3)

// ====== TFT(沿用 Ground_TX_ESP32 配置)======
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
      cfg.bus_shared    = true;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};
LGFX tft;

int yCur = 8;   // TFT 顯示游標

// 同時印到 Serial + TFT(一行一行往下)
void show(const char* msg, uint16_t color = TFT_WHITE) {
  Serial.println(msg);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(6, yCur);
  tft.print(msg);
  yCur += 22;
  if (yCur > 300) {   // 滿了就清掉重來
    tft.fillScreen(TFT_BLACK);
    yCur = 8;
  }
}

bool tryMount(uint32_t freq) {
  char buf[48];
  snprintf(buf, sizeof(buf), "掛載 @%lukHz ...", freq / 1000);
  show(buf, TFT_YELLOW);
  if (SD.begin(PIN_SD_CS, spiSD, freq)) {
    show("  → 成功!", TFT_GREEN);
    return true;
  }
  show("  → 失敗", TFT_RED);
  SD.end();
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::efontTW_16);

  show("=== SD 測試(獨立 SPI3)===", TFT_CYAN);
  show("SCK15 MOSI17 MISO47 CS0");

  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  spiSD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

  bool ok = tryMount(4000000) || tryMount(1000000) || tryMount(400000);
  if (!ok) {
    show("全部失敗,檢查:", TFT_RED);
    show("1. SCK15 MOSI17 MISO47 CS0");
    show("2. 卡插到底?FAT32?");
    show("3. 換一張卡試試");
    return;
  }

  // 卡資訊
  char buf[64];
  uint8_t type = SD.cardType();
  const char* typeName = (type == CARD_MMC) ? "MMC" :
                         (type == CARD_SD)  ? "SDSC" :
                         (type == CARD_SDHC)? "SDHC" : "UNKNOWN";
  snprintf(buf, sizeof(buf), "卡:%s %.1fGB", typeName, SD.cardSize() / 1073741824.0);
  show(buf, TFT_GREEN);

  // 寫入測試
  File f = SD.open("/sd_test.txt", FILE_WRITE);
  if (f) {
    f.println("hello from SD_Test");
    f.close();
    show("寫入測試 OK", TFT_GREEN);
  } else {
    show("掛載 OK 但寫入失敗!", TFT_ORANGE);
  }

  // 列根目錄(最多 8 個)
  show("根目錄:", TFT_CYAN);
  File root = SD.open("/");
  File entry;
  int n = 0;
  while ((entry = root.openNextFile()) && n < 8) {
    snprintf(buf, sizeof(buf), " %s", entry.name());
    show(buf);
    entry.close();
    n++;
  }
  root.close();
  show("=== 測試完成 ===", TFT_CYAN);
}

void loop() { delay(1000); }
