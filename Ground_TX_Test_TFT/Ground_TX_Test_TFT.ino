// ============================================================
// Ground_TX_Test_TFT — 手把腳位下的 TFT 獨立測試
//
// 從 tyty/tyty.ino 改寫:LGFX 配置一模一樣,**只換腳位**為我們手把用的:
//   SCK  = GPIO 38 (tyty 是 8)
//   MOSI = GPIO 39 (tyty 是 18)
//   MISO = -1      (tyty 也是 -1)
//   DC   = GPIO 21 (tyty 是 17)
//   CS   = GPIO 48 (tyty 是 15)
//   RST  = GPIO 16 (跟 tyty 一樣,主動驅動)
//
// **不含** NRF24 / SD / 搖桿 / 開關 等所有其他功能,純驗證 TFT 顯示。
//
// 接線需求(目前 R10 已拆,RESET 必須接 GPIO 16):
//   TFT VCC   → ESP32 3V3 腳
//   TFT GND   → ESP32 GND 腳
//   TFT LED   → ESP32 3V3 腳(或 B 板 3V3 軌)
//   TFT CS    → ESP32 GPIO 48
//   TFT RESET → ESP32 GPIO 16    ← R10 拆了,要主動接這條
//   TFT DC    → ESP32 GPIO 21
//   TFT SCK   → ESP32 GPIO 38
//   TFT MOSI  → ESP32 GPIO 39
//   TFT MISO  → 不接
//   T_*       → 不接
//
// 預期顯示:藍色標題列 + 4 個圓角色塊 + 漸層條 + 底部 3 行文字
// ============================================================

#include <LovyanGFX.hpp>

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
      cfg.pin_miso    = -1;
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
      cfg.bus_shared    = false;   // 獨立測試,沒共用 SPI
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

LGFX display;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Handle TFT Test ===");
  Serial.println("LGFX init...");

  display.init();
  display.setRotation(2);     // 直立 240×320 翻 180°(實體上下顛倒,故設 2)
  display.fillScreen(TFT_BLACK);

  // 標題列(藍底白字)
  display.fillRect(0, 0, 240, 30, display.color565(0, 100, 200));
  display.setTextColor(TFT_WHITE);
  display.setFont(&fonts::Font2);
  display.setCursor(8, 8);
  display.print("Handle TFT Test");

  // 4 個圓角色塊
  display.fillRoundRect(20, 50, 80, 60, 8, TFT_RED);
  display.fillRoundRect(120, 50, 80, 60, 8, TFT_GREEN);
  display.fillRoundRect(20, 130, 80, 60, 8, TFT_BLUE);
  display.fillRoundRect(120, 130, 80, 60, 8, TFT_YELLOW);

  // 漸層條
  for (int x = 0; x < 240; x++) {
    uint8_t r = map(x, 0, 240, 255, 0);
    uint8_t b = map(x, 0, 240, 0, 255);
    display.drawFastVLine(x, 210, 20, display.color565(r, 0, b));
  }

  // 底部三行文字
  display.setTextColor(TFT_WHITE);
  display.setCursor(8, 245);
  display.print("Pins: SCK=38 MOSI=39");
  display.setCursor(8, 265);
  display.print("DC=21 CS=48 RST=16");
  display.setCursor(8, 290);
  display.setTextColor(TFT_GREEN);
  display.print("If you see this, TFT OK!");

  Serial.println("[OK] Drawing done");
}

void loop() {
  delay(1000);
}
