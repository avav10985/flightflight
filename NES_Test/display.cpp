// NES 顯示層 — LovyanGFX 版(flightflight 手把 V2-A)
//
// 原範例用 Arduino_GFX,在我們的板子上白畫面(init 不相容);
// 改用手把主程式驗證過的 LovyanGFX ILI9341 配置(2026-06-12)。

extern "C"
{
#include <nes/nes.h>
}

#include "hw_config.h"
#include <LovyanGFX.hpp>

// ====== 手把 TFT(同 Ground_TX_ESP32 配置)======
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
      cfg.bus_shared    = false;   // NES_Test 獨佔 SPI2(SD 在 SPI3)
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};
static LGFX tft;

static int16_t w, h, frame_x, frame_y, frame_x_offset, frame_width, frame_height, frame_line_pixels;
extern int16_t bg_color;
extern uint16_t myPalette[];

static uint16_t lineBuf[NES_SCREEN_WIDTH];

extern void display_begin()
{
    tft.init();
    tft.setRotation(3);   // 橫向 320×240(畫面顛倒就改 1)
    bg_color = tft.color565(24, 28, 24);
    tft.fillScreen(bg_color);
}

extern void display_message(const char *msg)
{
    tft.setTextColor(TFT_ORANGE, bg_color);
    tft.setTextSize(2);
    tft.setCursor(8, 100);
    tft.println(msg);
}

extern "C" void display_init()
{
    w = tft.width();    // rotation 3 → 320
    h = tft.height();   //              240
    if (w > NES_SCREEN_WIDTH)
    {
        frame_x = (w - NES_SCREEN_WIDTH) / 2;
        frame_x_offset = 0;
        frame_width = NES_SCREEN_WIDTH;
    }
    else
    {
        frame_x = 0;
        frame_x_offset = (NES_SCREEN_WIDTH - w) / 2;
        frame_width = w;
    }
    frame_height = NES_SCREEN_HEIGHT;
    frame_line_pixels = frame_width;
    frame_y = (h - NES_SCREEN_HEIGHT) / 2;
    if (frame_y < 0) frame_y = 0;
}

extern "C" void display_write_frame(const uint8_t *data[])
{
    tft.startWrite();
    tft.setAddrWindow(frame_x, frame_y, frame_width, frame_height);
    for (int32_t i = 0; i < NES_SCREEN_HEIGHT; i++)
    {
        const uint8_t *src = data[i] + frame_x_offset;
        for (int32_t x = 0; x < frame_line_pixels; x++)
            lineBuf[x] = myPalette[src[x]];
        tft.writePixels(lineBuf, frame_line_pixels, true);
    }
    tft.endWrite();
}

extern "C" void display_clear()
{
    tft.fillScreen(bg_color);
}
