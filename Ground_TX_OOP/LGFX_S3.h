#pragma once
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
// ============================================================
// LovyanGFX：MSP2806 2.8" SPI ILI9341
// 配置照 tyty/tyty.ino 試燒成功的設定為基準,只改我們共用 SPI 必要的部分
// 直立 240×320
// ============================================================
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
      cfg.pin_sclk    = 38;                // 跟 NRF24/SD 共用(設計如此)
      cfg.pin_mosi    = 39;                // 同上
      cfg.pin_miso    = 40;                // 跟 SD 共用 SPI 必須設 40,否則 SD 拿不到 MISO 掛載失敗
      cfg.pin_dc      = 21;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs        = 48;
      cfg.pin_rst       = 16;              // GPIO 16 主動驅動(同 tyty,沒上拉電阻)
      cfg.pin_busy      = -1;
      cfg.memory_width  = 240;
      cfg.memory_height = 320;
      cfg.panel_width   = 240;
      cfg.panel_height  = 320;
      cfg.readable      = false;
      cfg.invert        = false;
      cfg.rgb_order     = false;
      cfg.dlen_16bit    = false;
      cfg.bus_shared    = true;            // ★ 跟 NRF24/SD 共用 SPI 必開(tyty 是 false 因獨立)
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};
