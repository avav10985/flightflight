// ============================================================
// NES_Test — 手把跑 NES 模擬器(arduino-nofrendo)
//
// === 需要的函式庫(git clone 到 Documents/Arduino/libraries)===
//   arduino-nofrendo(moononournation)
//   Arduino_GFX    (moononournation)
//
// === SD 卡 ===
//   根目錄放一個 .nes ROM(會自動找第一個)。
//   ⚠ 版權:請用 homebrew 免費遊戲測試。
//
// === 操作 ===
//   右搖桿上下 = 跳/蹲   左搖桿左右 = 移動
//   右肩鈕 = A   左肩鈕 = B
//   按鍵叢 OK = Start   按鍵叢 返回 = Select
//
// === 硬體 ===
//   全部沿用手把現有接線,不用改線
// ============================================================

#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <SPI.h>
#include <SD.h>
#include "hw_config.h"

extern "C"
{
#include <nofrendo.h>
}

SPIClass spiSD(HSPI);   // SD 獨立 SPI3(同 Ground_TX:SCK15 MISO47 MOSI17 CS0)

int16_t bg_color;
extern void display_begin();
extern void display_message(const char *msg);

void setup()
{
  // 顯示最先初始化:畫面變深灰 = setup 有在跑(除錯白畫面用)
  display_begin();
  display_message("display OK");

  Serial.begin(115200);
  display_message("serial OK");

  esp_task_wdt_deinit();  // 模擬器主迴圈會佔滿 CPU,關看門狗
  display_message("wdt off");

  // MAX98357A 開聲音(平常拉低靜音,模擬器要出聲)
  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, HIGH);
  display_message("amp on");

  // SD:獨立 SPI3 @20MHz,掛載到 /fs 讓 nofrendo 用 C fopen 讀 ROM
  display_message("SD init...");
  spiSD.begin(15, 47, 17, 0);
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);
  if (!SD.begin(0, spiSD, 20000000, FSROOT))
  {
    Serial.println("SD 掛載失敗");
    display_message("SD mount failed!");
    while (1) delay(1000);
  }
  display_message("SD OK");
  FS filesystem = SD;

  // 找根目錄第一個 .nes
  File root = filesystem.open("/");
  char *argv[1];
  bool foundRom = false;
  static char fullFilename[256];
  if (root)
  {
    File file = root.openNextFile();
    while (file)
    {
      if (!file.isDirectory())
      {
        char *filename = (char *)file.name();
        int len = strlen(filename);
        if (len > 4 && strstr(strlwr(filename + (len - 4)), ".nes"))
        {
          foundRom = true;
          sprintf(fullFilename, "%s/%s", FSROOT, filename);
          Serial.println(fullFilename);
          argv[0] = fullFilename;
          break;
        }
      }
      file = root.openNextFile();
    }
  }

  if (!foundRom)
  {
    Serial.println("SD 根目錄找不到 .nes ROM");
    display_message("No .nes ROM on SD root!");
    while (1) delay(1000);
  }

  display_message("ROM found, NES start!");
  delay(500);
  Serial.println("NoFrendo start!");
  nofrendo_main(1, argv);
  Serial.println("NoFrendo end!");
}

void loop() {}
