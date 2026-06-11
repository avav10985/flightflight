// ============================================================
// SD_Test — 單獨測 SD 卡(獨立 SPI3 新腳位版,2026-06-12)
//
// 沒有 TFT、沒有 I²S、沒有 NRF24,只測 SD。用 USB + Serial。
//
// === 接線(6 條)===
//   SD 模組 VCC  → ESP32-S3 板上的 5V 腳(⚠ 測試期間暫時改接這裡!)
//   SD 模組 GND  → GND
//   SD 模組 SCK  → GPIO 15
//   SD 模組 MOSI → GPIO 17
//   SD 模組 MISO → GPIO 3
//   SD 模組 CS   → GPIO 47
//
// === ⚠ 為什麼 VCC 要暫時改接板上 5V 腳 ===
//   平常 SD VCC 接 buck 5V 軌,但那條軌只有「電池模式」才有電。
//   這支測試要用 USB 看 Serial(電池要拔),所以 SD 改吃
//   板上 5V 腳(USB 進來的 5V)。測完整合時再接回 buck 軌。
//
// === 操作 ===
//   拔電池 → SD VCC 改接板上 5V → USB 燒錄 → 開 Serial Monitor 115200
//   會自動用 4MHz → 1MHz → 400kHz 三種速度嘗試掛載
// ============================================================

#include <SPI.h>
#include <SD.h>

#define PIN_SD_CS    47
#define PIN_SD_SCK   15
#define PIN_SD_MOSI  17
#define PIN_SD_MISO   3

SPIClass spiSD(HSPI);   // S3 第二組 SPI(SPI3)

bool tryMount(uint32_t freq) {
  Serial.printf("[*] 嘗試掛載 @ %lu kHz ... ", freq / 1000);
  if (SD.begin(PIN_SD_CS, spiSD, freq)) {
    Serial.println("成功!");
    return true;
  }
  Serial.println("失敗");
  SD.end();
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== SD 卡測試(獨立 SPI3:SCK15 / MOSI17 / MISO3 / CS47)===");

  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  spiSD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

  bool ok = tryMount(4000000) || tryMount(1000000) || tryMount(400000);
  if (!ok) {
    Serial.println("\n[!] 全部速度都掛載失敗,依序檢查:");
    Serial.println("    1. SD 模組 VCC 是不是接到板上 5V 腳?(USB 模式 buck 軌沒電)");
    Serial.println("    2. 六條線有沒有接對 / 接穩?(SCK15 MOSI17 MISO3 CS47)");
    Serial.println("    3. 卡有沒有插到底?是不是 FAT32?(≤32GB,電腦重格一次)");
    Serial.println("    4. 換一張卡試試(卡本身可能跟模組不合)");
    return;
  }

  // 卡資訊
  uint8_t type = SD.cardType();
  const char* typeName = (type == CARD_MMC) ? "MMC" :
                         (type == CARD_SD)  ? "SDSC" :
                         (type == CARD_SDHC)? "SDHC" : "UNKNOWN";
  Serial.printf("[+] 卡類型:%s\n", typeName);
  Serial.printf("[+] 容量:%.1f GB\n", SD.cardSize() / 1073741824.0);
  Serial.printf("[+] 已用:%llu MB / %llu MB\n",
                SD.usedBytes() / 1048576ULL, SD.totalBytes() / 1048576ULL);

  // 寫入測試
  File f = SD.open("/sd_test.txt", FILE_WRITE);
  if (f) {
    f.println("hello from SD_Test");
    f.close();
    Serial.println("[+] 寫入測試 OK(/sd_test.txt)");
  } else {
    Serial.println("[!] 掛載成功但寫入失敗(卡寫保護?)");
  }

  // 列根目錄
  Serial.println("[+] 根目錄:");
  File root = SD.open("/");
  File entry;
  while ((entry = root.openNextFile())) {
    Serial.printf("    %s  %lu bytes\n", entry.name(), (unsigned long)entry.size());
    entry.close();
  }
  root.close();
  Serial.println("\n=== 測試完成 ===");
}

void loop() { delay(1000); }
