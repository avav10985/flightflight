// ============================================================
// SD_Test — 純粹單獨測 SD 卡模組能不能掛載
//
// 沒有 TFT、沒有 I²S、沒有 NRF24,**只測 SD**。
//
// === 接線(6 條)===
//   SD 模組 VCC  → ESP32 3V3
//   SD 模組 GND  → ESP32 GND
//   SD 模組 MOSI → ESP32 GPIO 39
//   SD 模組 MISO → ESP32 GPIO 40
//   SD 模組 SCK  → ESP32 GPIO 38
//   SD 模組 CS   → ESP32 GPIO 0
//
// === ⚠️ 重要:GPIO 0 必須有上拉 ===
//   方法 1(正式):10kΩ 電阻接 GPIO 0 跟 3V3 之間
//   方法 2(臨時):直接用杜邦線把 GPIO 0 跟 3V3 短接
//
//   沒上拉的話 ESP32 開機可能進燒錄模式或 SD 不認得。
//
// === 預期 Serial 輸出 ===
//   === SD 卡測試 ===
//   [+] SD 掛載成功
//   [+] 卡類型:SDHC
//   [+] 容量:15.0 GB
//   [+] 已用空間:0 MB / 15330 MB
//   [+] 根目錄檔案清單:
//       (沒檔案就什麼也不印)
//
// === 如果失敗 ===
//   [!] SD 掛載失敗
//   → 檢查接線、R11、SD 卡有沒有真的插進模組、卡有沒有壞
// ============================================================

#include <SPI.h>
#include <SD.h>

// CS = GPIO 47(右側自由腳,不用上拉電阻,接線方便)
// 如果要用 GPIO 0(BOOT 腳)需要 R11 10kΩ 上拉到 3V3,否則進燒錄模式
#define PIN_SD_CS    47
#define PIN_SPI_SCK  38
#define PIN_SPI_MOSI 39
#define PIN_SPI_MISO 40

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("           SD 卡測試");
  Serial.println("========================================");
  Serial.printf("CS  = GPIO %d\n", PIN_SD_CS);
  Serial.printf("SCK = GPIO %d\n", PIN_SPI_SCK);
  Serial.printf("MOSI= GPIO %d\n", PIN_SPI_MOSI);
  Serial.printf("MISO= GPIO %d\n", PIN_SPI_MISO);
  Serial.println("");

  // 關 RGB LED
  neopixelWrite(48, 0, 0, 0);

  // 啟動 SPI 匯流排,指定腳位
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);

  // GPIO 0 (CS) 設輸出 + 預設 HIGH
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  // 嘗試掛載
  Serial.println("掛載中...");
  if (!SD.begin(PIN_SD_CS, SPI)) {
    Serial.println("[!] SD 掛載失敗");
    Serial.println("");
    Serial.println("可能原因:");
    Serial.println("  1. R11(10kΩ 上拉到 3V3)沒接,GPIO 0 浮空");
    Serial.println("  2. CS / SCK / MOSI / MISO 接錯腳");
    Serial.println("  3. SD 卡沒插進模組");
    Serial.println("  4. SD 卡沒格式化成 FAT32");
    Serial.println("  5. SD 模組或 SD 卡壞了");
    while (1) delay(1000);
  }

  Serial.println("[+] SD 掛載成功!");

  // 顯示卡資訊
  uint8_t cardType = SD.cardType();
  const char* typeName = "未知";
  if (cardType == CARD_MMC)       typeName = "MMC";
  else if (cardType == CARD_SD)   typeName = "SDSC";
  else if (cardType == CARD_SDHC) typeName = "SDHC";
  else if (cardType == CARD_NONE) typeName = "無卡";
  Serial.printf("[+] 卡類型:%s\n", typeName);

  uint64_t totalBytes = SD.totalBytes();
  uint64_t usedBytes  = SD.usedBytes();
  Serial.printf("[+] 總容量:%.2f GB\n", totalBytes / 1024.0 / 1024.0 / 1024.0);
  Serial.printf("[+] 已使用:%llu MB / %llu MB\n",
                usedBytes / 1024 / 1024, totalBytes / 1024 / 1024);

  // 列出根目錄檔案
  Serial.println("");
  Serial.println("[+] 根目錄檔案清單:");
  File root = SD.open("/");
  if (root) {
    bool any = false;
    while (true) {
      File f = root.openNextFile();
      if (!f) break;
      if (f.isDirectory()) {
        Serial.printf("    [DIR]  %s\n", f.name());
      } else {
        Serial.printf("    %8u  %s\n", f.size(), f.name());
      }
      f.close();
      any = true;
    }
    if (!any) Serial.println("    (空白)");
    root.close();
  }

  // 寫測試檔
  Serial.println("");
  Serial.println("[+] 嘗試寫入測試檔 /TEST.TXT ...");
  File f = SD.open("/TEST.TXT", FILE_WRITE);
  if (f) {
    f.println("Hello from ESP32-S3");
    f.printf("Millis: %lu\n", millis());
    f.close();
    Serial.println("[+] 寫入成功!");
  } else {
    Serial.println("[!] 寫入失敗");
  }

  // 讀回測試檔
  Serial.println("[+] 讀回 /TEST.TXT 內容:");
  f = SD.open("/TEST.TXT", FILE_READ);
  if (f) {
    while (f.available()) Serial.write(f.read());
    f.close();
  } else {
    Serial.println("[!] 讀取失敗");
  }

  Serial.println("");
  Serial.println("========================================");
  Serial.println("  測試完成,SD 卡可以正常使用!");
  Serial.println("========================================");
}

void loop() {
  // 不做事,結果已經在 setup 印完
  delay(5000);
}
