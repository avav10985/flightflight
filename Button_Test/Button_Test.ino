// ============================================================
// Button_Test — 所有手把輸入元件測試
//
// 測試對象(全部 V2-A 輸入):
//   SW_A 三段開關  GPIO 5(類比分壓,上/中/下)
//   SW_B 三段開關  GPIO 6(類比分壓,上/中/下)
//   選單階梯按鈕   GPIO 7(類比階梯 + / − / OK / 返回)
//   肩鍵 L        GPIO 8(數位 + 內部上拉)
//   肩鍵 R        GPIO 9(數位 + 內部上拉)
//
// === 用法 ===
// 1. 燒上去,開 Serial Monitor (115200)
// 2. 撥開關 / 按按鈕 / 按肩鍵
// 3. **觀察原始 ADC 值**,以此校準門檻
//
// 例如:按「-」如果顯示 ADC = 1900(落在 OK 範圍 1665~2415),
//      代表 OK 跟「-」的門檻寫錯了 → 看實際值重設
//
// === Serial 輸出格式 ===
// SW_A=中(2048)  SW_B=上(4095)  選單=無( 50)  肩L=放 肩R=放  mode=01
// SW_A=下( 100)  SW_B=中(2048)  選單=+(3500)  肩L=按 肩R=放  mode=20
// ============================================================

#define SW_A         5
#define SW_B         6
#define MENU_BTN     7
#define SHOULDER_L   8
#define SHOULDER_R   9

const char* SW_NAME[3] = { "上", "中", "下" };

byte readSwitch3(int pin) {
  int v = analogRead(pin);
  if (v > 2730) return 2;   // 下
  if (v < 1365) return 0;   // 上
  return 1;                  // 中
}

const char* menuName(int v) {
  if (v > 3250) return "+   ";
  if (v > 2415) return "-   ";
  if (v > 1665) return "OK  ";
  if (v > 640)  return "返回";
  return "無  ";
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("        手把所有輸入元件測試");
  Serial.println("========================================");
  Serial.println("撥 SW_A / SW_B、按 + - OK 返回、按肩鍵 L/R");
  Serial.println("Serial 會顯示原始 ADC 值,用來校準門檻");
  Serial.println("");
  Serial.println("現有門檻(Ground_TX_ESP32 用的):");
  Serial.println("  SW_A / SW_B:>2730=下、<1365=上、其他=中");
  Serial.println("  選單:>3250=+、>2415=-、>1665=OK、>640=返回、其他=無");
  Serial.println("");
  Serial.println("如果按下去的按鈕被判成別的,就照實際 ADC 值改門檻");
  Serial.println("");

  neopixelWrite(48, 0, 0, 0);

  pinMode(SHOULDER_L, INPUT_PULLUP);
  pinMode(SHOULDER_R, INPUT_PULLUP);
  analogReadResolution(12);
}

void loop() {
  int swA_raw  = analogRead(SW_A);
  int swB_raw  = analogRead(SW_B);
  int menu_raw = analogRead(MENU_BTN);
  byte swA     = readSwitch3(SW_A);
  byte swB     = readSwitch3(SW_B);
  bool shldL   = (digitalRead(SHOULDER_L) == LOW);
  bool shldR   = (digitalRead(SHOULDER_R) == LOW);

  Serial.printf("SW_A=%s(%4d)  SW_B=%s(%4d)  選單=%s(%4d)  肩L=%s 肩R=%s  mode=%02d\n",
                SW_NAME[swA], swA_raw,
                SW_NAME[swB], swB_raw,
                menuName(menu_raw), menu_raw,
                shldL ? "按" : "放",
                shldR ? "按" : "放",
                swA * 10 + swB);

  delay(200);
}
