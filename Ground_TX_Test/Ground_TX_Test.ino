// ============================================================
// 手把輸入測試（不含 NRF24 / LCD）
// 燒進地面站 ESP32，序列埠(115200)看所有輸入讀值，邊接線邊驗證。
// 接線對照見 手把接線總表.md。
//
// 桌上測試只要 USB 供電即可（不用 buck/電池）。
// 接一組驗一組：搖桿掃 0~4095、模式 0~8、按鈕認對、肩鍵 0/1。
// 門檻(readSwitch3 / readMenuBtn)和正式程式一致，這裡先調好，正式版照用。
// ============================================================

#define J_THROTTLE   34
#define J_PITCH      32
#define J_ROLL       33
#define SW_A         36
#define SW_B         39
#define MENU_BTN     35
#define SHOULDER_L   13
#define SHOULDER_R   14

enum { BTN_NONE, BTN_PLUS, BTN_MINUS, BTN_OK, BTN_BACK };
const char* BTN_NAME[] = { "NONE ", "PLUS ", "MINUS", "OK   ", "BACK " };

byte readSwitch3(int pin) {
  int v = analogRead(pin);
  if (v > 2730) return 0;   // 上
  if (v < 1365) return 2;   // 下
  return 1;                  // 中（開路分壓中點）
}

int readMenuBtn() {
  int v = analogRead(MENU_BTN);
  if (v > 3250) return BTN_PLUS;
  if (v > 2415) return BTN_MINUS;
  if (v > 1665) return BTN_OK;
  if (v > 640)  return BTN_BACK;
  return BTN_NONE;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(SHOULDER_L, INPUT_PULLUP);
  pinMode(SHOULDER_R, INPUT_PULLUP);
  Serial.println("\n=== 手把輸入測試 ===");
}

void loop() {
  int thr = analogRead(J_THROTTLE);
  int pit = analogRead(J_PITCH);
  int rol = analogRead(J_ROLL);
  byte a  = readSwitch3(SW_A);
  byte b  = readSwitch3(SW_B);
  byte mode = a * 3 + b;
  int rawBtn = analogRead(MENU_BTN);
  int btn = readMenuBtn();
  bool yawL = (digitalRead(SHOULDER_L) == LOW);
  bool yawR = (digitalRead(SHOULDER_R) == LOW);

  Serial.printf("Thr%4d Pit%4d Rol%4d | A%d B%d mode=%d | btn:%s(%4d) | yawL%d yawR%d\n",
                thr, pit, rol, a, b, mode, BTN_NAME[btn], rawBtn, yawL, yawR);
  delay(300);
}
