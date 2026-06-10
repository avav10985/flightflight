// ============================================================
// 6_Channel_Transmitter(舊手把,Arduino Nano)
//
// 2026-06-09 大改:跟新 Drone_FC_Full(commit 2a12a85)對通
//   - PIPE 0xABCDABCD71 → 0x4E6F9C2D5B
//   - CHANNEL 100 → 88
//   - Signal struct 加 mode / flags / paramID / paramVal(原本 6 bytes → 11 bytes)
//   - AutoAck false → true(新 drone 要 ACK)
//   - PA_MAX → PA_MIN(配對,LDO 撐不住高功率)
//   - 加 Serial debug(Nano 有 CH340,Serial 沒 USB CDC 問題)
//
// 硬體:
//   Arduino Nano + NRF24L01+PA/LNA (CE=D9, CSN=D10)
//   左搖桿:A2=油門、A1=偏航
//   右搖桿:A0=俯仰、A3=翻滾
//   開關:D0=aux1(mode 切換)、D3=aux2(校準觸發)
// ============================================================

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// 必須跟 Drone_FC_Full 一致(commit ae972df 換新)
const uint64_t pipeOut     = 0x4E6F9C2D5BLL;
const uint8_t  NRF_CHANNEL = 88;

RF24 radio(9, 10);   // CE, CSN

// 跟 Drone_FC_Full 的 Signal struct 必須完全一致(byte 順序 + 大小 = 11 bytes)
struct Signal {
  byte throttle, pitch, roll, yaw;
  byte mode;        // 兩位數編碼:0=安全/校準、1=手動自穩(目前唯一可飛)、2=GPS...
  byte flags;       // bit0=完整校準觸發
  byte paramID;     // PID 參數調整(這台不調,固定 0)
  float paramVal;   // 4 bytes 浮點數
};

Signal data;

void ResetData() {
  data.throttle = 0;
  data.pitch = 127;
  data.roll  = 127;
  data.yaw   = 127;
  data.mode  = 0;
  data.flags = 0;
  data.paramID = 0;
  data.paramVal = 0;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("=== 舊手把(Arduino Nano)啟動 ==="));

  pinMode(0, INPUT_PULLUP);   // aux1 開關(對 GND)
  pinMode(3, INPUT_PULLUP);   // aux2 開關(對 GND)

  bool nrfOK = radio.begin();
  Serial.print(F("[*] NRF24 begin: "));
  Serial.println(nrfOK ? F("OK") : F("FAIL"));
  Serial.print(F("[*] NRF24 chip connected: "));
  Serial.println(radio.isChipConnected() ? F("YES") : F("NO"));

  radio.openWritingPipe(pipeOut);
  radio.setChannel(NRF_CHANNEL);
  radio.setAutoAck(true);          // 新 drone 用 ACK
  radio.enableAckPayload();        // 開啟 ACK payload(收 telemetry,這台不用但要 enable)
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);   // 跟 drone 配對,先 MIN 測對通
  radio.stopListening();

  ResetData();
  Serial.println(F("[+] 就緒,開始發送"));
}

int Border_Map(int val, int lower, int middle, int upper, bool reverse) {
  val = constrain(val, lower, upper);
  if (val < middle) val = map(val, lower, middle, 0,   128);
  else              val = map(val, middle, upper, 128, 255);
  return (reverse ? 255 - val : val);
}

void loop() {
  data.roll     = Border_Map(analogRead(A3), 0, 512, 1023, true);
  data.pitch    = Border_Map(analogRead(A0), 0, 512, 1023, true);
  data.throttle = Border_Map(analogRead(A2), 0, 340, 570,  true);
  data.yaw      = Border_Map(analogRead(A1), 0, 512, 1023, false);

  // 開關映射到新 mode/flags:
  // aux1 (D0) 按下對 GND → !digitalRead = 1 → mode 1(可飛)
  // 沒按   → !digitalRead = 0 → mode 0(安全 / 校準)
  data.mode  = (!digitalRead(0)) ? 1 : 0;
  data.flags = (!digitalRead(3)) ? 1 : 0;   // aux2 按下 = 觸發完整校準

  bool ok = radio.write(&data, sizeof(Signal));

  // Debug:每秒印一次 TX 結果 + 數值
  static uint32_t txOk = 0, txFail = 0, lastStat = 0;
  if (ok) txOk++; else txFail++;
  if (millis() - lastStat > 1000) {
    lastStat = millis();
    Serial.print(F("[NRF] ok="));
    Serial.print(txOk);
    Serial.print(F(" fail="));
    Serial.print(txFail);
    Serial.print(F(" | mode="));
    Serial.print(data.mode);
    Serial.print(F(" thr="));
    Serial.print(data.throttle);
    Serial.print(F(" P="));
    Serial.print(data.pitch);
    Serial.print(F(" R="));
    Serial.print(data.roll);
    Serial.print(F(" Y="));
    Serial.println(data.yaw);
    txOk = 0;
    txFail = 0;
  }

  delay(20);   // ~ 50Hz
}
