// ============================================================
// NRF24_to_PWM —— B 案「Nano 對照組」的接收橋接
//
// 用途:把手把的 nRF24 封包轉成標準 RC 的 servo PWM,餵給
//       Brokking YMFC-AL 飛控(Arduino Nano #2)。
//
// 由來:改自 `不可上傳/舊6通道遙控/6_Channel_Receiver.ino`(原始版)。
//       原始版用 pipe 0xABCDABCD71 / ch100 / 6-byte struct / AutoAck false,
//       但舊 Nano 手把已在 2026-06-09 改成對 Drone_FC_Full 的新協定,
//       兩邊對不上收不到東西。這支把 RX 對齊新協定。
//
// 好處:同一隻手把可以飛真飛機(Drone_FC_Full)也可以飛這台 Nano 測試機,
//       切換不用重燒手把。V2 的 ESP32-S3 手把協定相同,也能直接驅動。
//
// 硬體(Arduino Nano #1 = 接收橋接):
//   NRF24L01+PA/LNA : CE=D9, CSN=D10, SCK=D13, MOSI=D11, MISO=D12
//   PWM 輸出        : D2=roll, D3=pitch, D4=throttle, D5=yaw, D6=mode(AUX)
//   供電            : 5V(NRF24 的 VCC 要 3V3!模組若無穩壓要外接)
//
// 接到 Brokking 飛控(Arduino Nano #2)的 D8~D11(PCINT0~3)。
// 通道順序 ch1=roll / ch2=pitch / ch3=throttle / ch4=yaw 就是 Brokking 的
// 標準慣例,但實際對應由他的 YMFC-AL_setup 程式自動偵測並寫進 EEPROM,
// 所以接線順序接錯也能靠 setup 校正。
//
// 註:這裡輸出的是 50Hz 標準 servo PWM(遙控器接收機都是這個規格),
//     跟「飛控送給 ESC 的 250Hz」是兩條不同的鏈路,別搞混。
// ============================================================

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

// ---- 必須跟手把/飛控完全一致(見 Drone_FC_Full.ino:100) ----
const uint64_t PIPE_IN     = 0x4E6F9C2D5BLL;
const uint8_t  NRF_CHANNEL = 88;

RF24 radio(9, 10);   // CE, CSN

// 必須跟手把的 Signal struct 完全一致(byte 順序 + 大小 = 11 bytes)
struct Signal {
  byte  throttle, pitch, roll, yaw;
  byte  mode;       // SW_A×10 + SW_B
  byte  flags;      // bit0=完整校準觸發(這台用不到)
  byte  paramID;    // PID 調參(這台用不到)
  float paramVal;
};

Signal data;

Servo chRoll, chPitch, chThrottle, chYaw, chMode;

unsigned long lastRecvTime = 0;

// 失聯安全值:油門切掉、三軸置中
void resetData() {
  data.throttle = 0;
  data.pitch    = 127;
  data.roll     = 127;
  data.yaw      = 127;
  data.mode     = 0;
  data.flags    = 0;
  data.paramID  = 0;
  data.paramVal = 0;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("=== NRF24_to_PWM 橋接啟動 ==="));

  resetData();

  // 先把 PWM 拉到安全值再 attach,避免上電瞬間亂送
  chRoll.attach(2);
  chPitch.attach(3);
  chThrottle.attach(4);
  chYaw.attach(5);
  chMode.attach(6);

  bool ok = radio.begin();
  Serial.print(F("[*] NRF24 begin: "));
  Serial.println(ok ? F("OK") : F("FAIL"));
  Serial.print(F("[*] chip connected: "));
  Serial.println(radio.isChipConnected() ? F("YES") : F("NO  <-- 檢查接線/供電"));

  radio.openReadingPipe(1, PIPE_IN);
  radio.setChannel(NRF_CHANNEL);
  radio.setAutoAck(true);            // 手把端是 true,必須配對
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);     // 跟 Drone_FC_Full 一致
  radio.startListening();

  Serial.println(F("[*] 監聽中(pipe 0x4E6F9C2D5B / ch88 / 250kbps)"));
}

void loop() {
  while (radio.available()) {
    radio.read(&data, sizeof(Signal));
    lastRecvTime = millis();
  }

  // 失聯 1 秒 → 回安全值(跟飛控端 failsafe 同一個門檻)
  if (millis() - lastRecvTime > 1000) resetData();

  chRoll.writeMicroseconds(     map(data.roll,     0, 255, 1000, 2000));
  chPitch.writeMicroseconds(    map(data.pitch,    0, 255, 1000, 2000));
  chThrottle.writeMicroseconds( map(data.throttle, 0, 255, 1000, 2000));
  chYaw.writeMicroseconds(      map(data.yaw,      0, 255, 1000, 2000));
  chMode.writeMicroseconds(     map(data.mode,     0,  22, 1000, 2000));

  // 1Hz 狀態列:看得到收沒收到、搖桿有沒有動
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg >= 1000) {
    lastDbg = millis();
    bool linked = (millis() - lastRecvTime <= 1000);
    Serial.print(linked ? F("LINK ") : F("---- "));
    Serial.print(F("T")); Serial.print(data.throttle);
    Serial.print(F(" R")); Serial.print(data.roll);
    Serial.print(F(" P")); Serial.print(data.pitch);
    Serial.print(F(" Y")); Serial.print(data.yaw);
    Serial.print(F(" M")); Serial.println(data.mode);
  }
}
