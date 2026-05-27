// ============================================================
// 地面端發射器（四軸 Mode 2 佈局）
//
// 硬體：沿用原本的 6_Channel_Transmitter 電路
//   - Arduino Nano
//   - NRF24L01+PA/LNA: CE=D9, CSN=D10, SCK=D13, MOSI=D11, MISO=D12
//   - 搖桿 1（左）: 上下=A0, 左右=A1
//   - 搖桿 2（右）: 上下=A2, 左右=A3
//   - 開關 SW1=D0(aux1 武裝), SW2=D3(aux2 校正)
//
// Mode 2 佈局：
//   左搖桿：上下 = 油門 throttle、左右 = 偏航 yaw
//   右搖桿：上下 = 俯仰 pitch、 左右 = 翻滾 roll
//
// ★ 方向不對就改下面 REV_xxx 的 true/false
// ============================================================

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t PIPE_OUT    = 0xABCDABCD71LL;
const uint8_t  NRF_CHANNEL = 100;

RF24 radio(9, 10);   // CE, CSN

// ==== 方向反轉設定（測試後不對就改這裡）====
const bool REV_THROTTLE = true;   // 油門：往上推 throttle 要增加
const bool REV_YAW      = false;  // 左搖桿往右 = 機頭右轉
const bool REV_PITCH    = true;   // 右搖桿往上 = 前進
const bool REV_ROLL     = true;   // 右搖桿往右 = 右傾
// ============================================

struct Signal {
  byte throttle;
  byte pitch;
  byte roll;
  byte yaw;
  byte aux1;
  byte aux2;
};

Signal data;

void resetData() {
  data.throttle = 0;
  data.pitch = data.roll = data.yaw = 127;
  data.aux1 = data.aux2 = 0;
}

// 中心型控制（roll/pitch/yaw）：中間 = 128
int centerMap(int val, bool reverse) {
  val = constrain(val, 0, 1023);
  int out = map(val, 0, 1023, 0, 255);
  return reverse ? 255 - out : out;
}

// 油門（線性，非中心型）：0 ~ 255
int throttleMap(int val, bool reverse) {
  val = constrain(val, 0, 1023);
  int out = map(val, 0, 1023, 0, 255);
  return reverse ? 255 - out : out;
}

void setup() {
  pinMode(0, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  radio.begin();
  radio.openWritingPipe(PIPE_OUT);
  radio.setChannel(NRF_CHANNEL);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();

  resetData();
}

void loop() {
  // Mode 2 通道對應
  data.throttle = throttleMap(analogRead(A0), REV_THROTTLE);  // 左搖桿 上下
  data.yaw      = centerMap(analogRead(A1), REV_YAW);         // 左搖桿 左右
  data.pitch    = centerMap(analogRead(A2), REV_PITCH);       // 右搖桿 上下
  data.roll     = centerMap(analogRead(A3), REV_ROLL);        // 右搖桿 左右

  // 開關：按下對 GND → digitalRead = LOW → 反相後 = 1
  data.aux1 = !digitalRead(0);   // 武裝
  data.aux2 = !digitalRead(3);   // 校正

  radio.write(&data, sizeof(Signal));
  delay(20);   // 約 50Hz
}
