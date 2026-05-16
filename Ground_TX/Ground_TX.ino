// ============================================================
// 地面端發射器（給上面 Drone_FC_Gyro 配對使用）
//
// 硬體：完全沿用你原本的 6_Channel_Transmitter 電路
//   - Arduino Nano
//   - NRF24L01+PA/LNA: CE=D9, CSN=D10, SCK=D13, MOSI=D11, MISO=D12
//   - 搖桿 1: VRy=A0, VRx=A1
//   - 搖桿 2: VRy=A2, VRx=A3
//   - 開關 SW1=D0, SW2=D3
//
// 與原版差異：
//   - 加 INPUT_PULLUP 修正 D0/D3 浮接 bug（純軟體，不改電路）
//   - 把 aux1/aux2 反相，按下=1 較直覺
//   - 失聯時 TX 不關掉，飛機端會自己 failsafe
// ============================================================

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t PIPE_OUT    = 0xABCDABCD71LL;
const uint8_t  NRF_CHANNEL = 100;

RF24 radio(9, 10);   // CE, CSN

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

// 把搖桿讀值對應到 0~255，中間 = 128
int borderMap(int val, int lower, int middle, int upper, bool reverse) {
  val = constrain(val, lower, upper);
  if (val < middle)
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return reverse ? 255 - val : val;
}

void setup() {
  // 開關用 INPUT_PULLUP，按下對 GND → 讀到 0；放開 → 讀到 1
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
  // 通道對應（同你原版設定）
  data.roll     = borderMap(analogRead(A3), 0, 512, 1023, true );   // CH1
  data.pitch    = borderMap(analogRead(A0), 0, 512, 1023, true );   // CH2
  data.throttle = borderMap(analogRead(A2), 0, 340,  570, true );   // CH3 油門
  data.yaw      = borderMap(analogRead(A1), 0, 512, 1023, false);   // CH4

  // 開關：按下對 GND → digitalRead = LOW → 反相後 aux = 1
  data.aux1 = !digitalRead(0);   // CH5
  data.aux2 = !digitalRead(3);   // CH6

  radio.write(&data, sizeof(Signal));
  delay(20);   // 約 50Hz 更新
}
