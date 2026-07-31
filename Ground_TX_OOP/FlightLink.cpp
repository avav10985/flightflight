#include "FlightLink.h"

bool FlightLink::begin() {
  bool ok = radio_.begin();
  radio_.openWritingPipe(pipe_);
  radio_.setChannel(channel_);
  radio_.setAutoAck(true);
  radio_.enableAckPayload();
  radio_.setDataRate(RF24_250KBPS);
  // 2026-06-09 PA_HIGH 還是讓 3V3 軌 sag(LDO 撐不住 50mA 尖峰)
  // 降到 PA_MIN(~7mA 尖峰),確認對通邏輯先,距離 ~10m 夠桌面測試
  // 飛實機要長距離再加大電容 / 加獨立 LDO 後改回 HIGH 或 MAX
  radio_.setPALevel(RF24_PA_MIN);
  radio_.stopListening();
  return ok;
}

bool FlightLink::send(const Signal& sig) {
  bool ok = radio_.write(&sig, sizeof(Signal));
  if (ok) {
    txOk++;
    if (radio_.isAckPayloadAvailable()) {
      radio_.read(&tele_, sizeof(tele_));
      teleOK_     = true;
      lastTeleMs_ = millis();
    }
  } else {
    txFail++;
    teleOK_ = false;
  }
  return ok;
}
