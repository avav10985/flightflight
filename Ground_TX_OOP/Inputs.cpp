#include "Inputs.h"

bool Inputs::fillSignal(Signal& s, uint8_t mode) {
  s.throttle = centerMap(analogReadAvg(J_THROTTLE), REV_THROTTLE);
  s.pitch    = centerMap(analogReadAvg(J_PITCH),    REV_PITCH);
  s.roll     = centerMap(analogReadAvg(J_ROLL),     REV_ROLL);

  // 中位死區:搖桿放中間時殘餘 ±幾 的抖動歸零,送出穩定中立 127
  if (abs((int)s.pitch - 127) <= 3) s.pitch = 127;
  if (abs((int)s.roll  - 127) <= 3) s.roll  = 127;
  if (s.throttle <= 4) s.throttle = 0;   // 油門到底吸成 0,怠速穩定

  // yaw 由肩鍵：L→42、R→212、都沒按/同時按→127（對應 ±60°/s）
  // mode 10 例外:左肩鈕是語音 PTT,yaw 固定中立(語音 yaw 之後再說)
  if (mode == 10) {
    s.yaw = 127;
  } else {
    bool yawL = (digitalRead(SHOULDER_L) == LOW);
    bool yawR = (digitalRead(SHOULDER_R) == LOW);
    s.yaw = (yawL && !yawR) ? 42 : (yawR && !yawL) ? 212 : 127;
  }

  s.mode = mode;

  // 進入 mode 0 → 送一次校準旗標（持續 600ms 確保飛機收到 0→1 邊緣）
  bool enteredMode0 = false;
  if (mode == 0 && lastMode_ != 0) {
    calHoldUntil_ = millis() + 600;
    enteredMode0  = true;   // 呼叫端 triggerCalPrompt("進場自動校準", 2400)
  }
  lastMode_ = mode;
  s.flags = (millis() < calHoldUntil_) ? 0x01 : 0x00;
  return enteredMode0;
}
