#ifndef _HW_CONFIG_H_
#define _HW_CONFIG_H_

// ============================================================
// NES_Test 硬體設定 — flightflight 手把 V2-A(ESP32-S3)
// 顯示 / 聲音 / SD 腳位都沿用手把現有接線,完全不用改線。
// 全部用現有接線,不用改任何線
// ============================================================

#define FSROOT "/fs"

/* 聲音:MAX98357A 外部 I2S DAC(手把 V2-B 腳位)
 * ⚠ 暫時關閉做切分測試:畫面出現後消失疑似崩潰,聲音的
 *   legacy I2S 驅動是頭號嫌疑。畫面穩了再開回來修聲音 */
// #define HW_AUDIO
#define HW_AUDIO_SAMPLERATE 22050
#define HW_AUDIO_EXTDAC
#define HW_AUDIO_EXTDAC_BCLK 11
#define HW_AUDIO_EXTDAC_WCLK 12
#define HW_AUDIO_EXTDAC_DOUT 13

/* 控制器:自訂實作(controller.cpp),不用範例的 HW_CONTROLLER_GPIO 巨集
 *   上下 = 右搖桿 Y(GPIO 4,有彈簧回中)
 *   左右 = 左搖桿 X(GPIO 2,有彈簧回中;左搖桿 Y 是拆彈簧油門不適合)
 *   A / B  = 右肩鈕 GPIO 9 / 左肩鈕 GPIO 8
 *   Start / Select = 按鍵叢電阻階梯 GPIO 7(OK / 返回)
 */
#define PIN_JOY_UD     4
#define PIN_JOY_LR     2
#define PIN_BTN_A      9
#define PIN_BTN_B      8
#define PIN_BTN_LADDER 7
#define PIN_AMP_SD    18   // MAX98357A shutdown,HIGH = 出聲

#endif /* _HW_CONFIG_H_ */
