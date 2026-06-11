#ifndef _HW_CONFIG_H_
#define _HW_CONFIG_H_

// ============================================================
// NES_Test 硬體設定 — flightflight 手把 V2-A(ESP32-S3)
// 顯示 / 聲音 / SD 腳位都沿用手把現有接線,完全不用改線。
// 唯一新需求:右搖桿 X 訊號接到 GPIO 10(本來就是 roll 預留線)
// ============================================================

#define FSROOT "/fs"

/* 聲音:MAX98357A 外部 I2S DAC(手把 V2-B 腳位) */
#define HW_AUDIO
#define HW_AUDIO_SAMPLERATE 22050
#define HW_AUDIO_EXTDAC
#define HW_AUDIO_EXTDAC_BCLK 11
#define HW_AUDIO_EXTDAC_WCLK 12
#define HW_AUDIO_EXTDAC_DOUT 13

/* 控制器:自訂實作(controller.cpp),不用範例的 HW_CONTROLLER_GPIO 巨集
 *   十字鍵 = 右搖桿(GPIO 4 上下 / GPIO 10 左右,ADC、軸向反相)
 *   A / B  = 右肩鈕 GPIO 9 / 左肩鈕 GPIO 8
 *   Start / Select = 按鍵叢電阻階梯 GPIO 7(OK / 返回)
 */
#define PIN_JOY_UD     4
#define PIN_JOY_LR    10
#define PIN_BTN_A      9
#define PIN_BTN_B      8
#define PIN_BTN_LADDER 7
#define PIN_AMP_SD    18   // MAX98357A shutdown,HIGH = 出聲

#endif /* _HW_CONFIG_H_ */
