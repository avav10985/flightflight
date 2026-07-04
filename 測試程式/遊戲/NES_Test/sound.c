/*
 * NES 聲音輸出 — i2s_std 新驅動版(flightflight 手把,2026-06-12)
 *
 * 原範例用 legacy driver/i2s.h,在 arduino-esp32 3.x(IDF 5.5)上
 * 啟動即崩潰(切分測試確認)。改用跟 Ground_TX 音樂播放同一套
 * i2s_std 驅動,行為照抄原版:每幀由 do_audio_frame() 拉
 * audio_callback 的 16-bit mono,複製成雙聲道後阻塞寫入。
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/i2s_std.h>

#include <nes/nes.h>

#include "hw_config.h"

#if defined(HW_AUDIO)

#define DEFAULT_FRAGSIZE 64

static void (*audio_callback)(void *buffer, int length) = NULL;
static int16_t *audio_frame;
static i2s_chan_handle_t sndTx = NULL;

int osd_init_sound()
{
	audio_frame = NOFRENDO_MALLOC(4 * DEFAULT_FRAGSIZE);

	i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
	cc.dma_desc_num = 7;
	cc.dma_frame_num = 256;
	i2s_new_channel(&cc, &sndTx, NULL);

	i2s_std_config_t sc = {
		.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(HW_AUDIO_SAMPLERATE),
		.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
		                                                I2S_SLOT_MODE_STEREO),
		.gpio_cfg = {
			.mclk = I2S_GPIO_UNUSED,
			.bclk = (gpio_num_t)HW_AUDIO_EXTDAC_BCLK,
			.ws   = (gpio_num_t)HW_AUDIO_EXTDAC_WCLK,
			.dout = (gpio_num_t)HW_AUDIO_EXTDAC_DOUT,
			.din  = I2S_GPIO_UNUSED,
		},
	};
	i2s_channel_init_std_mode(sndTx, &sc);
	i2s_channel_enable(sndTx);

	audio_callback = NULL;
	return 0;
}

void osd_stopsound()
{
	audio_callback = NULL;
}

void do_audio_frame()
{
	if (audio_callback == NULL || sndTx == NULL)
		return;
	int left = HW_AUDIO_SAMPLERATE / NES_REFRESH_RATE;
	while (left)
	{
		int n = DEFAULT_FRAGSIZE;
		if (n > left)
			n = left;
		audio_callback(audio_frame, n);

		/* 16-bit mono → 16-bit stereo(L=R,音量 >>2 同原版 EXTDAC)*/
		int16_t *mono_ptr = audio_frame + n;
		int16_t *stereo_ptr = audio_frame + n + n;
		int i = n;
		while (i--)
		{
			int16_t a = (int16_t)(*(--mono_ptr) >> 2);
			*(--stereo_ptr) = a;
			*(--stereo_ptr) = a;
		}

		size_t written = 0;
		i2s_channel_write(sndTx, (const char *)audio_frame, 4 * n, &written, portMAX_DELAY);
		left -= written / 4;
	}
}

void osd_setsound(void (*playfunc)(void *buffer, int length))
{
	audio_callback = playfunc;
}

void osd_getsoundinfo(sndinfo_t *info)
{
	info->sample_rate = HW_AUDIO_SAMPLERATE;
	info->bps = 16;
}

#else /* !HW_AUDIO:無聲 stub */

void do_audio_frame() {}
int osd_init_sound() { return 0; }
void osd_stopsound() {}
void osd_setsound(void (*playfunc)(void *buffer, int length)) {}
void osd_getsoundinfo(sndinfo_t *info)
{
	info->sample_rate = 22050;
	info->bps = 16;
}

#endif
