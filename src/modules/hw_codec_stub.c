/*
 * Stub implementation of hw_codec when no hardware codec is present.
 * Used when CONFIG_NRF5340_AUDIO_CS47L63_DRIVER is disabled (e.g., Meow Sense Tag).
 */

#include "hw_codec.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/logging/log.h>

#include "zbus_common.h"

LOG_MODULE_REGISTER(hw_codec_stub, LOG_LEVEL_INF);

ZBUS_SUBSCRIBER_DEFINE(volume_evt_sub, CONFIG_VOLUME_MSG_SUB_QUEUE_SIZE);

int hw_codec_volume_set(uint8_t set_val)
{
	return 0;
}

int hw_codec_volume_adjust(int8_t adjustment)
{
	return 0;
}

int hw_codec_volume_decrease(void)
{
	return 0;
}

int hw_codec_volume_increase(void)
{
	return 0;
}

int hw_codec_volume_mute(void)
{
	return 0;
}

int hw_codec_volume_unmute(void)
{
	return 0;
}

int hw_codec_default_conf_enable(void)
{
	LOG_INF("HW codec stub: no codec present, skipping default config");
	return 0;
}

int hw_codec_soft_reset(void)
{
	return 0;
}

int hw_codec_init(void)
{
	LOG_INF("HW codec stub: no codec present, skipping init");
	return 0;
}

int hw_codec_stop_audio(void)
{
	return 0;
}

int hw_codec_set_audio_mode(enum audio_mode mode)
{
	return 0;
}

enum audio_mode hw_codec_get_audio_mode(void)
{
	return AUDIO_MODE_NORMAL;
}
