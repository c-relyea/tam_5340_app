/*
 * Copyright (c) 2024 Waterfront Collective Inc.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <nrfx_clock.h>

#include "led.h"
#include "button_handler.h"
#include "button_assignments.h"
#include "channel_assignment.h"
#include "nrf5340_audio_dk.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tam_board, CONFIG_MODULE_NRF5340_AUDIO_DK_LOG_LEVEL);

static int channel_assign_check(void)
{
#if (CONFIG_AUDIO_DEV == HEADSET) && CONFIG_AUDIO_HEADSET_CHANNEL_RUNTIME
	int ret;
	bool pressed;

	ret = button_pressed(BUTTON_VOLUME_DOWN, &pressed);
	if (ret) {
		return ret;
	}
	if (pressed) {
		channel_assignment_set(AUDIO_CH_L);
		return 0;
	}

	ret = button_pressed(BUTTON_VOLUME_UP, &pressed);
	if (ret) {
		return ret;
	}
	if (pressed) {
		channel_assignment_set(AUDIO_CH_R);
		return 0;
	}
#endif
	return 0;
}

int nrf5340_audio_dk_init(void)
{
	int ret;

	ret = led_init();
	if (ret) {
		LOG_ERR("Failed to initialize LED module");
		return ret;
	}

	ret = button_handler_init();
	if (ret) {
		LOG_ERR("Failed to initialize button handler");
		return ret;
	}

	ret = channel_assign_check();
	if (ret) {
		LOG_ERR("Failed to get channel assignment");
		return ret;
	}

	/* Enable 128 MHz clock for cpu_app */
	ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);
	ret -= NRFX_ERROR_BASE_NUM;
	if (ret) {
		return ret;
	}

	return 0;
}
