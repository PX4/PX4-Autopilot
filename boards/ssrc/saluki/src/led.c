/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file led.c
 *
 * Saluki LED backend.
 */

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>

#include "chip.h"
#include "mpfs_gpio.h"
#include "board_config.h"

#include <nuttx/board.h>
#include <arch/board/board.h>

static uint32_t g_leds[] = {
	GPIO_nLED_BLUE,
	GPIO_nLED_RED,
	GPIO_nSAFETY_SWITCH_LED_OUT,
	GPIO_nLED_GREEN,
};

/* LED_ACTIVITY == 1, LED_BOOTLOADER == 2 */
static bool g_led_state[3];

__EXPORT void led_init(void)
{
	for (size_t l = 0; l < (sizeof(g_leds) / sizeof(g_leds[0])); l++) {
		if (g_leds[l] != 0) {
			g_led_state[l] = false;
			mpfs_configgpio(g_leds[l]);
		}
	}
}

static void set_led(int led, bool state)
{
	if (g_leds[led] != 0) {
		g_led_state[led] = state;
		mpfs_gpiowrite(g_leds[led], state);
	}
}

static bool get_led(int led)
{
	if (g_leds[led] != 0) {
		return g_led_state[led];
	}

	return false;
}

__EXPORT void led_on(int led)
{
	set_led(led, true);
}

__EXPORT void led_off(int led)
{
	set_led(led, false);
}

__EXPORT void led_toggle(int led)
{
	set_led(led, !get_led(led));
}
