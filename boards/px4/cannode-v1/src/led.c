/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file px4_cannode_led.c
 *
 * PX4ESC LED backend.
 */

#include <px4_platform_common/config.h>

#include <stdbool.h>
#include <nuttx/board.h>

#include "stm32.h"
#include "board_config.h"

#include <arch/board/board.h>

#include <systemlib/px4_macros.h>

/*
 * Ideally we'd be able to get these from up_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

static uint16_t g_ledmap[] = {
	GPIO_LED_GREEN,     // Indexed by BOARD_LED_GREEN
	GPIO_LED_YELLOW,    // Indexed by BOARD_LED_YELLOW
};

__EXPORT void led_init(void)
{
	/* Configure LED1-2 GPIOs for output */
	for (size_t l = 0; l < arraySize(g_ledmap); l++) {
		stm32_configgpio(g_ledmap[l]);
	}
}

__EXPORT void board_autoled_initialize(void)
{
	led_init();
}

static void phy_set_led(int led, bool state)
{
	/* Pull Up to switch on */
	stm32_gpiowrite(g_ledmap[led], state);
}

static bool phy_get_led(int led)
{

	return !stm32_gpioread(g_ledmap[led]);
}

__EXPORT void led_on(int led)
{
	phy_set_led(led, true);
}

__EXPORT void led_off(int led)
{
	phy_set_led(led, false);
}

__EXPORT void led_toggle(int led)
{

	phy_set_led(led, !phy_get_led(led));
}

static bool g_initialized;

// Nuttx Usages

__EXPORT void board_autoled_on(int led)
{
	switch (led) {
	default:
	case LED_STARTED:
	case LED_HEAPALLOCATE:
	case LED_IRQSENABLED:
		phy_set_led(BOARD_LED_GREEN, false);
		phy_set_led(BOARD_LED_YELLOW, false);
		break;

	case LED_STACKCREATED:
		phy_set_led(BOARD_LED_GREEN, true);
		phy_set_led(BOARD_LED_YELLOW, false);
		g_initialized = true;
		break;

	case LED_INIRQ:
	case LED_SIGNAL:
	case LED_ASSERTION:
	case LED_PANIC:
		phy_set_led(BOARD_LED_YELLOW, true);
		break;

	case LED_IDLE : /* IDLE */
		phy_set_led(BOARD_LED_GREEN, false);
		break;
	}
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/


__EXPORT void board_autoled_off(int led)
{
	switch (led) {
	default:
	case LED_STARTED:
	case LED_HEAPALLOCATE:
	case LED_IRQSENABLED:
	case LED_STACKCREATED:
		phy_set_led(BOARD_LED_GREEN, false);

	/* FALLTHROUGH */

	case LED_INIRQ:
	case LED_SIGNAL:
	case LED_ASSERTION:
	case LED_PANIC:
		phy_set_led(BOARD_LED_YELLOW, false);
		break;

	case LED_IDLE: /* IDLE */
		phy_set_led(BOARD_LED_GREEN, g_initialized);
		break;
	}
}
