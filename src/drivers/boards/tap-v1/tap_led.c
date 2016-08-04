/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *         Author: David Sidrane <david_s5@nscdg.com>
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
 * @file tap-v1_led.c
 *
 * TAP_V1 LED backend.
 */

#include <px4_config.h>

#include <stdbool.h>

#include "stm32.h"
#include "board_config.h"

#include <arch/board/board.h>

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

__EXPORT void led_init(void)
{
	/* Configure LED1-2 GPIOs for output */

	stm32_configgpio(GPIO_BLUE_LED);
	stm32_configgpio(GPIO_RED_LED);
}

__EXPORT void led_on(int led)
{
	if (led == 0) {
		/* Pull down to switch on */
		stm32_gpiowrite(GPIO_BLUE_LED, false);
	}

	if (led == 1) {
		/* Pull down to switch on */
		stm32_gpiowrite(GPIO_RED_LED, false);
	}
}

__EXPORT void led_off(int led)
{
	if (led == 0) {
		/* Pull up to switch off */
		stm32_gpiowrite(GPIO_BLUE_LED, true);
	}

	if (led == 1) {
		/* Pull up to switch off */
		stm32_gpiowrite(GPIO_RED_LED, true);
	}
}

__EXPORT void led_toggle(int led)
{
	if (led == 0) {
		stm32_gpiowrite(GPIO_BLUE_LED, !stm32_gpioread(GPIO_BLUE_LED));
	}

	if (led == 1) {
		stm32_gpiowrite(GPIO_RED_LED, !stm32_gpioread(GPIO_RED_LED));
	}
}
