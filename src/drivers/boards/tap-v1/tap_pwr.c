/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file tap-v1_pwr.c
 *
 * Board-specific SPI functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>

#include "up_arch.h"
#include "board_config.h"
#include <stm32_pwr.h>

extern void led_on(int led);
extern void led_off(int led);

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static int board_button_irq(int irq, FAR void *context)
{
	static struct timespec time_down;

	if (board_pwr_button_down()) {

		led_on(BOARD_LED_RED);

		clock_gettime(CLOCK_REALTIME, &time_down);

	} else {

		led_off(BOARD_LED_RED);

		struct timespec now;

		clock_gettime(CLOCK_REALTIME, &now);

		uint64_t tdown_ms = time_down.tv_sec * 1000 + time_down.tv_nsec / 1000000;

		uint64_t tnow_ms  = now.tv_sec * 1000 + now.tv_nsec / 1000000;

		if (tdown_ms != 0 && (tnow_ms - tdown_ms) >= MS_PWR_BUTTON_DOWN) {

			led_on(BOARD_LED_BLUE);

			up_mdelay(200);
			stm32_pwr_enablebkp(true);
			/* XXX wow, this is evil - write a magic number into backup register zero */
			*(uint32_t *)0x40002850 = 0xdeaddead;
			stm32_pwr_enablebkp(false);
			up_mdelay(50);
			up_systemreset();

			while (1);
		}
	}

	return OK;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: board_pwr_init()
 *
 * Description:
 *   Called to configure power control for the tap-v1 board.
 *
 * Input Parameters:
 *   stage- 0 for boot, 1 for board init
 *
 ************************************************************************************/

void board_pwr_init(int stage)
{
	if (stage == 0) {
		stm32_configgpio(POWER_ON_GPIO);
		stm32_configgpio(KEY_AD_GPIO);
	}

	if (stage == 1) {
		stm32_gpiosetevent(KEY_AD_GPIO, true, true, true, board_button_irq);
	}
}

/****************************************************************************
 * Name: board_pwr_button_down
 *
 * Description:
 *   Called to Read the logical state of the active low power button.
 *
 ****************************************************************************/

bool board_pwr_button_down(void)
{
	return 0 == stm32_gpioread(KEY_AD_GPIO);
}

/****************************************************************************
 * Name: board_pwr
 *
 * Description:
 *   Called to turn on or off the TAP
 *
 ****************************************************************************/

__EXPORT bool px4_board_pwr(bool on_not_off)
{
	if (on_not_off) {
		stm32_configgpio(POWER_ON_GPIO);

	} else {

		stm32_configgpio(POWER_OFF_GPIO);
	}

	return true;
}
