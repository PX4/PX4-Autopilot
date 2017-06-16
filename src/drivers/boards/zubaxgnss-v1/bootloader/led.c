/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *   Author: David Sidrane<david_s5@nscdg.com>
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
#include <px4_config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/board/board.h>
#include "chip/stm32_tim.h"


#include "led.h"
#include "timer.h"
#include "board_config.h"

const int LED_TICK = 10;
static bool led_a;
static bool led_b;
static bool led_c;
static int period;

static void led_process(bl_timer_id id, void *context)
{
	static int dc = 0;
	static bool toggle = true;
	dc  += LED_TICK;

	if (dc >= period) {
		dc = 0;
		toggle ^= true;
	}

	stm32_gpiowrite(GPIO_LED_INFO, led_a ? toggle : false);
	stm32_gpiowrite(GPIO_LED_CAN1, led_b ? toggle : false);
	stm32_gpiowrite(GPIO_LED_CAN2, led_c ? toggle : false);

}


void set_leds(bool a, bool b, bool c, int freqs)
{
	static bl_timer_id  tid = (bl_timer_id) - 1;

	if (tid == (bl_timer_id) - 1) {

		bl_timer_cb_t p = null_cb;
		p.cb = led_process;

		tid = timer_allocate(modeRepeating | modeStarted, LED_TICK, &p);
	}

	led_a = a;
	led_b = b;
	led_c = c;

	if (freqs == 0) {
		timer_stop(tid);

	} else {
		period = 1000 / freqs;
		timer_restart(tid, LED_TICK);
	}

}
