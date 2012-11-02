/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file px4io.c
 * Top-level logic for the PX4IO module.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <debug.h>
#include <stdlib.h>
#include <errno.h>

#include <nuttx/clock.h>

#include <arch/board/up_boardinitialize.h>
#include <arch/board/drv_gpio.h>
#include <arch/board/drv_ppm_input.h>
#include <drivers/drv_hrt.h>

#include "px4io.h"

__EXPORT int user_start(int argc, char *argv[]);

struct sys_state_s 	system_state;
int			gpio_fd;

static const char cursor[] = {'|', '/', '-', '\\'};

static const char *rc_input_mq_name = "rc_input";

static struct hrt_call timer_tick_call;
volatile int timers[TIMER_NUM_TIMERS];
static void timer_tick(void *arg);

int user_start(int argc, char *argv[])
{
	int     cycle = 0;
	bool	heartbeat = false;
	bool	failsafe = false;

	/* Do board init */
	(void)up_boardinitialize();

	/* print some startup info */
	lib_lowprintf("\nPX4IO: starting\n");
	struct mallinfo minfo = mallinfo();
	lib_lowprintf("free %u largest %u\n", minfo.mxordblk, minfo.fordblks);

	/* start the software timer service */
	hrt_call_every(&timer_tick_call, 1000, 1000, timer_tick, NULL);

	/* Open the GPIO driver so we can do LEDs and the like. */
	gpio_fd = open("/dev/gpio", 0);
	ASSERTCODE((gpio_fd >= 0), A_GPIO_OPEN_FAIL);

	/* default all the LEDs to off while we start */
	LED_AMBER(heartbeat);
	LED_BLUE(failsafe);
	LED_SAFETY(false);

	/* turn on servo power */
	POWER_SERVO(true);

	/* configure the PPM input driver */
	ppm_input_init(rc_input_mq_name);

	/* start the mixer */
	mixer_init(rc_input_mq_name);

	/* start the safety switch handler */
	safety_init();

	/* init the FMU link */
	comms_init();

	/* set up some timers for the main loop */
	timers[TIMER_BLINK_AMBER] = 250;	/* heartbeat blink @ 2Hz */
	timers[TIMER_STATUS_PRINT] = 1000;	/* print status message @ 1Hz */

	/*
	 * Main loop servicing communication with FMU
	 */
	while(true) {

		/* check for communication from FMU, send updates */
		comms_check();

		/* blink the heartbeat LED */
		if (timers[TIMER_BLINK_AMBER] == 0) {
			timers[TIMER_BLINK_AMBER] = 250;
			LED_AMBER((heartbeat = !heartbeat));
		}

		/* blink the failsafe LED if we don't have FMU input */
		if (!system_state.mixer_use_fmu) {
			if (timers[TIMER_BLINK_BLUE] == 0) {
				timers[TIMER_BLINK_BLUE] = 125;
				LED_BLUE((failsafe = !failsafe));
			}
		} else {
			LED_BLUE((failsafe = false));
		}
		
		/* print some simple status */
		if (timers[TIMER_STATUS_PRINT] == 0) {
			timers[TIMER_STATUS_PRINT] = 1000;
			lib_lowprintf("%c %s | %s | %s | C=%d    \r",
				cursor[cycle++ & 3],
				(system_state.armed         ? "ARMED"  : "SAFE"),
				(system_state.rc_channels   ? "RC OK"  : "NO RC"),
				(system_state.mixer_use_fmu ? "FMU OK" : "NO FMU"),
				system_state.rc_channels
			);
		}

	}

	/* Should never reach here */
	return ERROR;
}

static void
timer_tick(void *arg)
{
	for (unsigned i = 0; i < TIMER_NUM_TIMERS; i++)
		if (timers[i] > 0)
			timers[i]--;
}
