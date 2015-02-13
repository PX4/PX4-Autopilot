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
 * @file mavstation.c
 * Top-level logic for the mavstation module.
 */

#include <nuttx/config.h>

#include <stdio.h>	// required for task_create
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <time.h>
#include <signal.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <systemlib/perf_counter.h>

#include <stm32_uart.h>

#include "sysstate.h"
#include "gpio.h"
#include "appdebug.h"

__EXPORT int user_start(int argc, char *argv[]);

struct sys_state_s system_state;

#ifdef CONFIG_ARCH_DMA
static struct hrt_call serial_dma_call;
#endif
int MuxFlag, MuxState;

__EXPORT int mavstation_main(int argc, char *argv[]);
int
mavstation_main(int argc, char *argv[])
{
	/* reset all to zero */
	memset(&system_state, 0, sizeof(system_state));

	/* configure the high-resolution time/callout interface */
	hrt_init();

	/*
	 * Poll at 1ms intervals for received bytes that have not triggered
	 * a DMA event.
	 */
#ifdef CONFIG_ARCH_DMA
	hrt_call_every(&serial_dma_call, 1000, 1000, (hrt_callout)stm32_serial_dma_poll, NULL);
#endif

	/* print some startup info */
	debug("\nmavstation: starting\n");

	/* start gpio interface */
	gpio_interface_init();
	/* pass usart2 to raspberry pi by default */
	gpio_interface_setusart2mux(false);
	MuxState = 0;
	MuxFlag = 0;

	/* add a performance counter for the interface */
	perf_counter_t interface_perf = perf_alloc(PC_ELAPSED, "interface");

	/* and one for measuring the loop rate */
	perf_counter_t loop_perf = perf_alloc(PC_INTERVAL, "loop");

	/*
	 * Run everything in a tight loop.
	 */

	for (;;) {

		/* track the rate at which the loop is running */
		perf_count(loop_perf);

		/* kick the interface */
		perf_begin(interface_perf);
		gpio_interface_tick();

		if (gpio_interface_getbtn(0) & !MuxFlag) {
			if (MuxState) {
				gpio_interface_setusart2mux(false);
				gpio_interface_setled(2, 1);
				MuxState = 0;

			} else {
				gpio_interface_setusart2mux(true);
				gpio_interface_setled(2, 0);
				MuxState = 1;
			}

			MuxFlag = 1;

		} else if (!gpio_interface_getbtn(0)) {
			MuxFlag = 0;
		}

#ifdef DEBUG_GPIOS

		for (int i = 1; i < 5; i++) {
			gpio_interface_setled(((i % 2)), gpio_interface_getbtn(i));
		}

#endif

		perf_end(interface_perf);

		/* check for debug activity */
		show_debug_messages();
		isr_debug_tick();

	}
}

