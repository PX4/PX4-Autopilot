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
 * @file controls.c
 *
 * R/C inputs and servo outputs.
 */


#include <nuttx/config.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <debug.h>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <poll.h>

#include <nuttx/clock.h>

#include <drivers/drv_hrt.h>
#include <systemlib/hx_stream.h>
#include <systemlib/perf_counter.h>
#include <systemlib/ppm_decode.h>

#define DEBUG
#include "px4io.h"

#define RC_FAILSAFE_TIMEOUT		2000000		/**< two seconds failsafe timeout */
#define RC_CHANNEL_HIGH_THRESH		1700
#define RC_CHANNEL_LOW_THRESH		1300

static void	ppm_input(void);

void
controls_main(void)
{
	struct pollfd fds[2];

	/* DSM input */
	fds[0].fd = dsm_init("/dev/ttyS0");
	fds[0].events = POLLIN;

	/* S.bus input */
	fds[1].fd = sbus_init("/dev/ttyS2");
	fds[1].events = POLLIN;

	for (;;) {
		/* run this loop at ~100Hz */
		poll(fds, 2, 10);

		/*
		 * Gather R/C control inputs from supported sources.
		 *
		 * Note that if you're silly enough to connect more than
		 * one control input source, they're going to fight each
		 * other.  Don't do that.
		 */
		bool locked = false;

		/*
		 * Store RC channel count to detect switch to RC loss sooner
		 * than just by timeout
		 */
		unsigned rc_channels = system_state.rc_channels;

		if (fds[0].revents & POLLIN)
			locked |= dsm_input();

		if (fds[1].revents & POLLIN)
			locked |= sbus_input();

		/*
		 * If we don't have lock from one of the serial receivers,
		 * look for PPM. It shares an input with S.bus, so there's
		 * a possibility it will mis-parse an S.bus frame.
		 *
		 * XXX each S.bus frame will cause a PPM decoder interrupt
		 * storm (lots of edges).  It might be sensible to actually
		 * disable the PPM decoder completely if we have an alternate
		 * receiver lock.
		 */
		if (!locked)
			ppm_input();

		/* check for manual override status */
		if (system_state.rc_channel_data[4] > RC_CHANNEL_HIGH_THRESH) {
			/* force manual input override */
			system_state.mixer_manual_override = true;

		} else {
			/* override not engaged, use FMU */
			system_state.mixer_manual_override = false;
		}

		/*
		 * If we haven't seen any new control data in 200ms, assume we
		 * have lost input and tell FMU.
		 */
		if ((hrt_absolute_time() - system_state.rc_channels_timestamp) > 200000) {

			if (system_state.rc_channels > 0) {
				/*
				 * If the RC signal status (lost / present) has
				 * just changed, request an update immediately.
				 */
				system_state.fmu_report_due = true;
			}

			/* set the number of channels to zero - no inputs */
			system_state.rc_channels = 0;
		}

		/*
		 * PWM output updates are performed in addition on each comm update.
		 * the updates here are required to ensure operation if FMU is not started
		 * or stopped responding.
		 */
		mixer_tick();
	}
}

static void
ppm_input(void)
{
	/*
	 * Look for new PPM input.
	 */
	if (ppm_last_valid_decode != 0) {

		/* avoid racing with PPM updates */
		irqstate_t state = irqsave();

		/* PPM data exists, copy it */
		system_state.rc_channels = ppm_decoded_channels;

		for (unsigned i = 0; i < ppm_decoded_channels; i++) {
			system_state.rc_channel_data[i] = ppm_buffer[i];
		}

		/* copy the timestamp and clear it */
		system_state.rc_channels_timestamp = ppm_last_valid_decode;
		ppm_last_valid_decode = 0;

		irqrestore(state);

		/* trigger an immediate report to the FMU */
		system_state.fmu_report_due = true;
	}
}
