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

#define DEBUG
#include "px4io.h"

void
controls_main(void)
{
	struct pollfd fds[2];

	fds[0].fd = dsm_init("/dev/ttyS0");
	fds[0].events = POLLIN;


	fds[1].fd = sbus_init("/dev/ttyS2");
	fds[1].events = POLLIN;

	for (;;) {
		/* run this loop at ~100Hz */
		poll(fds, 2, 10);

		if (fds[0].revents & POLLIN)
			dsm_input();
		if (fds[1].revents & POLLIN)
			sbus_input();

		/* XXX do ppm processing, bypass mode, etc. here */

		/* do PWM output updates */
		mixer_tick();
	}
}
