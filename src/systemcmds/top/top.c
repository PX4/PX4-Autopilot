/****************************************************************************
 *
 *   Copyright (c) 2012, 2013, 2015 PX4 Development Team. All rights reserved.
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
 * @file top.c
 * Tool similar to UNIX top command
 * @see http://en.wikipedia.org/wiki/Top_unix
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>

#include <systemlib/cpuload.h>
#include <systemlib/printload.h>
#include <drivers/drv_hrt.h>

/**
 * Start the top application.
 */
__EXPORT int top_main(int argc, char *argv[]);

int
top_main(int argc, char *argv[])
{
	hrt_abstime curr_time = hrt_absolute_time();

	struct print_load_s load;
	init_print_load_s(curr_time, &load);

	/* clear screen */
	dprintf(1, "\033[2J\n");

	if (argc > 1 && !strcmp(argv[1], "once")) {
		print_load(curr_time, 1, &load);
		sleep(1);
		print_load(hrt_absolute_time(), 1, &load);
		return 0;
	}

	for (;;) {
		print_load(curr_time, 1, &load);

		/* Sleep 200 ms waiting for user input five times ~ 1s */
		for (int k = 0; k < 5; k++) {
			char c;

			struct pollfd fds;
			int ret;
			fds.fd = 0; /* stdin */
			fds.events = POLLIN;
			ret = poll(&fds, 1, 0);

			if (ret > 0) {

				read(0, &c, 1);

				switch (c) {
				case 0x03: // ctrl-c
				case 0x1b: // esc
				case 'c':
				case 'q':
					return OK;
					/* not reached */
				}
			}

			usleep(200000);
		}

		curr_time = hrt_absolute_time();
	}

	return OK;
}
