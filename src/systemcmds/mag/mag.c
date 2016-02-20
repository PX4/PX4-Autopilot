/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Andrew Tridgell
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
 * @file mag.c
 */

#include <px4_config.h>
#include <termios.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <apps/nsh.h>
#include <fcntl.h>
#include <poll.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/sensor_mag.h>
#define mag_report sensor_mag_s
/*
struct sensor_mag_s {
		uint64_t timestamp;
		uint64_t error_count;
		float x;
		float y;
		float z;
		float range_ga;
		float scaling;
		float temperature;
		int16_t x_raw;
		int16_t y_raw;
		int16_t z_raw;
};
 */

__EXPORT int mag_main(int argc, char *argv[]);

int
mag_main(int argc, char *argv[])
{
	if (argc < 2) {
		printf("Usage: mag [update rate 1 - 10 hertz]\n");
		exit(1);
	}
	int rate = atoi(argv[1]);
	printf("mag %u\n", rate);
	if (rate > 10 || rate < 1) {
		rate = 1;
	}

	int mag_fd = orb_subscribe(ORB_ID(sensor_mag));
	struct mag_report mrp;

	while (true) {

		bool updated = false;
		orb_check(mag_fd, &updated);

		if (updated) {
			orb_copy(ORB_ID(sensor_mag), mag_fd, &mrp);

			printf("magxyz %d %d %d\n", mrp.x_raw, mrp.y_raw, mrp.z_raw);
		}
//		usleep(100000);

		/* Sleep 100 ms waiting for user input five times ~ 1s */
		for (int k = 0; k < (11 - rate); k++) {
			struct pollfd fds;
			int ret;
			fds.fd = 0; /* stdin */
			fds.events = POLLIN;
			ret = poll(&fds, 1, 0);
			if (ret > 0) {
				char c;

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
			usleep(100000);
		}

	}
	return OK;
}
