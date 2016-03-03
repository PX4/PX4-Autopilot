/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *   Author: Robert Dickenson
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
 *
 * Utility for viewing magnetometer ORB publications, specifically for
 * development work on the Invensense mpu9250 connected via SPI.
 *
 * @author Robert Dickenson
 *
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

int mag1(void);
int mag2(void);

int
mag1(void)
{
	int16_t x_raw = 0;
	int16_t y_raw = 0;
	int16_t z_raw = 0;
	struct mag_report mrb;
	int mag_fd = orb_subscribe(ORB_ID(sensor_mag));

	while (true) {

		bool updated = false;
		orb_check(mag_fd, &updated);

		if (updated) {
			orb_copy(ORB_ID(sensor_mag), mag_fd, &mrb);

//			printf("magxyz %d %d %d ", mrb.x_raw, mrb.y_raw, mrb.z_raw);
			printf("magxyz %8.4f %8.4f %8.4f ", (double)mrb.x, (double)mrb.y, (double)mrb.z);

			uint8_t cnt0 = (mrb.error_count & 0x000000FF00000000) >> 32;
			uint8_t cnt1 = (mrb.error_count & 0x00000000FF000000) >> 24;
			uint8_t cnt2 = (mrb.error_count & 0x0000000000FF0000) >> 16;
			uint8_t cnt3 = (mrb.error_count & 0x000000000000FF00) >> 8;
			uint8_t cnt4 = (mrb.error_count & 0x00000000000000FF);
			printf("[%x %x %x %x %x] ", cnt0, cnt1, cnt2, cnt3, cnt4);

			uint32_t mag_interval = (mrb.error_count & 0xFFFFFF0000000000) >> 40;
			printf("%u  ", mag_interval);
			if ((mag_interval) < 5000) {
				printf("* ");
			}
			if ((mag_interval) > 15000) {
				printf("! ");
			}

			if (mrb.x_raw == x_raw && mrb.y_raw == y_raw && mrb.z_raw == z_raw ) {
				printf("dup "); // duplicate magnetometer xyz data
			}
			x_raw = mrb.x_raw;
			y_raw = mrb.y_raw;
			z_raw = mrb.z_raw;
			printf("\n");
		}

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
			}
		}
	}
	return OK;
}

#define MAX_SAMPLES 100
static struct mag_report mrb[MAX_SAMPLES];
static uint64_t last_timestamp;

int
mag2(void)
{
	int i = 0;

	int mag_fd = orb_subscribe(ORB_ID(sensor_mag));

	while (i < MAX_SAMPLES) {

		bool updated = false;
		orb_check(mag_fd, &updated);

		if (updated) {
			orb_copy(ORB_ID(sensor_mag), mag_fd, &mrb[i++]);
		}
/*
		struct pollfd fds;
		int ret;
		fds.fd = 0; // stdin
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
			}
		}
 */
	}

	uint32_t total_time = 0;
	last_timestamp = mrb[0].timestamp;

	for (i = 0; i < MAX_SAMPLES; i++) {

//			printf("magxyz %d %d %d ", mrb[i].x_raw, mrb[i].y_raw, mrb[i].z_raw);
			printf("magxyz %8.4f %8.4f %8.4f ", (double)mrb[i].x, (double)mrb[i].y, (double)mrb[i].z);

			uint8_t cnt0 = (mrb[i].error_count & 0x000000FF00000000) >> 32;
			uint8_t cnt1 = (mrb[i].error_count & 0x00000000FF000000) >> 24;
			uint8_t cnt2 = (mrb[i].error_count & 0x0000000000FF0000) >> 16;
			uint8_t cnt3 = (mrb[i].error_count & 0x000000000000FF00) >> 8;
			uint8_t cnt4 = (mrb[i].error_count & 0x00000000000000FF);
			printf("[%x %x %x %x %x] ", cnt0, cnt1, cnt2, cnt3, cnt4);

			uint32_t mag_interval = (mrb[i].error_count & 0xFFFFFF0000000000) >> 40;
			printf("%u  ", mag_interval);
			if ((mag_interval) < 5000) {
				printf("* ");
			}
			if ((mag_interval) > 15000) {
				printf("! ");
			}

			uint32_t orb_interval = mrb[i].timestamp - last_timestamp;
			if (orb_interval != mag_interval) {
				printf("orb ", orb_interval); // missed some orb publications
			}
			last_timestamp = mrb[i].timestamp;
			total_time += orb_interval;

			if (mrb[i].x_raw == mrb[i-1].x_raw && mrb[i].y_raw == mrb[i-1].y_raw && mrb[i].z_raw == mrb[i-1].z_raw ) {
				printf("dup "); // duplicate magnetometer xyz data
			}

			printf("\n");
	}
	double t = (double)total_time / 1000000.0;
	printf("total_time: %.3f s\n", t);
	return OK;
}


__EXPORT int mag_main(int argc, char *argv[]);

int
mag_main(int argc, char *argv[])
{
	if (argc < 2) {
		mag1();
	} else {
		int reps = atoi(argv[1]);
		if (reps > 20 || reps < 1) {
			reps = 1;
		}
		while (reps--) {
			mag2();
		}
	}
	return OK;
}
