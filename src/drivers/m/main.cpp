/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file main.cpp
 *
 * Driver for the Invensense mpu9250 connected via SPI.
 *
 * @authors Robert Dickenson
 *
 * based on the mpu6000 driver
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>
#include <string.h>

#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_mag.h>

#include "mag.h"
#include "mpu9250.h"

#define MPU_DEVICE_PATH_ACCEL		"/dev/mpu9250_accel"
#define MPU_DEVICE_PATH_GYRO		"/dev/mpu9250_gyro"
#define MPU_DEVICE_PATH_MAG 		"/dev/mpu9250_mag"

/** driver 'main' command */
extern "C" { __EXPORT int m_main(int argc, char *argv[]); }

bool quit(void);
void mpu(int interval);
void measure_callback(struct Report &report);


#define MAX_SAMPLES 1000
struct Report reports[MAX_SAMPLES];
static int idx = 0;
MPU9250	*g_dev;


extern int duplicates;

void measure_callback(struct Report &report)
{
	if (idx < MAX_SAMPLES) {
		report.cnt0 = duplicates;
		duplicates = 0;
		reports[idx] = report;
		idx++;
	}
}

void
mpu(int interval)
{
	g_dev->set_frequency_high();

	while (true) {

		printf("********* SAMPLING ***********\n");

		memset(&reports, 0, sizeof(reports));
		idx = 0;
		if (interval != 0) {
			g_dev->start(interval);
			while (idx < MAX_SAMPLES) {
				usleep(1000);
				// allow the timer to call measure_callback on each cycle
			}
			g_dev->stop();
		} else {
			while (idx < MAX_SAMPLES) {
				struct Report report;
				memset(&report, 0, sizeof(report));
				if (g_dev->measure(report)) {
					measure_callback(report);
				}
			}
		}

		printf("   ax    ay    az :    mx    my    mz : [c0 c1 c2 c3 c4]\n");
		for (int i = 0; i < MAX_SAMPLES; i++) {
			if (quit()) return;

				printf("%5d %5d %5d : ", reports[i].accel_x, reports[i].accel_y, reports[i].accel_z);
//				printf("%5d %5d %5d : ", reports[i].gyro_x, reports[i].gyro_y, reports[i].gyro_z);
				printf("%5d %5d %5d : ", reports[i].mag_x, reports[i].mag_y, reports[i].mag_z);
				printf("[%02x %02x %02x %02x %02x] ", reports[i].cnt0, reports[i].cnt1, reports[i].cnt2, reports[i].cnt3, reports[i].cnt4);

				if (!(reports[i].cnt0)) {
					printf("* ");
				} else {
					printf("  ");
				}
				if (reports[i].cnt1) {
					printf("! ");
				} else {
					printf("  ");
				}
				if (reports[i].period) {
					printf("%6u us ", reports[i].period);
				} else {
					printf("       ");
				}
				printf("\n");
		}

	}
}

int
m_main(int argc, char *argv[])
{
	int interval = 0;
	int filter = 0;
	int ch;

	while ((ch = getopt(argc, argv, "I:F:")) != EOF) {
		switch (ch) {
		case 'I':  // interval
			interval = atoi(optarg);
			break;

		case 'F':  // filter
			filter = atoi(optarg);
			break;

		default:
			break;
		}
	}

	if (interval > 2000 || interval < 0) {
		interval = 1000;
	}

	printf("%s interval = %u, filter = %u\n", argv[0], interval, filter);
	printf("sizeof(reports) %u\n", sizeof(reports));

	const char *verb = argv[optind];

	/* create the driver */
	g_dev = new MPU9250(filter);


	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	if (!strcmp(verb, "regdump")) {
		g_dev->print_registers();
	} else {
		mpu(interval);
	}

	if (g_dev != nullptr) {
		delete(g_dev);
	}
	exit(0);

fail:
	if (g_dev != nullptr) {
		delete g_dev;
	}
	errx(1, "driver start failed");
	exit(1);
}

bool
quit(void)
{
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
			return true;
		}
	}
	return false;
}
