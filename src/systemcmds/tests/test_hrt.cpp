/****************************************************************************
 * px4/sensors/test_hrt.c
 *
 *  Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_config.h>
#include <px4_posix.h>
#include <sys/types.h>
#include <sys/time.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>

#include "tests_main.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern uint16_t ppm_buffer[];
extern unsigned ppm_decoded_channels;
// extern uint16_t ppm_edge_history[];
// extern uint16_t ppm_pulse_history[];

int test_ppm(int argc, char *argv[])
{
#ifdef HRT_PPM_CHANNEL
	unsigned i;

	printf("channels: %u\n", ppm_decoded_channels);

	for (i = 0; i < ppm_decoded_channels; i++) {
		printf("  %u\n", ppm_buffer[i]);
	}

	// printf("edges\n");

	// for (i = 0; i < 32; i++) {
	// 	printf("  %u\n", ppm_edge_history[i]);
	// }

	// printf("pulses\n");

	// for (i = 0; i < 32; i++) {
	// 	printf("  %u\n", ppm_pulse_history[i]);
	// }

	fflush(stdout);
#else
	printf("PPM not configured\n");
#endif
	return 0;
}

int test_tone(int argc, char *argv[])
{
	int fd, result;
	unsigned long tone;

	fd = px4_open(TONEALARM0_DEVICE_PATH, O_WRONLY);

	if (fd < 0) {
		printf("failed opening " TONEALARM0_DEVICE_PATH "\n");
		goto out;
	}

	tone = 1;

	if (argc == 2) {
		tone = atoi(argv[1]);
	}

	if (tone  == 0) {
		result = px4_ioctl(fd, TONE_SET_ALARM, TONE_STOP_TUNE);

		if (result < 0) {
			printf("failed clearing alarms\n");
			goto out;

		} else {
			printf("Alarm stopped.\n");
		}

	} else {
		result = px4_ioctl(fd, TONE_SET_ALARM, TONE_STOP_TUNE);

		if (result < 0) {
			printf("failed clearing alarms\n");
			goto out;
		}

		result = px4_ioctl(fd, TONE_SET_ALARM, tone);

		if (result < 0) {
			printf("failed setting alarm %lu\n", tone);

		} else {
			printf("Alarm %lu (disable with: tests tone 0)\n", tone);
		}
	}

out:

	if (fd >= 0) {
		px4_close(fd);
	}

	return 0;
}

/****************************************************************************
 * Name: test_hrt
 ****************************************************************************/

int test_hrt(int argc, char *argv[])
{
	struct hrt_call call;
	hrt_abstime prev, now;
	int i;
	struct timeval tv1, tv2;

	printf("start-time (hrt, sec/usec), end-time (hrt, sec/usec), microseconds per 1/10 second\n");

	for (i = 0; i < 10; i++) {
		prev = hrt_absolute_time();
		gettimeofday(&tv1, nullptr);
		px4_usleep(100000);
		now = hrt_absolute_time();
		gettimeofday(&tv2, nullptr);
		printf("%lu (%lu/%lu), %lu (%lu/%lu), %lu\n",
		       (unsigned long)prev, (unsigned long)tv1.tv_sec, (unsigned long)tv1.tv_usec,
		       (unsigned long)now, (unsigned long)tv2.tv_sec, (unsigned long)tv2.tv_usec,
		       (unsigned long)(hrt_absolute_time() - prev));
		fflush(stdout);
	}

	px4_usleep(1000000);

	printf("one-second ticks\n");

	for (i = 0; i < 3; i++) {
		hrt_call_after(&call, 1000000, nullptr, nullptr);

		while (!hrt_called(&call)) {
			px4_usleep(1000);
		}

		printf("tick\n");
		fflush(stdout);
	}

	return 0;
}
