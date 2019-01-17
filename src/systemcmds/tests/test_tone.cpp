/****************************************************************************
 *
 *  Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file test_tone.cpp
 * Test for audio tones.
 */

#include <stdlib.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <px4_posix.h>

#include "tests_main.h"


int test_tone(int argc, char *argv[])
{
	int fd, result;
	unsigned long tone;

	fd = px4_open(TONE_ALARM0_DEVICE_PATH, O_WRONLY);

	if (fd < 0) {
		printf("failed opening " TONE_ALARM0_DEVICE_PATH "\n");
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
