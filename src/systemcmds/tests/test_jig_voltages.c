/****************************************************************************
 *
 *  Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file test_jig_voltages.c
 * Tests for jig voltages.
 */

#include <px4_platform_common/config.h>
#include <px4_platform_common/defines.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "tests_main.h"

#include <drivers/drv_adc.h>
#include <systemlib/err.h>

int test_jig_voltages(int argc, char *argv[])
{
	int fd = open(ADC0_DEVICE_PATH, O_RDONLY);
	int ret = OK;

	if (fd < 0) {
		PX4_ERR("can't open ADC device");
		return 1;
	}

	/* make space for the  maximum channels */
	px4_adc_msg_t data[PX4_MAX_ADC_CHANNELS];

	/* read all channels available */
	ssize_t count = read(fd, data, sizeof(data));

	if (count < 0) {
		close(fd);
		PX4_ERR("can't read from ADC driver. Forgot 'adc start' command?");
		return 1;
	}

	unsigned channels = count / sizeof(data[0]);

	for (unsigned j = 0; j < channels; j++) {
		printf("%d: %u  ", data[j].am_channel, data[j].am_data);
	}

	printf("\n");

	PX4_INFO("\t ADC operational.\n");

	/* Expected values */
	int16_t expected_min[] = {2800, 2800, 1800,  800};
	int16_t expected_max[] = {3100, 3100, 2100, 1100};
	char *check_res[channels];

	if (channels < 4) {
		close(fd);
		PX4_ERR("not all four test channels available, aborting.");
		return 1;

	} else {
		/* Check values */
		check_res[0] = (expected_min[0] < data[0].am_data && expected_max[0] > data[0].am_data) ? "OK" : "FAIL";
		check_res[1] = (expected_min[1] < data[1].am_data && expected_max[1] > data[1].am_data) ? "OK" : "FAIL";
		check_res[2] = (expected_min[2] < data[2].am_data && expected_max[2] > data[2].am_data) ? "OK" : "FAIL";
		check_res[3] = (expected_min[3] < data[3].am_data && expected_max[3] > data[3].am_data) ? "OK" : "FAIL";

		/* Accumulate result */
		ret += (expected_min[0] > data[0].am_data || expected_max[0] < data[0].am_data) ? 1 : 0;
		ret += (expected_min[1] > data[1].am_data || expected_max[1] < data[1].am_data) ? 1 : 0;
		ret += (expected_min[2] > data[2].am_data || expected_max[2] < data[2].am_data) ? 1 : 0;
		ret += (expected_min[3] > data[3].am_data || expected_max[3] < data[3].am_data) ? 1 : 0;

		PX4_INFO("Sample:");
		PX4_INFO("channel: %d value: %d (allowed min: %d, allowed max: %d), result: %s",
			 data[0].am_channel, (int)(data[0].am_data), expected_min[0], expected_max[0], check_res[0]);
		PX4_INFO("channel: %d value: %d (allowed min: %d, allowed max: %d), result: %s",
			 data[1].am_channel, (int)(data[1].am_data), expected_min[1], expected_max[1], check_res[1]);
		PX4_INFO("channel: %d value: %d (allowed min: %d, allowed max: %d), result: %s",
			 data[2].am_channel, (int)(data[2].am_data), expected_min[2], expected_max[2], check_res[2]);
		PX4_INFO("channel: %d value: %d (allowed min: %d, allowed max: %d), result: %s",
			 data[3].am_channel, (int)(data[3].am_data), expected_min[3], expected_max[3], check_res[3]);

		if (ret != OK) {
			PX4_ERR("\t JIG voltages test FAILED. Some channels where out of allowed range. Check supply voltages.");
			goto errout_with_dev;
		}
	}

	PX4_INFO("JIG voltages test successful.");

errout_with_dev:

	if (fd != 0) { close(fd); }

	return ret;
}
