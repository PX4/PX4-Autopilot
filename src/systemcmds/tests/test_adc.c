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
 * @file test_adc.c
 * Test for the analog to digital converter.
 */

#include <px4_time.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_log.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "tests_main.h"

#include <drivers/drv_adc.h>

int test_adc(int argc, char *argv[])
{
	int fd = px4_open(ADC0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("ERROR: can't open ADC device");
		return 1;
	}

	for (unsigned i = 0; i < 5; i++) {
		/* make space for a maximum number of channels */
		px4_adc_msg_t data[PX4_MAX_ADC_CHANNELS];
		/* read all channels available */
		ssize_t count = px4_read(fd, data, sizeof(data));

		if (count < 0) {
			goto errout_with_dev;
		}

		unsigned channels = count / sizeof(data[0]);

		for (unsigned j = 0; j < channels; j++) {
			printf("%d: %u  ", data[j].am_channel, data[j].am_data);
		}

		printf("\n");
		px4_usleep(150000);
	}

	printf("\t ADC test successful.\n");

errout_with_dev:

	if (fd != 0) { px4_close(fd); }

	return OK;
}
