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

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi.h>

#include "tests.h"

#include <nuttx/analog/adc.h>

int test_adc(int argc, char *argv[])
{
	int		fd0 = 0;
	int		ret = 0;

	#pragma pack(push,1)
	struct adc_msg4_s {
		uint8_t      am_channel1;	/**< The 8-bit ADC Channel 1 */
		int32_t      am_data1;		/**< ADC convert result 1 (4 bytes) */
		uint8_t      am_channel2;	/**< The 8-bit ADC Channel 2 */
		int32_t      am_data2;		/**< ADC convert result 2 (4 bytes) */
		uint8_t      am_channel3;	/**< The 8-bit ADC Channel 3 */
		int32_t      am_data3;		/**< ADC convert result 3 (4 bytes) */
		uint8_t      am_channel4;	/**< The 8-bit ADC Channel 4 */
		int32_t      am_data4;		/**< ADC convert result 4 (4 bytes) */
	};
	#pragma pack(pop)

	struct adc_msg4_s sample1;

	ssize_t nbytes;
	int j;
	int errval;

	fd0 = open("/dev/adc0", O_RDONLY | O_NONBLOCK);

	if (fd0 <= 0) {
		message("/dev/adc0 open fail: %d\n", errno);
		return ERROR;

	} else {
		message("opened /dev/adc0 successfully\n");
	}
	usleep(10000);

	for (j = 0; j < 10; j++) {

		/* sleep 20 milliseconds */
		usleep(20000);
		nbytes = read(fd0, &sample1, sizeof(sample1));

		/* Handle unexpected return values */

		if (nbytes < 0) {
			errval = errno;

			if (errval != EINTR) {
				message("reading /dev/adc0 failed: %d\n", errval);
				errval = 3;
				goto errout_with_dev;
			}

			message("\tinterrupted read..\n");

		} else if (nbytes == 0) {
			message("\tno data read, ignoring.\n");
			ret = ERROR;
		}

		/* Print the sample data on successful return */

		else {
			if (nbytes != sizeof(sample1)) {
				message("\tsample 1 size %d is not matching struct size %d, ignoring\n",
					nbytes, sizeof(sample1));
				ret = ERROR;

			} else {

				message("CYCLE %d:\n", j);

				message("channel: %d value: %d\n",
					(int)sample1.am_channel1, sample1.am_data1);
				message("channel: %d value: %d\n",
					(int)sample1.am_channel2, sample1.am_data2);
				message("channel: %d value: %d\n",
					(int)sample1.am_channel3, sample1.am_data3);
				message("channel: %d value: %d\n",
					(int)sample1.am_channel4, sample1.am_data4);
			}
		}
		fflush(stdout);
	}

	message("\t ADC test successful.\n");

errout_with_dev:

	if (fd0 != 0) close(fd0);

	return ret;
}
