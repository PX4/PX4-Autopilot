/****************************************************************************
 * px4/sensors/test_gpio.c
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

/****************************************************************************
 * Name: test_gpio
 ****************************************************************************/

int test_adc(int argc, char *argv[])
{
	int		fd0;
	int		ret = 0;
	//struct adc_msg_s sample[4],sample2[4],sample3[4],sample4[4],sample5[4],sample6[4],sample7[4],sample8[4],sample9[4];

	struct adc_msg4_s {
		uint8_t      am_channel1;               /* The 8-bit ADC Channel */
		int32_t      am_data1;                  /* ADC convert result (4 bytes) */
		uint8_t      am_channel2;               /* The 8-bit ADC Channel */
		int32_t      am_data2;                  /* ADC convert result (4 bytes) */
		uint8_t      am_channel3;               /* The 8-bit ADC Channel */
		int32_t      am_data3;                  /* ADC convert result (4 bytes) */
		uint8_t      am_channel4;               /* The 8-bit ADC Channel */
		int32_t      am_data4;                  /* ADC convert result (4 bytes) */
	} __attribute__((__packed__));;

	struct adc_msg4_s sample1[4], sample2[4];

	size_t readsize;
	ssize_t nbytes, nbytes2;
	int i;
	int j;
	int errval;

	for (j = 0; j < 1; j++) {
		char name[11];
		sprintf(name, "/dev/adc%d", j);
		fd0 = open(name, O_RDONLY | O_NONBLOCK);

		if (fd0 < 0) {
			printf("ADC: %s open fail\n", name);
			return ERROR;

		} else {
			printf("Opened %s successfully\n", name);
		}

		/* first adc read round */
		readsize = 4 * sizeof(struct adc_msg_s);
		up_udelay(10000);//microseconds
		nbytes = read(fd0, sample1, readsize);
		up_udelay(10000);//microseconds
		nbytes2 = read(fd0, sample2, readsize);
//	nbytes2 = read(fd0, sample3, readsize);
//	nbytes2 = read(fd0, sample4, readsize);
//	nbytes2 = read(fd0, sample5, readsize);
//	nbytes2 = read(fd0, sample6, readsize);
//	nbytes2 = read(fd0, sample7, readsize);
//	nbytes2 = read(fd0, sample8, readsize);
		//nbytes2 = read(fd0, sample9, readsize);

		/* Handle unexpected return values */

		if (nbytes < 0) {
			errval = errno;

			if (errval != EINTR) {
				message("read %s failed: %d\n",
					name, errval);
				errval = 3;
				goto errout_with_dev;
			}

			message("\tInterrupted read...\n");

		} else if (nbytes == 0) {
			message("\tNo data read, Ignoring\n");
		}

		/* Print the sample data on successful return */

		else {
			int nsamples = nbytes / sizeof(struct adc_msg_s);

			if (nsamples * sizeof(struct adc_msg_s) != nbytes) {
				message("\tread size=%d is not a multiple of sample size=%d, Ignoring\n",
					nbytes, sizeof(struct adc_msg_s));

			} else {
				message("Sample:");

				for (i = 0; i < 1 ; i++) {
					message("%d: channel: %d value: %d\n",
						i, sample1[i].am_channel1, sample1[i].am_data1);
					message("Sample:");
					message("%d: channel: %d value: %d\n",
						i, sample1[i].am_channel2, sample1[i].am_data2);
					message("Sample:");
					message("%d: channel: %d value: %d\n",
						i, sample1[i].am_channel3, sample1[i].am_data3);
					message("Sample:");
					message("%d: channel: %d value: %d\n",
						i, sample1[i].am_channel4, sample1[i].am_data4);
					message("Sample:");
					message("%d: channel: %d value: %d\n",
						i + 1, sample2[i].am_channel1, sample2[i].am_data1);
					message("Sample:");
					message("%d: channel: %d value: %d\n",
						i + 1, sample2[i].am_channel2, sample2[i].am_data2);
					message("Sample:");
					message("%d: channel: %d value: %d\n",
						i + 1, sample2[i].am_channel3, sample2[i].am_data3);
					message("Sample:");
					message("%d: channel: %d value: %d\n",
						i + 1, sample2[i].am_channel4, sample2[i].am_data4);
//    			message("%d: channel: %d value: %d\n",
//						i, sample9[i].am_channel, sample9[i].am_data);
				}
			}
		}
	}

	printf("\t ADC test successful.\n");

errout_with_dev:

	if (fd0 != 0) close(fd0);

	return ret;
}
