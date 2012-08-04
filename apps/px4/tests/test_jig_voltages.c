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

int test_jig_voltages(int argc, char *argv[])
{
	int		fd0 = 0;
	int		ret = OK;
	const int nchannels = 4;

	struct adc_msg4_s
	{
	  uint8_t      am_channel1;               /* The 8-bit ADC Channel */
	  int32_t      am_data1;                  /* ADC convert result (4 bytes) */
	  uint8_t      am_channel2;               /* The 8-bit ADC Channel */
	  int32_t      am_data2;                  /* ADC convert result (4 bytes) */
	  uint8_t      am_channel3;               /* The 8-bit ADC Channel */
	  int32_t      am_data3;                  /* ADC convert result (4 bytes) */
	  uint8_t      am_channel4;               /* The 8-bit ADC Channel */
	  int32_t      am_data4;                  /* ADC convert result (4 bytes) */
	}__attribute__((__packed__));;

	struct adc_msg4_s sample1[4];

	size_t readsize;
	ssize_t nbytes;
	int i = 0;
	int j = 0;
	int errval;

	char name[11];
	sprintf(name, "/dev/adc%d", j);
	fd0 = open(name, O_RDONLY | O_NONBLOCK);
	if (fd0 < 0)
	{
		printf("ADC: %s open fail\n", name);
		return ERROR;
	} else {
		printf("Opened %s successfully\n", name);
	}


	/* Expected values */
	int16_t expected_min[] = {2700, 2700, 2200, 2000};
	int16_t expected_max[] = {3000, 3000, 2500, 2200};
	char* check_res[nchannels];

	/* first adc read round */
	readsize = 4 * sizeof(struct adc_msg_s);

	/* Empty all buffers */
	do {
		nbytes = read(fd0, sample1, readsize);
	}
	while (nbytes > 0);

	up_udelay(20000);//microseconds
	/* Take measurements */
	nbytes = read(fd0, sample1, readsize);

    /* Handle unexpected return values */

    if (nbytes <= 0)
      {
        errval = errno;
        if (errval != EINTR)
          {
            message("read %s failed: %d\n",
                    name, errval);
            errval = 3;
            goto errout_with_dev;
          }

        message("\tInterrupted read...\n");
      }
    else if (nbytes == 0)
      {
        message("\tNo data read, Ignoring\n");
      }

    /* Print the sample data on successful return */

    else
    {
    	int nsamples = nbytes / sizeof(struct adc_msg_s);
    	if (nsamples * sizeof(struct adc_msg_s) != nbytes)
    	{
    		message("\tread size=%d is not a multiple of sample size=%d, Ignoring\n",
    				nbytes, sizeof(struct adc_msg_s));
    	}
    	else
    	{
    		/* Check values */
    		check_res[0] = (expected_min[0] < sample1[i].am_data1 && expected_max[0] > sample1[i].am_data1) ? "OK" : "FAIL";
    		check_res[1] = (expected_min[1] < sample1[i].am_data2 && expected_max[1] > sample1[i].am_data2) ? "OK" : "FAIL";
    		check_res[2] = (expected_min[2] < sample1[i].am_data3 && expected_max[2] > sample1[i].am_data3) ? "OK" : "FAIL";
    		check_res[3] = (expected_min[3] < sample1[i].am_data4 && expected_max[3] > sample1[i].am_data4) ? "OK" : "FAIL";

    		/* Accumulate result */
    		ret += (expected_min[0] > sample1[i].am_data1 || expected_max[0] < sample1[i].am_data1) ? 1 : 0;
    		// XXX Chan 11 not connected on test setup
    		//ret += (expected_min[1] > sample1[i].am_data2 || expected_max[1] < sample1[i].am_data2) ? 1 : 0;
    		ret += (expected_min[2] > sample1[i].am_data3 || expected_max[2] < sample1[i].am_data3) ? 1 : 0;
    		ret += (expected_min[3] > sample1[i].am_data4 || expected_max[3] < sample1[i].am_data4) ? 1 : 0;

    		message("Sample:");
    		message("%d: channel: %d value: %d (allowed min: %d, allowed max: %d), result: %s\n",
    					i, sample1[i].am_channel1, sample1[i].am_data1, expected_min[0], expected_max[0], check_res[0]);
        	message("Sample:");
    		message("%d: channel: %d value: %d (allowed min: %d, allowed max: %d), result: %s\n",
						i, sample1[i].am_channel2, sample1[i].am_data2, expected_min[1], expected_max[1], check_res[1]);
        	message("Sample:");
    		message("%d: channel: %d value: %d (allowed min: %d, allowed max: %d), result: %s\n",
						i, sample1[i].am_channel3, sample1[i].am_data3, expected_min[2], expected_max[2], check_res[2]);
        	message("Sample:");
    		message("%d: channel: %d value: %d (allowed min: %d, allowed max: %d), result: %s\n",
						i, sample1[i].am_channel4, sample1[i].am_data4, expected_min[3], expected_max[3], check_res[3]);

    		if (ret != OK) {
    			printf("\t ADC test FAILED. Some channels where out of allowed range. Check supply voltages.\n");
    		 	goto errout_with_dev;
    		}
    	}
    }

	printf("\t ADC test successful.\n");

	errout_with_dev:
	  if (fd0 != 0) close(fd0);

	return ret;
}
