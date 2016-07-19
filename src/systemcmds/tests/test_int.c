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

#include <inttypes.h>

#include <px4_config.h>
#include <px4_defines.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <arch/board/board.h>

#include "tests.h"

#include <math.h>
#include <float.h>


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
 * Name: test_led
 ****************************************************************************/

typedef union {
	int32_t i;
	int64_t l;
	uint8_t b[8];
} test_32_64_t;

int test_int(int argc, char *argv[])
{
	int ret = 0;

	printf("\n--- 64 BIT MATH TESTS ---\n");

	int64_t large = 354156329598;

	int64_t calc = large * 5;

	if (calc == 1770781647990) {
		printf("\t success: 354156329598 * 5 == %" PRId64 "\n", calc);

	} else {
		printf("\t FAIL: 354156329598 * 5 != %" PRId64 "\n", calc);
		ret = -1;
	}

	fflush(stdout);





	printf("\n--- 32 BIT / 64 BIT MIXED MATH TESTS ---\n");


	int32_t small = 50;
	int32_t large_int = 2147483647; // MAX INT value

	uint64_t small_times_large = large_int * (uint64_t)small;

	if (small_times_large == 107374182350) {
		printf("\t success: 64bit calculation: 50 * 2147483647 (max int val) == %" PRId64 "\n", small_times_large);

	} else {
		printf("\t FAIL: 50 * 2147483647 != %" PRId64 ", 64bit cast might fail\n", small_times_large);
		ret = -1;
	}

	fflush(stdout);

	if (ret == 0) {
		printf("\n SUCCESS: All float and double tests passed.\n");

	} else {
		printf("\n FAIL: One or more tests failed.\n");
	}

	printf("\n");

	return ret;
}
