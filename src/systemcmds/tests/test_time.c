/****************************************************************************
 * px4/tests/test_time.c
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

#include <px4_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <arch/board/board.h>

#include "tests.h"

#include <math.h>
#include <float.h>
#include <drivers/drv_hrt.h>


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

/* emulate hrt_absolute_time using the cycle counter */
static hrt_abstime
cycletime(void)
{
	static uint64_t basetime;
	static uint32_t lasttime;
	uint32_t cycles;

	cycles = *(unsigned long *)0xe0001004;

	if (cycles < lasttime) {
		basetime += 0x100000000ULL;
	}

	lasttime = cycles;

	return (basetime + cycles) / 168;	/* XXX magic number */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_time
 ****************************************************************************/

int test_time(int argc, char *argv[])
{
	hrt_abstime h, c;
	int64_t lowdelta, maxdelta = 0;
	int64_t delta, deltadelta;

	/* enable the cycle counter */
	(*(unsigned long *)0xe000edfc) |= (1 << 24);    /* DEMCR |= DEMCR_TRCENA */
	(*(unsigned long *)0xe0001000) |= 1;    	/* DWT_CTRL |= DWT_CYCCNT_ENA */

	/* get an average delta between the two clocks - this should stay roughly the same */
	delta = 0;

	for (unsigned i = 0; i < 100; i++) {
		uint32_t flags = irqsave();

		h = hrt_absolute_time();
		c = cycletime();

		irqrestore(flags);

		delta += h - c;
	}

	lowdelta = abs(delta / 100);

	/* loop checking the time */
	for (unsigned i = 0; i < 100; i++) {

		usleep(rand());

		uint32_t flags = irqsave();

		c = cycletime();
		h = hrt_absolute_time();

		irqrestore(flags);

		delta = abs(h - c);
		deltadelta = abs(delta - lowdelta);

		if (deltadelta > maxdelta) {
			maxdelta = deltadelta;
		}

		if (deltadelta > 1000) {
			fprintf(stderr, "h %" PRIu64 " c %" PRIu64 " d %" PRId64 "\n", h, c, delta - lowdelta);
		}
	}

	printf("Maximum jitter %" PRId64 "us\n", maxdelta);

	return 0;
}
