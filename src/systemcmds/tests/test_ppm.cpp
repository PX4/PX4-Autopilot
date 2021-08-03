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
 * @file test_ppm.cpp
 * Test for ppm channel values.
 */

#include <stdio.h>

#include "tests_main.h"


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
