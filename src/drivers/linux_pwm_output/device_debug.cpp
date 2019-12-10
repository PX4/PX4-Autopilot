/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file device_debug.cpp
 * Example driver
 * This driver will dump all messages passed to it.
 *
 * @author SalimTerryLi <lhf2613@gmail.com>
 */

#include "device_debug.h"

int device_debug::deviceConfigure(int argc, char **argv)
{
	PX4_INFO("dump cmdline args:");
	int ch;
	optind = 1; // reset scan index
	opterr = 0; // turn off parse error output

	while ((ch = getopt(argc, argv, "abcdefghijklmnopqrstuvwxyz:")) != -1) {
		PX4_INFO("arg: %c", ch);
	}

	optind = 1; // reset scan index
	PX4_INFO("dump complete");
	return 0;
}

int device_debug::deviceInit()
{
	PX4_INFO("driver init");
	return 0;
}

int device_debug::deviceDeinit()
{
	PX4_INFO("driver removed");
	return 0;
}

int device_debug::setFreq(int freq)
{
	PX4_INFO("set pwm frequency: %d", freq);
	return 0;
}

int device_debug::updatePWM(const uint16_t *outputs, unsigned num_outputs)
{
	printf("PWM:");

	for (uint16_t i = 0; i < num_outputs; i++) {
		printf(" %.4d", outputs[i]);
	}

	printf("\n");
	return 0;
}
