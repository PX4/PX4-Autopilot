/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/defines.h>

#include <stdio.h>
#include <unistd.h>

#include <DevMgr.hpp>

#include <drivers/drv_rgbled.h>

#include "navio_rgbled.h"

using namespace DriverFramework;

int do_test();

int do_test()
{
	DevHandle h;
	RGBLED *g_dev = nullptr;

	if (Framework::initialize() < 0) {
		printf("Framework init failed\n");
		return -1;
	}

	g_dev = new RGBLED("navio_rgbled test");
	g_dev->start();

	DevMgr::getHandle(RGBLED0_DEVICE_PATH, h);

	if (!h.isValid()) {
		printf("No RGB LED at " RGBLED0_DEVICE_PATH);
		return -1;
	}

	printf("off\n");
	h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_OFF);
	sleep(2);

	printf("red\n");
	h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_RED);
	sleep(2);

	printf("yellow\n");
	h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_YELLOW);
	sleep(2);

	printf("purple\n");
	h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_PURPLE);
	sleep(2);

	printf("green\n");
	h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_GREEN);
	sleep(2);

	printf("blue\n");
	h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_BLUE);
	sleep(2);

	printf("blue blink slow\n");
	h.ioctl(RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_BLINK_SLOW);
	sleep(10);

	printf("green blink normal\n");
	h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_GREEN);
	h.ioctl(RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_BLINK_NORMAL);
	sleep(10);

	printf("red blink fast\n");
	h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_RED);
	h.ioctl(RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_BLINK_FAST);
	sleep(10);

	printf("blue breathe (bogus)\n");
	h.ioctl(RGBLED_SET_COLOR, (unsigned long)RGBLED_COLOR_BLUE);
	h.ioctl(RGBLED_SET_MODE, (unsigned long)RGBLED_MODE_BREATHE);
	sleep(10);

	return 0;
}
