
/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file hrt_test.cpp
 * Test High Resolution Timers in Linux
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

#include "hrt_test.h"

#include <drivers/drv_hrt.h>
#include <px4_log.h>
#include <px4_time.h>

#include <unistd.h>
#include <stdio.h>
#include <cstring>

px4::AppState HRTTest::appState;

static struct hrt_call t1;
static int update_interval = 1;

static void timer_expired(void *arg)
{
	static int i = 0;
	PX4_INFO("Test\n");

	if (i < 5) {
		i++;
		hrt_call_after(&t1, update_interval, timer_expired, (void *)nullptr);
	}
}

int HRTTest::main()
{
	appState.setRunning(true);

	hrt_abstime t = hrt_absolute_time();
	px4_usleep(1000000);
	hrt_abstime elt = hrt_elapsed_time(&t);
	PX4_INFO("Elapsed time %llu in 1 sec (usleep)\n", (unsigned long long)elt);
	PX4_INFO("Start time %llu\n", (unsigned long long)t);

	t = hrt_absolute_time();
	px4_sleep(1);
	elt = hrt_elapsed_time(&t);
	PX4_INFO("Elapsed time %llu in 1 sec (sleep)\n", (unsigned long long)elt);
	PX4_INFO("Start time %llu\n", (unsigned long long)t);

	memset(&t1, 0, sizeof(t1));

	PX4_INFO("HRT_CALL %d\n", hrt_called(&t1));

	hrt_call_after(&t1, update_interval, timer_expired, (void *)nullptr);
	px4_sleep(2);
	PX4_INFO("HRT_CALL - %d\n", hrt_called(&t1));
	hrt_cancel(&t1);
	PX4_INFO("HRT_CALL + %d\n", hrt_called(&t1));

	return 0;
}
