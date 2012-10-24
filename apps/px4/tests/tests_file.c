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
 * @file tests_file.c
 *
 * File write test.
 */

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <string.h>

#include <drivers/drv_hrt.h>

#include "tests.h"

int
test_file(int argc, char *argv[])
{
	uint8_t buf[512];
	hrt_abstime start, end;
	perf_counter_t wperf = perf_alloc(PC_ELAPSED, "SD writes");

	int fd = open("/fs/microsd/testfile", O_TRUNC | O_WRONLY | O_CREAT);
	memset(buf, 0, sizeof(buf));

	start = hrt_absolute_time();
	for (unsigned i = 0; i < 1024; i++) {
		perf_begin(wperf);
		write(fd, buf, sizeof(buf));
		perf_end(wperf);
	}
	end = hrt_absolute_time();

	close(fd);

	warnx("512KiB in %llu microseconds", end - start);
	perf_print_counter(wperf);
	perf_free(wperf);

	return 0;
}
