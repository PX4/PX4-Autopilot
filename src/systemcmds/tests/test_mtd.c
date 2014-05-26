/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file test_mtd.c
 *
 * Param storage / file test.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <sys/stat.h>
#include <poll.h>
#include <dirent.h>
#include <stdio.h>
#include <stddef.h>
#include <unistd.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <string.h>

#include <drivers/drv_hrt.h>

#include "tests.h"

#define PARAM_FILE_NAME "/fs/mtd_params"

static int check_user_abort(int fd);
static void print_fail(void);
static void print_success(void);

int check_user_abort(int fd) {
	/* check if user wants to abort */
	char c;

	struct pollfd fds;
	int ret;
	fds.fd = 0; /* stdin */
	fds.events = POLLIN;
	ret = poll(&fds, 1, 0);

	if (ret > 0) {

		read(0, &c, 1);

		switch (c) {
		case 0x03: // ctrl-c
		case 0x1b: // esc
		case 'c':
		case 'q':
		{
			warnx("Test aborted.");
			fsync(fd);
			close(fd);
			return OK;
			/* not reached */
			}
		}
	}

	return 1;
}

void print_fail()
{
	printf("<[T]: MTD: FAIL>\n");
}

void print_success()
{
	printf("<[T]: MTD: OK>\n");
}

int
test_mtd(int argc, char *argv[])
{
	unsigned iterations= 0;

	/* check if microSD card is mounted */
	struct stat buffer;
	if (stat(PARAM_FILE_NAME, &buffer)) {
		warnx("file %s not found, aborting MTD test", PARAM_FILE_NAME);
		print_fail();
		return 1;
	}

	// XXX get real storage space here
	unsigned file_size = 4096;

	/* perform tests for a range of chunk sizes */
	unsigned chunk_sizes[] = {256, 512, 4096};

	for (unsigned c = 0; c < (sizeof(chunk_sizes) / sizeof(chunk_sizes[0])); c++) {

		printf("\n====== FILE TEST: %u bytes chunks ======\n", chunk_sizes[c]);

		uint8_t write_buf[chunk_sizes[c]] __attribute__((aligned(64)));

		/* fill write buffer with known values */
		for (unsigned i = 0; i < sizeof(write_buf); i++) {
			/* this will wrap, but we just need a known value with spacing */
			write_buf[i] = i+11;
		}

		uint8_t read_buf[chunk_sizes[c]] __attribute__((aligned(64)));
		hrt_abstime start, end;

		int fd = open(PARAM_FILE_NAME, O_RDONLY);
		int rret = read(fd, read_buf, chunk_sizes[c]);
		close(fd);
		if (rret <= 0) {
			err(1, "read error");
		}

		fd = open(PARAM_FILE_NAME, O_WRONLY);

		printf("printing 2 percent of the first chunk:\n");
		for (unsigned i = 0; i < sizeof(read_buf) / 50; i++) {
			printf("%02X", read_buf[i]);
		}
		printf("\n");

		iterations = file_size / chunk_sizes[c];

		start = hrt_absolute_time();
		for (unsigned i = 0; i < iterations; i++) {
			int wret = write(fd, write_buf, chunk_sizes[c]);

			if (wret != (int)chunk_sizes[c]) {
				warn("WRITE ERROR!");
				print_fail();
				return 1;
			}

			fsync(fd);

			if (!check_user_abort(fd))
				return OK;

		}
		end = hrt_absolute_time();

		close(fd);
		fd = open(PARAM_FILE_NAME, O_RDONLY);

		/* read back data for validation */
		for (unsigned i = 0; i < iterations; i++) {
			int rret2 = read(fd, read_buf, chunk_sizes[c]);

			if (rret2 != (int)chunk_sizes[c]) {
				warnx("READ ERROR!");
				print_fail();
				return 1;
			}
			
			/* compare value */
			bool compare_ok = true;

			for (unsigned j = 0; j < chunk_sizes[c]; j++) {
				if (read_buf[j] != write_buf[j]) {
					warnx("COMPARISON ERROR: byte %d", j);
					print_fail();
					compare_ok = false;
					break;
				}
			}

			if (!compare_ok) {
				warnx("ABORTING FURTHER COMPARISON DUE TO ERROR");
				print_fail();
				return 1;
			}

			if (!check_user_abort(fd))
				return OK;

		}


		close(fd);

	}

	/* fill the file with 0xFF to make it look new again */
	char ffbuf[64];
	memset(ffbuf, 0xFF, sizeof(ffbuf));
	int fd = open(PARAM_FILE_NAME, O_WRONLY);
	for (unsigned i = 0; i < file_size / sizeof(ffbuf); i++) {
		int ret = write(fd, ffbuf, sizeof(ffbuf));

		if (ret != sizeof(ffbuf)) {
			warnx("ERROR! Could not fill file with 0xFF");
			close(fd);
			print_fail();
			return 1;
		}
	}

	(void)close(fd);
	print_success();

	return 0;
}
