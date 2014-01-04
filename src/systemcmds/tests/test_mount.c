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
 * @file test_mount.c
 *
 * Device mount / unmount stress test
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <sys/stat.h>
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

int
test_mount(int argc, char *argv[])
{
	const unsigned iterations = 100;
	const unsigned alignments = 65;

	/* check if microSD card is mounted */
	struct stat buffer;
	if (stat("/fs/microsd/", &buffer)) {
		warnx("no microSD card mounted, aborting file test");
		return 1;
	}

	/* read current test status from file, write test instructions for next round */
	const char* cmd_filename = "/fs/microsd/mount_test_cmds";

	/* initial values */
	int it_left_fsync = 100;
	int it_left_abort = 100;

	int cmd_fd;
	if (stat(cmd_filename, &buffer)) {

		/* command file exists, read off state */
		cmd_fd = open(cmd_filename, O_RDWR);
		char buf[64];
		int ret = read(cmd_fd, buf, sizeof(buf));
		if (ret > 0)
			ret = sscanf("%d %d", &it_left_fsync, &it_left_abort);

		warnx("Iterations left: #%d / #%d\n(%s)", it_left_fsync, it_left_abort, buf);

		/* now write again what to do next */
		if (it_left_fsync > 0)
			it_left_fsync--;
		if (it_left_fsync == 0 && it_left_abort > 0)
			it_left_abort--;

		if (it_left_abort == 0)
			(void)unlink(cmd_filename);
			return 0;

	} else {

		/* this must be the first iteration, do something */
		cmd_fd = open(cmd_filename, O_TRUNC | O_WRONLY | O_CREAT);
	}

	char buf[64];
	sprintf(buf, "%d %d", it_left_fsync, it_left_abort);
	write(cmd_fd, buf, strlen(buf) + 1);	

	/* perform tests for a range of chunk sizes */
	unsigned chunk_sizes[] = {1, 5, 8, 13, 16, 32};

	for (unsigned c = 0; c < (sizeof(chunk_sizes) / sizeof(chunk_sizes[0])); c++) {

		printf("\n====== FILE TEST: %u bytes chunks ======\n", chunk_sizes[c]);

		for (unsigned a = 0; a < alignments; a++) {

			printf("\n");
			warnx("----- alignment test: %u bytes -----", a);

			uint8_t write_buf[chunk_sizes[c] + alignments] __attribute__((aligned(64)));

			/* fill write buffer with known values */
			for (int i = 0; i < sizeof(write_buf); i++) {
				/* this will wrap, but we just need a known value with spacing */
				write_buf[i] = i+11;
			}

			uint8_t read_buf[chunk_sizes[c] + alignments] __attribute__((aligned(64)));
			hrt_abstime start, end;
			//perf_counter_t wperf = perf_alloc(PC_ELAPSED, "SD writes (aligned)");

			int fd = open("/fs/microsd/testfile", O_TRUNC | O_WRONLY | O_CREAT);

			warnx("testing unaligned writes - please wait..");

			start = hrt_absolute_time();
			for (unsigned i = 0; i < iterations; i++) {
				//perf_begin(wperf);
				int wret = write(fd, write_buf + a, chunk_sizes[c]);

				if (wret != chunk_sizes[c]) {
					warn("WRITE ERROR!");

					if ((0x3 & (uintptr_t)(write_buf + a)))
						errx(1, "memory is unaligned, align shift: %d", a);

				}

				if (it_left_fsync > 0) {
					fsync(fd);
				} else {
					if (it_left_abort % chunk_sizes[c] == 0) {
						systemreset();
					}
				}
				//perf_end(wperf);

			}
			end = hrt_absolute_time();

			//warnx("%dKiB in %llu microseconds", iterations / 2, end - start);

			//perf_print_counter(wperf);
			//perf_free(wperf);

			close(fd);
			fd = open("/fs/microsd/testfile", O_RDONLY);

			/* read back data for validation */
			for (unsigned i = 0; i < iterations; i++) {
				int rret = read(fd, read_buf, chunk_sizes[c]);

				if (rret != chunk_sizes[c]) {
					warnx("READ ERROR!");
					return 1;
				}
				
				/* compare value */
				bool compare_ok = true;

				for (int j = 0; j < chunk_sizes[c]; j++) {
					if (read_buf[j] != write_buf[j + a]) {
						warnx("COMPARISON ERROR: byte %d, align shift: %d", j, a);
						compare_ok = false;
						break;
					}
				}

				if (!compare_ok) {
					warnx("ABORTING FURTHER COMPARISON DUE TO ERROR");
					return 1;
				}

			}

			int ret = unlink("/fs/microsd/testfile");
			close(fd);

			if (ret) {
				warnx("UNLINKING FILE FAILED");
				return 1;
			}

		}
	}

	/* list directory */
	DIR		*d;
	struct dirent	*dir;
	d = opendir("/fs/microsd");
	if (d) {

		while ((dir = readdir(d)) != NULL) {
			//printf("%s\n", dir->d_name);
		}

		closedir(d);

		warnx("directory listing ok (FS mounted and readable)");

	} else {
		/* failed opening dir */
		warnx("FAILED LISTING MICROSD ROOT DIRECTORY");
		return 1;
	}

	/* we always reboot for the next test if we get here */
	systemreset();

	/* never going to get here */
	return 0;
}
