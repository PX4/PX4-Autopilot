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
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <string.h>

#include <drivers/drv_hrt.h>

#include "tests.h"

const int fsync_tries = 1;
const int abort_tries = 10;

int
test_mount(int argc, char *argv[])
{
	const unsigned iterations = 2000;
	const unsigned alignments = 10;

	const char *cmd_filename = "/fs/microsd/mount_test_cmds.txt";


	/* check if microSD card is mounted */
	struct stat buffer;

	if (stat("/fs/microsd/", &buffer)) {
		warnx("no microSD card mounted, aborting file test");
		return 1;
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

		if (stat(cmd_filename, &buffer) == OK) {
			(void)unlink(cmd_filename);
		}

		return 1;
	}

	/* read current test status from file, write test instructions for next round */

	/* initial values */
	int it_left_fsync = fsync_tries;
	int it_left_abort = abort_tries;

	int cmd_fd;

	if (stat(cmd_filename, &buffer) == OK) {

		/* command file exists, read off state */
		cmd_fd = open(cmd_filename, O_RDWR | O_NONBLOCK);
		char buf[64];
		int ret = read(cmd_fd, buf, sizeof(buf));

		if (ret > 0) {
			int count = 0;
			ret = sscanf(buf, "TEST: %u %u %n", &it_left_fsync, &it_left_abort, &count);

		} else {
			buf[0] = '\0';
		}

		if (it_left_fsync > fsync_tries) {
			it_left_fsync = fsync_tries;
		}

		if (it_left_abort > abort_tries) {
			it_left_abort = abort_tries;
		}

		warnx("Iterations left: #%d / #%d of %d / %d\n(%s)", it_left_fsync, it_left_abort,
		      fsync_tries, abort_tries, buf);

		int it_left_fsync_prev = it_left_fsync;

		/* now write again what to do next */
		if (it_left_fsync > 0) {
			it_left_fsync--;
		}

		if (it_left_fsync == 0 && it_left_abort > 0) {

			it_left_abort--;

			/* announce mode switch */
			if (it_left_fsync_prev != it_left_fsync && it_left_fsync == 0) {
				warnx("\n SUCCESSFULLY PASSED FSYNC'ED WRITES, CONTINUTING WITHOUT FSYNC");
				fsync(fileno(stdout));
				fsync(fileno(stderr));
				usleep(20000);
			}

		}

		if (it_left_abort == 0) {
			(void)unlink(cmd_filename);
			return 0;
		}

	} else {

		/* this must be the first iteration, do something */
		cmd_fd = open(cmd_filename, O_TRUNC | O_WRONLY | O_CREAT);

		warnx("First iteration of file test\n");
	}

	char buf[64];
	(void)sprintf(buf, "TEST: %d %d ", it_left_fsync, it_left_abort);
	lseek(cmd_fd, 0, SEEK_SET);
	write(cmd_fd, buf, strlen(buf) + 1);
	fsync(cmd_fd);

	/* perform tests for a range of chunk sizes */
	unsigned chunk_sizes[] = {32, 64, 128, 256, 512, 600, 1200};

	for (unsigned c = 0; c < (sizeof(chunk_sizes) / sizeof(chunk_sizes[0])); c++) {

		printf("\n\n====== FILE TEST: %u bytes chunks (%s) ======\n", chunk_sizes[c],
		       (it_left_fsync > 0) ? "FSYNC" : "NO FSYNC");
		printf("unpower the system immediately (within 0.5s) when the hash (#) sign appears\n");
		fsync(fileno(stdout));
		fsync(fileno(stderr));
		usleep(50000);

		for (unsigned a = 0; a < alignments; a++) {

			printf(".");

			uint8_t write_buf[chunk_sizes[c] + alignments] __attribute__((aligned(64)));

			/* fill write buffer with known values */
			for (unsigned i = 0; i < sizeof(write_buf); i++) {
				/* this will wrap, but we just need a known value with spacing */
				write_buf[i] = i + 11;
			}

			uint8_t read_buf[chunk_sizes[c] + alignments] __attribute__((aligned(64)));

			int fd = open("/fs/microsd/testfile", O_TRUNC | O_WRONLY | O_CREAT);

			for (unsigned i = 0; i < iterations; i++) {

				int wret = write(fd, write_buf + a, chunk_sizes[c]);

				if (wret != (int)chunk_sizes[c]) {
					warn("WRITE ERROR!");

					if ((0x3 & (uintptr_t)(write_buf + a))) {
						warnx("memory is unaligned, align shift: %d", a);
					}

					return 1;

				}

				if (it_left_fsync > 0) {
					fsync(fd);

				} else {
					printf("#");
					fsync(fileno(stdout));
					fsync(fileno(stderr));
				}
			}

			if (it_left_fsync > 0) {
				printf("#");
			}

			printf(".");
			fsync(fileno(stdout));
			fsync(fileno(stderr));
			usleep(200000);

			close(fd);
			fd = open("/fs/microsd/testfile", O_RDONLY);

			/* read back data for validation */
			for (unsigned i = 0; i < iterations; i++) {
				int rret = read(fd, read_buf, chunk_sizes[c]);

				if (rret != (int)chunk_sizes[c]) {
					warnx("READ ERROR!");
					return 1;
				}

				/* compare value */
				bool compare_ok = true;

				for (unsigned j = 0; j < chunk_sizes[c]; j++) {
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

	fsync(fileno(stdout));
	fsync(fileno(stderr));
	usleep(20000);



	/* we always reboot for the next test if we get here */
	warnx("Iteration done, rebooting..");
	fsync(fileno(stdout));
	fsync(fileno(stderr));
	usleep(50000);
	systemreset(false);

	/* never going to get here */
	return 0;
}
