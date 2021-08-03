/****************************************************************************
 *
 *  Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file test_file.c
 * File write test.
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>

#include <sys/stat.h>
#include <poll.h>
#include <dirent.h>
#include <stdio.h>
#include <stddef.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>
#include <string.h>

#include <drivers/drv_hrt.h>

#include "tests_main.h"

static int check_user_abort(int fd);

int check_user_abort(int fd)
{
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
		case 'q': {
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

int
test_file(int argc, char *argv[])
{
	const unsigned iterations = 100;
	const unsigned alignments = 65;

	/* check if microSD card is mounted */
	struct stat buffer;

	if (stat(PX4_STORAGEDIR "/", &buffer)) {
		warnx("no microSD card mounted, aborting file test");
		return 1;
	}

	/* perform tests for a range of chunk sizes */
	int chunk_sizes[] = {1, 5, 8, 13, 16, 32, 33, 64, 70, 128, 133, 256, 300, 512, 555, 1024, 1500};

	for (unsigned c = 0; c < (sizeof(chunk_sizes) / sizeof(chunk_sizes[0])); c++) {

		printf("\n====== FILE TEST: %u bytes chunks ======\n", chunk_sizes[c]);

		for (unsigned a = 0; a < alignments; a++) {

			printf("\n");
			warnx("----- alignment test: %u bytes -----", a);

			uint8_t write_buf[chunk_sizes[c] + alignments] __attribute__((aligned(64)));

			/* fill write buffer with known values */
			for (size_t i = 0; i < sizeof(write_buf); i++) {
				/* this will wrap, but we just need a known value with spacing */
				write_buf[i] = i + 11;
			}

			uint8_t read_buf[chunk_sizes[c] + alignments] __attribute__((aligned(64)));
			hrt_abstime start, end;

			int fd = px4_open(PX4_STORAGEDIR "/testfile", O_TRUNC | O_WRONLY | O_CREAT);

			warnx("testing unaligned writes - please wait..");

			start = hrt_absolute_time();

			for (unsigned i = 0; i < iterations; i++) {
				int wret = write(fd, write_buf + a, chunk_sizes[c]);

				if (wret != chunk_sizes[c]) {
					warn("WRITE ERROR!");

					if ((0x3 & (uintptr_t)(write_buf + a))) {
						warnx("memory is unaligned, align shift: %d", a);
					}

					return 1;
				}

				fsync(fd);

				if (!check_user_abort(fd)) {
					return OK;
				}

			}

			end = hrt_absolute_time();

			warnx("write took %" PRIu64 " us", (end - start));

			px4_close(fd);
			fd = open(PX4_STORAGEDIR "/testfile", O_RDONLY);

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

				if (!check_user_abort(fd)) {
					return OK;
				}

			}

			/*
			 * ALIGNED WRITES AND UNALIGNED READS
			 */

			close(fd);
			int ret = unlink(PX4_STORAGEDIR "/testfile");
			fd = px4_open(PX4_STORAGEDIR "/testfile", O_TRUNC | O_WRONLY | O_CREAT);

			warnx("testing aligned writes - please wait.. (CTRL^C to abort)");

			for (unsigned i = 0; i < iterations; i++) {
				int wret = write(fd, write_buf, chunk_sizes[c]);

				if (wret != chunk_sizes[c]) {
					warnx("WRITE ERROR!");
					return 1;
				}

				if (!check_user_abort(fd)) {
					return OK;
				}

			}

			fsync(fd);

			warnx("reading data aligned..");

			px4_close(fd);
			fd = open(PX4_STORAGEDIR "/testfile", O_RDONLY);

			bool align_read_ok = true;

			/* read back data unaligned */
			for (unsigned i = 0; i < iterations; i++) {
				int rret = read(fd, read_buf, chunk_sizes[c]);

				if (rret != chunk_sizes[c]) {
					warnx("READ ERROR!");
					return 1;
				}

				for (int j = 0; j < chunk_sizes[c]; j++) {
					if (read_buf[j] != write_buf[j]) {
						warnx("COMPARISON ERROR: byte %d: %u != %u", j, (unsigned int)read_buf[j], (unsigned int)write_buf[j]);
						align_read_ok = false;
						break;
					}

					if (!check_user_abort(fd)) {
						return OK;
					}
				}

				if (!align_read_ok) {
					warnx("ABORTING FURTHER COMPARISON DUE TO ERROR");
					return 1;
				}

			}

			warnx("align read result: %s\n", (align_read_ok) ? "OK" : "ERROR");

			warnx("reading data unaligned..");

			close(fd);
			fd = open(PX4_STORAGEDIR "/testfile", O_RDONLY);

			bool unalign_read_ok = true;
			int unalign_read_err_count = 0;

			memset(read_buf, 0, sizeof(read_buf));

			/* read back data unaligned */
			for (unsigned i = 0; i < iterations; i++) {
				int rret = read(fd, read_buf + a, chunk_sizes[c]);

				if (rret != chunk_sizes[c]) {
					warnx("READ ERROR!");
					return 1;
				}

				for (int j = 0; j < chunk_sizes[c]; j++) {

					if ((read_buf + a)[j] != write_buf[j]) {
						warnx("COMPARISON ERROR: byte %d, align shift: %d: %u != %u", j, a, (unsigned int)read_buf[j + a],
						      (unsigned int)write_buf[j]);
						unalign_read_ok = false;
						unalign_read_err_count++;

						if (unalign_read_err_count > 10) {
							break;
						}
					}

					if (!check_user_abort(fd)) {
						return OK;
					}
				}

				if (!unalign_read_ok) {
					warnx("ABORTING FURTHER COMPARISON DUE TO ERROR");
					return 1;
				}

			}

			ret = unlink(PX4_STORAGEDIR "/testfile");
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
	d = opendir(PX4_STORAGEDIR);

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

	return 0;
}
