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
test_file(int argc, char *argv[])
{
	const unsigned iterations = 100;
	const unsigned alignments = 65;

	/* check if microSD card is mounted */
	struct stat buffer;
	if (stat("/fs/microsd/", &buffer)) {
		warnx("no microSD card mounted, aborting file test");
		return 1;
	}

	/* perform tests for a range of chunk sizes */
	unsigned chunk_sizes[] = {1, 5, 8, 13, 16, 32, 33, 64, 70, 128, 133, 256, 300, 512, 555, 1024, 1500};

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

				fsync(fd);
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
					errx(1, "READ ERROR!");
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
					errx(1, "ABORTING FURTHER COMPARISON DUE TO ERROR");
				}

			}

			/*
			 * ALIGNED WRITES AND UNALIGNED READS
			 */

			close(fd);
			int ret = unlink("/fs/microsd/testfile");
			fd = open("/fs/microsd/testfile", O_TRUNC | O_WRONLY | O_CREAT);

			warnx("testing aligned writes - please wait..");

			start = hrt_absolute_time();
			for (unsigned i = 0; i < iterations; i++) {
				int wret = write(fd, write_buf, chunk_sizes[c]);

				if (wret != chunk_sizes[c]) {
					err(1, "WRITE ERROR!");
				}

			}

			fsync(fd);

			warnx("reading data aligned..");

			close(fd);
			fd = open("/fs/microsd/testfile", O_RDONLY);

			bool align_read_ok = true;

			/* read back data unaligned */
			for (unsigned i = 0; i < iterations; i++) {
				int rret = read(fd, read_buf, chunk_sizes[c]);

				if (rret != chunk_sizes[c]) {
					err(1, "READ ERROR!");
				}
				
				/* compare value */
				bool compare_ok = true;

				for (int j = 0; j < chunk_sizes[c]; j++) {
					if (read_buf[j] != write_buf[j]) {
						warnx("COMPARISON ERROR: byte %d: %u != %u", j, (unsigned int)read_buf[j], (unsigned int)write_buf[j]);
						align_read_ok = false;
						break;
					}
				}

				if (!align_read_ok) {
					errx(1, "ABORTING FURTHER COMPARISON DUE TO ERROR");
				}

			}

			warnx("align read result: %s\n", (align_read_ok) ? "OK" : "ERROR");

			warnx("reading data unaligned..");

			close(fd);
			fd = open("/fs/microsd/testfile", O_RDONLY);

			bool unalign_read_ok = true;
			int unalign_read_err_count = 0;

			memset(read_buf, 0, sizeof(read_buf));

			/* read back data unaligned */
			for (unsigned i = 0; i < iterations; i++) {
				int rret = read(fd, read_buf + a, chunk_sizes[c]);

				if (rret != chunk_sizes[c]) {
					err(1, "READ ERROR!");
				}

				for (int j = 0; j < chunk_sizes[c]; j++) {

					if ((read_buf + a)[j] != write_buf[j]) {
						warnx("COMPARISON ERROR: byte %d, align shift: %d: %u != %u", j, a, (unsigned int)read_buf[j + a], (unsigned int)write_buf[j]);
						unalign_read_ok = false;
						unalign_read_err_count++;
						
						if (unalign_read_err_count > 10)
							break;
					}
				}

				if (!unalign_read_ok) {
					errx(1, "ABORTING FURTHER COMPARISON DUE TO ERROR");
				}

			}

			ret = unlink("/fs/microsd/testfile");
			close(fd);

			if (ret)
				err(1, "UNLINKING FILE FAILED");

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
		err(1, "FAILED LISTING MICROSD ROOT DIRECTORY");
	}

	return 0;
}
#if 0
int
test_file(int argc, char *argv[])
{
	const iterations = 1024;

	/* check if microSD card is mounted */
	struct stat buffer;
	if (stat("/fs/microsd/", &buffer)) {
		warnx("no microSD card mounted, aborting file test");
		return 1;
	}

	uint8_t buf[512];
	hrt_abstime start, end;
	perf_counter_t wperf = perf_alloc(PC_ELAPSED, "SD writes");

	int fd = open("/fs/microsd/testfile", O_TRUNC | O_WRONLY | O_CREAT);
	memset(buf, 0, sizeof(buf));

	start = hrt_absolute_time();
	for (unsigned i = 0; i < iterations; i++) {
		perf_begin(wperf);
		write(fd, buf, sizeof(buf));
		perf_end(wperf);
	}
	end = hrt_absolute_time();

	close(fd);

	unlink("/fs/microsd/testfile");

	warnx("%dKiB in %llu microseconds", iterations / 2, end - start);
	perf_print_counter(wperf);
	perf_free(wperf);

	warnx("running unlink test");

	/* ensure that common commands do not run against file count limits */
	for (unsigned i = 0; i < 64; i++) {

		warnx("unlink iteration #%u", i);

		int fd = open("/fs/microsd/testfile", O_TRUNC | O_WRONLY | O_CREAT);
		if (fd < 0)
			errx(1, "failed opening test file before unlink()");
		int ret = write(fd, buf, sizeof(buf));
		if (ret < 0)
			errx(1, "failed writing test file before unlink()");
		close(fd);

		ret = unlink("/fs/microsd/testfile");
		if (ret != OK)
			errx(1, "failed unlinking test file");

		fd = open("/fs/microsd/testfile", O_TRUNC | O_WRONLY | O_CREAT);
		if (fd < 0)
			errx(1, "failed opening test file after unlink()");
		ret = write(fd, buf, sizeof(buf));
		if (ret < 0)
			errx(1, "failed writing test file after unlink()");
		close(fd);
	}

	return 0;
}
#endif
