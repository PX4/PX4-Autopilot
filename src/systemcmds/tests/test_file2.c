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
 * @file test_file2.c
 * File write test.
 */

#include <px4_platform_common/defines.h>
#include <sys/stat.h>
#include <dirent.h>
#include <inttypes.h>
#include <stdio.h>
#include <stddef.h>
#include <unistd.h>
#include <fcntl.h>
#include <perf/perf_counter.h>
#include <string.h>
#include <stdlib.h>
#include <px4_platform_common/getopt.h>

#include "tests_main.h"

#define FLAG_FSYNC 1
#define FLAG_LSEEK 2

#define LOG_PATH PX4_STORAGEDIR

/*
  return a predictable value for any file offset to allow detection of corruption
 */
static uint8_t get_value(uint32_t ofs)
{
	union {
		uint32_t ofs;
		uint8_t buf[4];
	} u;
	u.ofs = ofs;
	return u.buf[ofs % 4];
}

static int test_corruption(const char *filename, uint32_t write_chunk, uint32_t write_size, uint16_t flags)
{
	printf("Testing on %s with write_chunk=%" PRIu32 " write_size=%" PRIu32 "\n",
	       filename, write_chunk, write_size);

	uint32_t ofs = 0;
	int fd = open(filename, O_CREAT | O_RDWR | O_TRUNC, PX4_O_MODE_666);

	if (fd == -1) {
		perror(filename);
		return 1;
	}

	// create a file of size write_size, in write_chunk blocks
	uint8_t counter = 0;

	while (ofs < write_size) {
		uint8_t buffer[write_chunk];

		for (uint16_t j = 0; j < write_chunk; j++) {
			buffer[j] = get_value(ofs);
			ofs++;
		}

		if (write(fd, buffer, sizeof(buffer)) != (int)sizeof(buffer)) {
			printf("write failed at offset %" PRIu32 "\n", ofs);
			return 1;
		}

		if (flags & FLAG_FSYNC) {
			fsync(fd);
		}

		if (counter % 100 == 0) {
			printf("write ofs=%" PRIu32 "\r", ofs);
		}

		counter++;
	}

	close(fd);

	printf("write ofs=%" PRIu32 "\n", ofs);

	// read and check
	fd = open(filename, O_RDONLY);

	if (fd == -1) {
		perror(filename);
		return 1;
	}

	counter = 0;
	ofs = 0;

	while (ofs < write_size) {
		uint8_t buffer[write_chunk];

		if (counter % 100 == 0) {
			printf("read ofs=%" PRIu32 "\r", ofs);
		}

		counter++;

		if (read(fd, buffer, sizeof(buffer)) != (int)sizeof(buffer)) {
			printf("read failed at offset %" PRIu32 "\n", ofs);
			close(fd);
			return 1;
		}

		for (uint16_t j = 0; j < write_chunk; j++) {
			if (buffer[j] != get_value(ofs)) {
				printf("corruption at ofs=%" PRIu32 " got %" PRIu8 "\n", ofs, buffer[j]);
				close(fd);
				return 1;
			}

			ofs++;
		}

		if (flags & FLAG_LSEEK) {
			lseek(fd, 0, SEEK_CUR);
		}
	}

	printf("read ofs=%" PRIu32 "\n", ofs);
	close(fd);
	unlink(filename);
	printf("All OK\n");
	return 0;
}

static void usage(void)
{
	printf("test file2 [options] [filename]\n");
	printf("\toptions:\n");
	printf("\t-s SIZE     set file size\n");
	printf("\t-c CHUNK    set IO chunk size\n");
	printf("\t-F          fsync on every write\n");
	printf("\t-L          lseek on every read\n");
}

int test_file2(int argc, char *argv[])
{
	int opt;
	uint16_t flags = 0;
	const char *filename = LOG_PATH "/testfile2.dat";
	uint32_t write_chunk = 64;
	uint32_t write_size = 5 * 1024;

	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((opt = px4_getopt(argc, argv, "c:s:FLh", &myoptind, &myoptarg)) != EOF) {
		switch (opt) {
		case 'F':
			flags |= FLAG_FSYNC;
			break;

		case 'L':
			flags |= FLAG_LSEEK;
			break;

		case 's':
			write_size = strtoul(myoptarg, NULL, 0);
			break;

		case 'c':
			write_chunk = strtoul(myoptarg, NULL, 0);
			break;

		case 'h':
		default:
			usage();
			return 1;
		}
	}

	argc -= myoptind;
	argv += myoptind;

	if (argc > 0) {
		filename = argv[0];
	}

	/* check if microSD card is mounted */
	struct stat buffer;

	if (stat(LOG_PATH, &buffer)) {
		fprintf(stderr, "no microSD card mounted, aborting file test");
		return 1;
	}

	return test_corruption(filename, write_chunk, write_size, flags);
}
