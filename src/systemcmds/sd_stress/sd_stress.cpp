/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file sd_stress.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>

static const unsigned MAX_PATH_LEN = 52;

static const char *TEMPDIR = PX4_STORAGEDIR"/stress";
static const char *TEMPDIR2 = PX4_STORAGEDIR"/moved";
static const char *TEMPFILE = "tmp";

static void usage()
{
	PRINT_MODULE_DESCRIPTION("Test operations on an SD Card");

	PRINT_MODULE_USAGE_NAME_SIMPLE("sd_stress", "command");
	PRINT_MODULE_USAGE_PARAM_INT('r', 5, 1, 10000, "Number of runs", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 100, 1, 10000, "Number of bytes", true);
}

static bool create_dir(const char *path)
{
	int ret = mkdir(TEMPDIR, S_IRWXU | S_IRWXG | S_IRWXO);

	if (ret < 0) {
		printf("mkdir %s failed, ret: %d, errno: %d -> %s\n", path, ret, errno, strerror(errno));
		return false;
	}

	return true;
}

static bool remove_dir(const char *path)
{
	int ret = rmdir(TEMPDIR2);

	if (ret < 0) {
		printf("rmdir %s failed, ret: %d, errno: %d -> %s\n", path, ret, errno, strerror(errno));
		return false;
	}

	return true;
}

static bool create_files(const char *dir, const char *name, unsigned num_files, const char *bytes, unsigned num_bytes)
{
	if (num_files > 999) {
		printf("too many files\n");
		return false;
	}

	unsigned path_len = strlen(dir) + strlen(name);

	if (path_len + 5 >= MAX_PATH_LEN) {
		printf("path name too long\n");
		return false;
	}

	for (unsigned i = 0; i < num_files; ++i) {
		char path[MAX_PATH_LEN];
		snprintf(path, MAX_PATH_LEN, "%s/%s%03u", dir, name, i);

		int fd = open(path, O_CREAT | O_WRONLY);

		if (fd < 0) {
			printf("open %s failed, errno: %d -> %s\n", path, errno, strerror(errno));
			return false;
		}

		int ret = write(fd, bytes, num_bytes);

		if (ret != (int)num_bytes) {
			printf("write %s failed, ret: %d, errno %d -> %s\n", path, ret, errno, strerror(errno));
			return false;
		}

		ret = close(fd);

		if (ret < 0) {
			printf("close %s failed, ret: %d, errno %d -> %s\n", path, ret, errno, strerror(errno));
			return false;
		}
	}

	return true;
}

static bool remove_files(const char *dir, const char *name, unsigned num_files)
{
	if (num_files > 999) {
		printf("too many files\n");
		return false;
	}

	unsigned path_len = strlen(dir) + strlen(name);

	if (path_len + 5 >= MAX_PATH_LEN) {
		printf("path name too long\n");
		return false;
	}

	for (unsigned i = 0; i < num_files; ++i) {
		char path[MAX_PATH_LEN];
		snprintf(path, MAX_PATH_LEN, "%s/%s%03u", dir, name, i);

		int ret = unlink(path);

		if (ret < 0) {
			printf("unlink %s failed, ret: %d, errno %d -> %s\n", path, ret, errno, strerror(errno));
			return false;
		}
	}

	return true;
}

static bool rename_dir(const char *old_dir, const char *new_dir)
{
	int ret = rename(old_dir, new_dir);

	if (ret < 0) {
		printf("rename %s to %s failed, ret: %d, errno %d -> %s\n", old_dir, new_dir, ret, errno, strerror(errno));
		return false;
	}

	return true;
}

extern "C" __EXPORT int sd_stress_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	unsigned num_runs = 5;
	unsigned num_bytes = 100;

	while ((ch = px4_getopt(argc, argv, "r:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'r':
			num_runs = strtol(myoptarg, nullptr, 0);
			break;

		case 'b':
			num_bytes = strtol(myoptarg, nullptr, 0);
			break;

		default:
			usage();
			return -1;
			break;
		}
	}

	int ret = 0;

	char *bytes = (char *)malloc(num_bytes);
	memset(bytes, 0xAA, num_bytes);

	for (unsigned i = 0; i < num_runs; ++i) {
		const uint64_t start_time = hrt_absolute_time();

		const bool result =
			create_dir(TEMPDIR)
			&& create_files(TEMPDIR, TEMPFILE, 100, bytes, num_bytes)
			&& rename_dir(TEMPDIR, TEMPDIR2)
			&& remove_files(TEMPDIR2, TEMPFILE, 100)
			&& remove_dir(TEMPDIR2);

		printf("iteration %u took %06" PRIu64 " us: %s\n", i, hrt_absolute_time() - start_time, result ? "OK" : "FAIL");

		if (!result) {
			ret = -1;
			break;
		}
	}

	free(bytes);
	return ret;
}
