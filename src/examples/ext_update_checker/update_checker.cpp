/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file update_checker.cpp
 * fake update checker application to test external component updates preflight checks.
 *
 * @author Claudio Micheli <claudio@auterion.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <sys/stat.h>

extern "C" __EXPORT int update_checker_main(int argc, char *argv[]);

static constexpr char ext_component_path[] = PX4_STORAGEDIR "/ext_component_updated";
static constexpr int FLASH_DURATION = 15;

static void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: update_checker {fake_success|fake_fail} \n\n");
}

bool file_exists(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer) == 0;
}

void fake_update(const char flash_result[], uint8_t return_code)
{
	FILE *fp = fopen(ext_component_path, "w");

	if (fp == nullptr) {
		PX4_ERR("Error unable to open file");
		return;
	}

	PX4_INFO("Start fake flashing, duration: %d s", FLASH_DURATION);
	sleep(FLASH_DURATION);

	PX4_INFO("Fake flashing done %s", flash_result);
	fprintf(fp, "%s %d\n", flash_result, return_code);
	fclose(fp);
}

int update_checker_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!file_exists(ext_component_path)) {

		if (!strcmp(argv[1], "fake_success")) {
			fake_update("SUCCESS", 0);
		}

		if (!strcmp(argv[1], "fake_fail")) {
			fake_update("FAIL", (rand() % 10) + 1);
		}

	} else {
		PX4_INFO("Update file already exists - Nothing to do");
	}

	PX4_INFO("exiting");

	return 0;
}
