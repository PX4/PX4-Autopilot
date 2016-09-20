/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file rtd.c
 *
 * rtd service and utility app.
 *
 * @author David Sidrane <david_s5@nscdg.com>
 */

#include <px4_config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/stat.h>
#include <px4_log.h>


#include <arch/board/board.h>

#include <board_config.h>

__EXPORT int rtd_main(int argc, char *argv[]);

/****************************************************************************
 * Name: dd_infopen
 ****************************************************************************/

static int rtd_filetype(const char *filename)
{
	struct stat sb;
	int ret;

	/* Get the type of the file */

	ret = stat(filename, &sb);

	if (ret < 0) {
		return ret;  /* Return -1 on failure */
	}

	return S_ISBLK(sb.st_mode); /* Return true(1) if block, false(0) if char */
}

static void usage(void)
{
	PX4_WARN(
		"rtd Usage:\n"
		"rtd start <ramdisk path> <char dev path>\n"
	);

}

int rtd_main(int argc, char *argv[])
{
	int ret = -1;

	if (argc == 4) {
		if (!strcmp(argv[1], "start")) {
			ret = rtd_filetype(argv[2]);

			if (ret < 0) {
				PX4_WARN("%s Does not exist", argv[2]);
				ret = -1;

			} else {
				ret = rtd_filetype(argv[3]);

				if (ret >= 0) {
					PX4_WARN("%s File exists", argv[3]);
				}

				ret = bchdev_register(argv[2], argv[3], false);

				if (ret < 0) {
					PX4_WARN("Failed to create %s on %s", argv[2], argv[3]);
				}

				return ret;
			}
		}
	}

	usage();
	return -1;
}
