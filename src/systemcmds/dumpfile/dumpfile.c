/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file dumpfile.c
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>

#include <systemlib/err.h>

__EXPORT int dumpfile_main(int argc, char *argv[]);

static void print_usage(void)
{
	PRINT_MODULE_DESCRIPTION("Dump file utility. Prints file size and contents in binary mode (don't replace LF with CR LF) to stdout.");

	PRINT_MODULE_USAGE_NAME_SIMPLE("dumpfile", "command");
	PRINT_MODULE_USAGE_ARG("<file>", "File to dump", false);

}

int
dumpfile_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return 1;
	}

	/* open input file */
	FILE *f;
	f = fopen(argv[1], "r");

	if (f == NULL) {
		PX4_ERR("Failed to open file (%i)", errno);
		return 1;
	}

	/* get file size */
	fseek(f, 0L, SEEK_END);
	int size = ftell(f);
	fseek(f, 0L, SEEK_SET);

	printf("File size: %d bytes\n", size);

	/* configure stdout */
	int out = fileno(stdout);

	struct termios tc;
	struct termios tc_old;
	tcgetattr(out, &tc);

	/* save old terminal attributes to restore it later on exit */
	memcpy(&tc_old, &tc, sizeof(tc));

	/* don't add CR on each LF*/
	tc.c_oflag &= ~ONLCR;

	if (tcsetattr(out, TCSANOW, &tc) < 0) {
		PX4_ERR("failed setting stdout attributes");
		fclose(f);
		return 1;
	}

	char buf[512];
	int nread;

	/* dump file */
	while ((nread = fread(buf, 1, sizeof(buf), f)) > 0) {
		if (write(out, buf, nread) <= 0) {
			PX4_ERR("write failed");
			break;
		}
	}

	fsync(out);
	fclose(f);

	/* restore old terminal attributes */
	if (tcsetattr(out, TCSANOW, &tc_old) < 0) {
		PX4_ERR("failed to restore stdout attributes");
		return 1;
	}

	return 0;
}
