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
 * @file mixer.c
 *
 * Mixer utility.
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <drivers/drv_mixer.h>
#include <uORB/topics/actuator_controls.h>

__EXPORT int mixer_main(int argc, char *argv[]);

static void	usage(const char *reason);
static void	load(const char *devname, const char *fname);

int
mixer_main(int argc, char *argv[])
{
	if (argc < 2)
		usage("missing command");

	if (!strcmp(argv[1], "load")) {
		if (argc < 4)
			usage("missing device or filename");

		load(argv[2], argv[3]);

	} else {
		usage("unrecognised command");
	}

	return 0;
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage:\n");
	fprintf(stderr, "  mixer load <device> <filename>\n");
	/* XXX automatic setups for quad, etc. */
	exit(1);
}

static void
load(const char *devname, const char *fname)
{
	int		dev = -1;
	int		ret, result = 1;

	/* open the device */
	if ((dev = open(devname, 0)) < 0) {
		fprintf(stderr, "can't open %s\n", devname);
		goto out;
	}

	/* tell it to load the file */
	ret = ioctl(dev, MIXERIOCLOADFILE, (unsigned long)fname);

	if (ret != 0) {
		fprintf(stderr, "failed loading %s\n", fname);
	}

	result = 0;
out:

	if (dev != -1)
		close(dev);

	exit(result);
}
