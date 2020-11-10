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
 * @file mixer.cpp
 *
 * Mixer utility.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>

#include <drivers/drv_mixer.h>
#include <lib/mixer/MixerGroup.hpp>
#include <lib/mixer/mixer_load.h>
#include <uORB/topics/actuator_controls.h>

/**
 * Mixer utility for loading mixer files to devices
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mixer_main(int argc, char *argv[]);

static void	usage(const char *reason);
static int	load(const char *devname, const char *fname, bool append);

int
mixer_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "load")) {
		if (argc < 4) {
			usage("missing device or filename");
			return 1;
		}

		int ret = load(argv[2], argv[3], false);

		if (ret != 0) {
			PX4_ERR("failed to load mixer");
			return 1;
		}

	} else if (!strcmp(argv[1], "append")) {
		if (argc < 4) {
			usage("missing device or filename");
			return 1;
		}

		int ret = load(argv[2], argv[3], true);

		if (ret != 0) {
			PX4_ERR("failed to append mixer");
			return 1;
		}

	} else {
		usage("Unknown command");
		return 1;
	}

	return 0;
}

static void
usage(const char *reason)
{
	if (reason && *reason) {
		PX4_INFO("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Load or append mixer files to the ESC driver.

Note that the driver must support the used ioctl's, which is the case on NuttX, but for example not on RPi.
)DESCR_STR");


	PRINT_MODULE_USAGE_NAME("mixer", "command");

	PRINT_MODULE_USAGE_COMMAND("load");
	PRINT_MODULE_USAGE_ARG("<file:dev> <file>", "Output device (eg. /dev/pwm_output0) and mixer file", false);
	PRINT_MODULE_USAGE_COMMAND("append");
	PRINT_MODULE_USAGE_ARG("<file:dev> <file>", "Output device (eg. /dev/pwm_output0) and mixer file", false);
}

static int
load(const char *devname, const char *fname, bool append)
{
	// sleep a while to ensure device has been set up
	px4_usleep(20000);

	int dev;

	/* open the device */
	if ((dev = px4_open(devname, 0)) < 0) {
		PX4_ERR("can't open %s\n", devname);
		return 1;
	}

	/* reset mixers on the device, but not if appending */
	if (!append) {
		if (px4_ioctl(dev, MIXERIOCRESET, 0)) {
			PX4_ERR("can't reset mixers on %s", devname);
			return 1;
		}
	}

	char buf[2048];

	if (load_mixer_file(fname, &buf[0], sizeof(buf)) < 0) {
		PX4_ERR("can't load mixer file: %s", fname);
		return 1;
	}

	/* Pass the buffer to the device */
	int ret = px4_ioctl(dev, MIXERIOCLOADBUF, (unsigned long)buf);

	if (ret < 0) {
		PX4_ERR("failed to load mixers from %s", fname);
		return 1;
	}

	return 0;
}
