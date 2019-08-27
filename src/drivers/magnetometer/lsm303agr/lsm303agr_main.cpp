/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "LSM303AGR.hpp"

#include <px4_platform_common/config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>

#define LSM303AGR_DEVICE_PATH_MAG	"/dev/lsm303agr_mag"

extern "C" { __EXPORT int lsm303agr_main(int argc, char *argv[]); }

/**
 * Local functions in support of the shell command.
 */
namespace lsm303agr
{

LSM303AGR	*g_dev;

void	start(enum Rotation rotation);
void	info();
void	usage();

/**
 * Start the driver.
 *
 * This function call only returns once the driver is
 * up and running or failed to detect the sensor.
 */
void
start(enum Rotation rotation)
{
	int fd_mag = -1;

	if (g_dev != nullptr) {
		errx(0, "already started");
	}

	/* create the driver */
#if defined(PX4_SPIDEV_LSM303A_M) && defined(PX4_SPIDEV_LSM303A_X)
	g_dev = new LSM303AGR(PX4_SPI_BUS_SENSOR5, LSM303AGR_DEVICE_PATH_MAG, PX4_SPIDEV_LSM303A_M, rotation);
#else
	errx(0, "External SPI not available");
#endif

	if (g_dev == nullptr) {
		PX4_ERR("alloc failed");
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	fd_mag = px4_open(LSM303AGR_DEVICE_PATH_MAG, O_RDONLY);

	/* don't fail if open cannot be opened */
	if (0 <= fd_mag) {
		if (px4_ioctl(fd_mag, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			goto fail;
		}
	}

	px4_close(fd_mag);

	exit(0);
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running\n");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'reset'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (external bus)");
	PX4_INFO("    -R rotation");
}

} // namespace

int
lsm303agr_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	enum Rotation rotation = ROTATION_NONE;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "XR:a:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			lsm303agr::usage();
			exit(0);
		}
	}

	if (myoptind >= argc) {
		lsm303agr::usage();
		exit(0);
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		lsm303agr::start(rotation);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		lsm303agr::info();
	}

	errx(1, "unrecognized command, try 'start', 'info'");
}
