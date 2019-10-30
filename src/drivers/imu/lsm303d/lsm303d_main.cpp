/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file lsm303d_main.cpp
 * Driver for the ST LSM303D MEMS accelerometer / magnetometer connected via SPI.
 */

#include "LSM303D.hpp"

#include <px4_platform_common/getopt.h>

/**
 * Local functions in support of the shell command.
 */
namespace lsm303d
{

LSM303D	*g_dev;

void	start(bool external_bus, enum Rotation rotation, unsigned range);
void	info();
void	usage();

/**
 * Start the driver.
 *
 * This function call only returns once the driver is
 * up and running or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation, unsigned range)
{
	if (g_dev != nullptr) {
		errx(0, "already started");
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_ACCEL_MAG)
		g_dev = new LSM303D(PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_ACCEL_MAG, rotation);
#else
		errx(0, "External SPI not available");
#endif

	} else {
		g_dev = new LSM303D(PX4_SPI_BUS_SENSORS, PX4_SPIDEV_ACCEL_MAG, rotation);
	}

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating LSM303D obj");
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

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
	PX4_INFO("missing command: try 'start', 'info'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (external bus)");
	PX4_INFO("    -R rotation");
}

} // namespace

int
lsm303d_main(int argc, char *argv[])
{
	bool external_bus = false;
	enum Rotation rotation = ROTATION_NONE;
	int accel_range = 8;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "XR:a:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		case 'a':
			accel_range = atoi(myoptarg);
			break;

		default:
			lsm303d::usage();
			exit(0);
		}
	}

	if (myoptind >= argc) {
		lsm303d::usage();
		exit(0);
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		lsm303d::start(external_bus, rotation, accel_range);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		lsm303d::info();
	}

	errx(1, "unrecognized command, try 'start', info'");
}
