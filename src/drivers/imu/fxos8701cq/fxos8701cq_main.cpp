/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
 * @file fxos8701cq.cpp
 * Driver for the NXP FXOS8701CQ 6-axis sensor with integrated linear accelerometer and
 * magnetometer connected via SPI.
 */

#include "FXOS8701CQ.hpp"

#include <px4_platform_common/getopt.h>

/**
 * Local functions in support of the shell command.
 */
namespace fxos8701cq
{

FXOS8701CQ *g_dev{nullptr};

int	start(bool external_bus, enum Rotation rotation);
int	info();
int	stop();
int	regdump();
int	usage();
int	test_error();

/**
 * Start the driver.
 *
 * This function call only returns once the driver is
 * up and running or failed to detect the sensor.
 */
int
start(bool external_bus, enum Rotation rotation)
{
	if (g_dev != nullptr) {
		PX4_INFO("already started");
		return 0;
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_ACCEL_MAG)
		g_dev = new FXOS8701CQ(PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_ACCEL_MAG, rotation);
#else
		PX4_ERR("External SPI not available");
		return 0;
#endif

	} else {
		g_dev = new FXOS8701CQ(PX4_SPI_BUS_SENSORS,  PX4_SPIDEV_ACCEL_MAG, rotation);
	}

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating FXOS8701C obj");
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	return PX4_OK;
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("driver start failed");
	return PX4_ERROR;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running\n");
		return 1;
	}

	g_dev->print_info();

	return 0;
}

int
stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running\n");
		return 1;
	}

	delete g_dev;
	g_dev = nullptr;

	return 0;
}

/**
 * dump registers from device
 */
int
regdump()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running\n");
		return 1;
	}

	printf("regdump @ %p\n", g_dev);
	g_dev->print_registers();

	return 0;
}

/**
 * trigger an error
 */
int
test_error()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running\n");
		return 1;
	}

	g_dev->test_error();

	return 0;
}

int
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'stop', 'testerror' or 'regdump'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (external bus)");
	PX4_INFO("    -R rotation");

	return 0;
}

} // namespace

extern "C" { __EXPORT int fxos8701cq_main(int argc, char *argv[]); }

int fxos8701cq_main(int argc, char *argv[])
{
	bool external_bus = false;
	int ch;
	enum Rotation rotation = ROTATION_NONE;

	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "XR:a:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			fxos8701cq::usage();
			exit(0);
		}
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return fxos8701cq::start(external_bus, rotation);

	} else if (!strcmp(verb, "stop")) {
		return fxos8701cq::stop();

	} else if (!strcmp(verb, "info")) {
		return fxos8701cq::info();

	} else if (!strcmp(verb, "regdump")) {
		return fxos8701cq::regdump();

	} else if (!strcmp(verb, "testerror")) {
		return fxos8701cq::test_error();
	}

	PX4_ERR("unrecognized command, try 'start', 'stop', 'info', 'testerror' or 'regdump'");
	return PX4_ERROR;
}
