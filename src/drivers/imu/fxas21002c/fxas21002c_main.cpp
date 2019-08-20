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

#include "FXAS21002C.hpp"

extern "C" { __EXPORT int fxas21002c_main(int argc, char *argv[]); }

/**
 * Local functions in support of the shell command.
 */
namespace fxas21002c
{

FXAS21002C	*g_dev;

void	start(bool external_bus, enum Rotation rotation);
void	info();
void	regdump();
void	usage();
void	test_error();

/**
 * Start the driver.
 *
 * This function call only returns once the driver is
 * up and running or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
	if (g_dev != nullptr) {
		PX4_INFO("already started");
		exit(0);
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_GYRO)
		g_dev = new FXAS21002C(PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_GYRO, rotation);
#else
		PX4_ERR("External SPI not available");
		exit(0);
#endif

	} else {
		g_dev = new FXAS21002C(PX4_SPI_BUS_SENSORS, PX4_SPIDEV_GYRO, rotation);
	}

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating FXAS21002C obj");
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
		PX4_ERR("driver not running\n");
		exit(1);
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * dump registers from device
 */
void
regdump()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running\n");
		exit(1);
	}

	printf("regdump @ %p\n", g_dev);
	g_dev->print_registers();

	exit(0);
}

/**
 * trigger an error
 */
void
test_error()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running\n");
		exit(1);
	}

	g_dev->test_error();

	exit(0);
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'testerror' or 'regdump'");
	PX4_INFO("options:");
	PX4_INFO("    -X    (external bus)");
	PX4_INFO("    -R rotation");
}

} // namespace

int
fxas21002c_main(int argc, char *argv[])
{
	bool external_bus = false;
	enum Rotation rotation = ROTATION_NONE;

	int ch = 0;
	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "XR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			fxas21002c::usage();
			return 0;
		}
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		fxas21002c::start(external_bus, rotation);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		fxas21002c::info();
	}

	/*
	 * dump device registers
	 */
	if (!strcmp(verb, "regdump")) {
		fxas21002c::regdump();
	}

	/*
	 * trigger an error
	 */
	if (!strcmp(verb, "testerror")) {
		fxas21002c::test_error();
	}

	PX4_WARN("unrecognized command, try 'start', 'info', 'testerror' or 'regdump'");
	return -1;
}
