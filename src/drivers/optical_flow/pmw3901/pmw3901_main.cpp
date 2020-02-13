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

#include "PMW3901.hpp"
#include <px4_platform_common/spi.h>

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int pmw3901_main(int argc, char *argv[]);

/**
 * Local functions in support of the shell command.
 */
namespace pmw3901
{

PMW3901	*g_dev;

void	start(int external_spi_bus, int cs_index);
void	stop();
void	test();
void	reset();
void	info();
void	usage();

/**
 * Start the driver.
 */
void
start(int external_spi_bus, int cs_index)
{
	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	// expect the device on the n-th external bus
	SPIBusIterator bus_iterator(SPIBusIterator::FilterType::ExternalBus, cs_index, external_spi_bus);

	if (bus_iterator.next()) {
		g_dev = new PMW3901(bus_iterator.bus().bus, bus_iterator.devid(), (enum Rotation)0);

	} else {
		PX4_ERR("No external SPI bus");
	}

	if (g_dev == nullptr) {
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
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Print a little info about how to start/stop/use the driver
 */
void usage()
{
	PX4_INFO("usage: pmw3901 {start|test|reset|info'}");
	PX4_INFO("    [-b SPI_BUS] Use n-th external bus (default=1)");
	PX4_INFO("    [-c cs] chip-select signal (default=1)");
}

} // namespace pmw3901


int
pmw3901_main(int argc, char *argv[])
{
	if (argc < 2) {
		pmw3901::usage();
		return PX4_ERROR;
	}

	bool err_flag = false;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int external_spi_bus = 1;
	int chipselect_index = 1;

	while ((ch = px4_getopt(argc, argv, "b:c:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			external_spi_bus = (uint8_t)atoi(myoptarg);
			break;

		case 'c':
			chipselect_index = (uint8_t)atoi(myoptarg);
			break;

		default:
			err_flag = true;
			break;
		}
	}

	if (err_flag) {
		pmw3901::usage();
		return PX4_ERROR;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		pmw3901::start(external_spi_bus, chipselect_index);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		pmw3901::stop();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		pmw3901::info();
	}

	pmw3901::usage();
	return PX4_ERROR;
}
