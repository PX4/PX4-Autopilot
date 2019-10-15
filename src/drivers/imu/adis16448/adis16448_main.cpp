/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file ADIS16448.cpp
 */

#include "ADIS16448.h"

/**
 * Local functions in support of the shell command.
 */
namespace adis16448
{

ADIS16448 *g_dev;

int info();
int start(enum Rotation rotation);
int stop();
void usage();

/**
 * Start the driver.
 */
int
start(enum Rotation rotation)
{
	if (g_dev != nullptr) {
		// If already started, the still command succeeded.
		PX4_INFO("already started");
	}

	// Create the driver.
#if defined(PX4_SPI_BUS_EXT)
	g_dev = new ADIS16448(PX4_SPI_BUS_EXT, PX4_SPIDEV_EXT_MPU, rotation);
#else
	PX4_ERR("External SPI not available");
#endif

	if (g_dev != nullptr) {
		if (g_dev->init() == OK) {
			return PX4_OK;
		}

		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("driver start failed");

	return PX4_ERROR;
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_INFO("driver not running");

		return PX4_ERROR;
	}

	delete g_dev;
	g_dev = nullptr;

	return PX4_OK;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_INFO("driver not running");
	}

	g_dev->print_info();

	return PX4_OK;
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'stop'");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
}

} // namespace


/**
 * Driver 'main' command.
 */
extern "C" int adis16448_main(int argc, char *argv[])
{
	enum Rotation rotation = ROTATION_NONE;
	int ch;

	/* start options */
	while ((ch = getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;

		default:
			adis16448::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	// Start/load the driver.
	if (!strcmp(verb, "start")) {
		return adis16448::start(rotation);
	}

	// Print driver information.
	if (!strcmp(verb, "info")) {
		return adis16448::info();
	}

	// Stop
	if (!strcmp(verb, "stop")) {
		return adis16448::stop();
	}

	adis16448::usage();

	return PX4_OK;
}
