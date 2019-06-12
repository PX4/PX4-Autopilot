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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>

#include <string.h>

#include "MS4525.hpp"

// I2C bus address is 1010001x
static constexpr uint8_t I2C_ADDRESS_MS4525DO = 0x28;

// Local functions in support of the shell command.
namespace ms4525
{
MS4525 *g_dev{nullptr};

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
static int start_bus(uint8_t i2c_bus, uint8_t address)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return -1;
	}

	// create the driver
	g_dev = new MS4525(i2c_bus, address);

	if (g_dev == nullptr) {
		return -1;
	}

	// try to initialize
	if (g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return -1;
	}

	return 0;
}

/**
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 *
 */
static int start()
{
	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(i2c_bus_options[i], I2C_ADDRESS_MS4525DO) == PX4_OK) {
			return 0;
		}
	}

	return -1;
}

static int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		return -1;
	}

	return 0;
}

static int usage()
{
	PX4_INFO("usage: ms4525 command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus");
	PX4_INFO("\t-a --all");
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop");

	return 0;
}

} // namespace ms4525

// Driver 'main' command.
extern "C" __EXPORT int ms4525_main(int argc, char *argv[])
{
	uint8_t i2c_bus = PX4_I2C_BUS_EXPANSION;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	bool start_all = false;

	while ((ch = px4_getopt(argc, argv, "ab:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		case 'a':
			start_all = true;
			break;

		default:
			return ms4525::usage();
		}
	}

	if (myoptind >= argc) {
		return ms4525::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return ms4525::start();

		} else {
			return ms4525::start_bus(i2c_bus, I2C_ADDRESS_MS4525DO);
		}

	} else if (!strcmp(argv[myoptind], "stop")) {
		return ms4525::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		//return ms4525::status();
	}

	return ms4525::usage();
}
