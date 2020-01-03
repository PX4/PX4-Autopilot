/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "SF1XX.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

namespace sf1xx
{

SF1XX *g_dev{nullptr};

static int start_bus(uint8_t rotation, int i2c_bus)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return -1;
	}

	/* create the driver */
	g_dev = new SF1XX(rotation, i2c_bus);

	if (g_dev == nullptr) {
		return -1;
	}

	if (g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return -1;
	}

	return 0;
}

static int start(uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return -1;
	}

	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(rotation, i2c_bus_options[i]) == PX4_OK) {
			return PX4_OK;
		}
	}

	return 0;
}

static int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		return -1;
	}

	return 0;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return -1;
	}

	g_dev->print_info();

	return 0;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for Lightware SFxx series LIDAR rangefinders: SF10/a, SF10/b, SF10/c, SF11/c, SF/LW20.

Setup/usage information: https://docs.px4.io/en/sensor/sfxx_lidar.html

### Examples

Attempt to start driver on any bus (start on bus where first sensor found).
$ sf1xx start -a
Stop driver
$ sf1xx stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sf1xx", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "Attempt to start driver on all I2C buses", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 1, 1, 2000, "Start driver on specific I2C bus", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 1, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");

	return 0;
}

} // namespace

extern "C" __EXPORT int sf1xx_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	bool start_all = false;
	int i2c_bus = PX4_I2C_BUS_EXPANSION;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "ab:R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		case 'a':
			start_all = true;
			break;

		default:
			sf1xx::usage();
			return -1;
		}
	}

	if (myoptind >= argc) {
		sf1xx::usage();
		return -1;
	}

	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return sf1xx::start(rotation);

		} else {
			return sf1xx::start_bus(rotation, i2c_bus);
		}
	} else if (!strcmp(argv[myoptind], "stop")) {
		return sf1xx::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return sf1xx::status();
	}

	sf1xx::usage();
	return -1;
}
