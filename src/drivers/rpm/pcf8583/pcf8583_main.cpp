/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "PCF8583.hpp"

#include <px4_platform_common/getopt.h>

namespace pcf8583
{
PCF8583 *g_dev{nullptr};

static int start(int i2c_bus)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return 0;
	}

	// create the driver
	g_dev = new PCF8583(i2c_bus);

	if (g_dev == nullptr) {
		PX4_ERR("driver alloc failed");
		return -1;
	}

	if (g_dev->init() != PX4_OK) {
		PX4_ERR("driver init failed");
		delete g_dev;
		g_dev = nullptr;
		return -1;
	}

	PX4_INFO("pcf8583 for bus: %d started.", i2c_bus);

	return 0;
}

static int stop()
{
	if (g_dev == nullptr) {
		PX4_WARN("driver not running");
		return -1;
	}

	delete g_dev;
	g_dev = nullptr;

	return 0;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_INFO("driver not running");
		return 0;
	}

	g_dev->print_info();

	return 0;
}

static int usage()
{
	PX4_INFO("missing command: try 'start', 'stop', 'status'");
	PX4_INFO("options:");
	PX4_INFO("    -b i2cbus (%d)", PX4_I2C_BUS_EXPANSION);

	return 0;
}

} // namespace pcf8583

extern "C" int pcf8583_main(int argc, char *argv[])
{
	int i2c_bus = PX4_I2C_BUS_EXPANSION;
	int myoptind = 1;
	int ch = 0;
	const char *myoptarg = nullptr;

	// start options
	while ((ch = px4_getopt(argc, argv, "b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		default:
			return pcf8583::usage();
		}
	}

	if (myoptind >= argc) {
		pcf8583::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return pcf8583::start(i2c_bus);

	} else if (!strcmp(verb, "stop")) {
		return pcf8583::stop();

	} else if (!strcmp(verb, "status")) {
		return pcf8583::status();
	}

	return pcf8583::usage();
}
