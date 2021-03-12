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

#include "VescSerialDevice.hpp"
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

// Local functions in support of the shell command
namespace vesc
{

VescDevice *g_dev{nullptr};

int start(const char *port);
int status();
int stop();
int usage();

int
start(const char *port)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	// Instantiate the driver
	g_dev = new VescDevice(port);

	if (g_dev == nullptr) {
		PX4_ERR("driver start failed");
		return PX4_ERROR;
	}

	if (OK != g_dev->init()) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return 0;
}

int stop()
{
	if (g_dev != nullptr) {
		PX4_INFO("stopping driver");
		delete g_dev;
		g_dev = nullptr;
		PX4_INFO("driver stopped");

	} else {
		PX4_ERR("driver not running");
		return 1;
	}

	return PX4_OK;
}

int
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial driver for VESC motor contollers

Most boards are configured to enable/start the driver on a specified UART using the VESC_PORT parameter.

Setup/usage information to be added.

### Examples

Attempt to start driver on a specified serial device.
$ vesc start -d /dev/ttyS1
Stop driver
$ vesc stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vesc", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Print driver status");
	return PX4_OK;
}

} // namespace

extern "C" __EXPORT int vesc_main(int argc, char *argv[])
{
	int ch{0};
	const char *device_path{nullptr};
	int myoptind{1};
	const char *myoptarg{nullptr};

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option");
			return PX4_ERROR;
		}
	}

	if (myoptind >= argc) {
		PX4_ERR("Unrecognized command");
		return vesc::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		if (strcmp(device_path, "") != 0) {
			return vesc::start(device_path);

		} else {
			PX4_WARN("Please specify device path");
			return vesc::usage();
		}

	} else if (!strcmp(argv[myoptind], "stop")) {
		return vesc::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return vesc::status();
	}

	return vesc::usage();
}
