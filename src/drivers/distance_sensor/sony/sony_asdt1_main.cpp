/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "sony_asdt1.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <cstring>

namespace sony_as_dt1
{

AS_DT1 *g_dev{nullptr};

static int start(const char *port, bool one_shot, bool flshow_only)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return -1;
	}

	if (port == nullptr) {
		PX4_ERR("no device specified");
		return -1;
	}

	if (one_shot && flshow_only) {
		PX4_ERR("choose either -o or -s");
		return -1;
	}

	g_dev = new AS_DT1(port, one_shot, flshow_only);

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

Sony AS-DT1 serial driver. It opens the UART, configures 115200 8N1, and writes one
padded `format binz` command followed by one padded `fsync 200` command.

### Examples

$ sony_asdt1 start -d /dev/ttyS4
$ sony_asdt1 start -d /dev/ttyS4 -o
$ sony_asdt1 start -d /dev/ttyS4 -s
$ sony_asdt1 status
$ sony_asdt1 stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sony_asdt1", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_FLAG('o', "Read once after 1 second instead of scheduled 10 ms reads", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('s', "Send flshow and print response instead of starting measurements", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");
	return PX4_OK;
}

} // namespace sony_as_dt1

extern "C" __EXPORT int sony_asdt1_main(int argc, char *argv[])
{
	const char *device_path = nullptr;
	bool one_shot = false;
	bool flshow_only = false;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:os", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path = myoptarg;
			break;

		case 'o':
			one_shot = true;
			break;

		case 's':
			flshow_only = true;
			break;

		default:
			sony_as_dt1::usage();
			return -1;
		}
	}

	if (myoptind >= argc) {
		sony_as_dt1::usage();
		return -1;
	}

	if (!strcmp(argv[myoptind], "start")) {
		return sony_as_dt1::start(device_path, one_shot, flshow_only);

	} else if (!strcmp(argv[myoptind], "stop")) {
		return sony_as_dt1::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return sony_as_dt1::status();
	}

	sony_as_dt1::usage();
	return -1;
}
