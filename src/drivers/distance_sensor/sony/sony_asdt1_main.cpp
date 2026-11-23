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

#include <parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <cstring>

namespace sony_as_dt1
{

AS_DT1 *g_dev{nullptr};

static int start(const char *serial_device_path, bool print_config_only, float yaw_offset_degrees)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return -1;
	}

	if (serial_device_path == nullptr) {
		PX4_ERR("no device specified");
		return -1;
	}

	g_dev = new AS_DT1(serial_device_path, print_config_only, yaw_offset_degrees);

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

Sony AS-DT1 serial driver. The driver probes 921600 and 115200 baud, configures
the sensor for binary streaming, and publishes multipoint distance measurements.

### Examples

$ sony_asdt1 start -d /dev/ttyS4
$ sony_asdt1 start -d /dev/ttyS4 -s
$ sony_asdt1 status
$ sony_asdt1 stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sony_asdt1", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_FLAG('s', "Send flshow and print response instead of starting measurements", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");
	return PX4_OK;
}

} // namespace sony_as_dt1

extern "C" __EXPORT int sony_asdt1_main(int argc, char *argv[])
{
	const char *device_path = nullptr;
	bool print_config_only = false;
	float yaw_offset_degrees = 0.0f;
	int cli_option = 0;
	int cli_option_index = 1;
	const char *cli_option_arg = nullptr;

	while ((cli_option = px4_getopt(argc, argv, "d:s", &cli_option_index, &cli_option_arg)) != EOF) {
		switch (cli_option) {
		case 'd':
			device_path = cli_option_arg;
			break;

		case 's':
			print_config_only = true;
			break;

		default:
			sony_as_dt1::usage();
			return -1;
		}
	}

	if (cli_option_index >= argc) {
		sony_as_dt1::usage();
		return -1;
	}

	const char *command = argv[cli_option_index];

	if (!strcmp(command, "start")) {
		const param_t handle = param_find("SENS_ASDT1_ROT");

		if (handle != PARAM_INVALID) {
			(void)param_get(handle, &yaw_offset_degrees);
		}

		return sony_as_dt1::start(device_path, print_config_only, yaw_offset_degrees);

	} else if (!strcmp(command, "stop")) {
		return sony_as_dt1::stop();

	} else if (!strcmp(command, "status")) {
		return sony_as_dt1::status();
	}

	sony_as_dt1::usage();
	return -1;
}
