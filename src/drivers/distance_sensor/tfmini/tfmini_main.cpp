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

#include "TFMINI.hpp"

#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>

/**
 * Local functions in support of the shell command.
 */
namespace tfmini
{

// One driver per serial port, so a vehicle can carry e.g. a forward- and a
// backward-facing sensor. Each instance is addressed by its port.
static constexpr int MAX_INSTANCES = 2;

TFMINI	*g_dev[MAX_INSTANCES] {};

int start(const char *port, uint8_t rotation);
int status();
int stop(const char *port);
int usage();

static int find_instance(const char *port)
{
	for (int i = 0; i < MAX_INSTANCES; i++) {
		if (g_dev[i] != nullptr && strcmp(g_dev[i]->port(), port) == 0) {
			return i;
		}
	}

	return -1;
}

int
start(const char *port, uint8_t rotation)
{
	if (find_instance(port) >= 0) {
		PX4_ERR("already started on %s", port);
		return PX4_OK;
	}

	int slot = -1;

	for (int i = 0; i < MAX_INSTANCES; i++) {
		if (g_dev[i] == nullptr) {
			slot = i;
			break;
		}
	}

	if (slot < 0) {
		PX4_ERR("already running %d instances", MAX_INSTANCES);
		return PX4_ERROR;
	}

	// Instantiate the driver.
	TFMINI *dev = new TFMINI(port, rotation);

	if (dev == nullptr) {
		PX4_ERR("driver start failed");
		return PX4_ERROR;
	}

	if (OK != dev->init()) {
		PX4_ERR("driver start failed");
		delete dev;
		return PX4_ERROR;
	}

	g_dev[slot] = dev;

	return PX4_OK;
}

int
status()
{
	int running = 0;

	for (int i = 0; i < MAX_INSTANCES; i++) {
		if (g_dev[i] != nullptr) {
			printf("instance %d @ %p\n", i, g_dev[i]);
			g_dev[i]->print_info();
			running++;
		}
	}

	if (running == 0) {
		PX4_ERR("driver not running");
		return 1;
	}

	return 0;
}

int stop(const char *port)
{
	int stopped = 0;

	for (int i = 0; i < MAX_INSTANCES; i++) {
		if (g_dev[i] == nullptr) {
			continue;
		}

		if (port != nullptr && strcmp(g_dev[i]->port(), port) != 0) {
			continue;
		}

		PX4_INFO("stopping driver on %s", g_dev[i]->port());
		delete g_dev[i];
		g_dev[i] = nullptr;
		stopped++;
	}

	if (stopped == 0) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_INFO("driver stopped");
	return PX4_OK;
}

int
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the Benewake TFmini LiDAR.

Most boards are configured to enable/start the driver on a specified UART using the SENS_TFMINI_CFG parameter,
with the sensor orientation taken from SENS_TFMINI_ROT. A second sensor on another UART is configured with
SENS_TFMINI2_CFG and SENS_TFMINI2_ROT. Up to two instances run at once, one per serial port.

Setup/usage information: https://docs.px4.io/main/en/sensor/tfmini.html

### Examples

Attempt to start driver on a specified serial device.
$ tfmini start -d /dev/ttyS1
Start a second, backward-facing sensor on another port
$ tfmini start -d /dev/ttyS4 -R 4
Stop the driver on one port
$ tfmini stop -d /dev/ttyS4
Stop all instances
$ tfmini stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tfmini", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation (MAV_SENSOR_ORIENTATION, or p:<param>) - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver (all instances, or the one on -d)");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Print driver status");
	return PX4_OK;
}

} // namespace

extern "C" __EXPORT int tfmini_main(int argc, char *argv[])
{
	int ch = 0;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = TFMINI_DEFAULT_PORT;
	bool device_given = false;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R': {
				int rot = -1;

				if (px4_get_parameter_value(myoptarg, rot) != 0) {
					PX4_ERR("rotation parsing failed");
					return PX4_ERROR;
				}

				rotation = (uint8_t)rot;
				break;
			}

		case 'd':
			device_path = myoptarg;
			device_given = true;
			break;

		default:
			PX4_WARN("Unknown option!");
			return PX4_ERROR;
		}
	}

	if (myoptind >= argc) {
		PX4_ERR("unrecognized command");
		return tfmini::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		if (strcmp(device_path, "") != 0) {
			return tfmini::start(device_path, rotation);

		} else {
			PX4_WARN("Please specify device path!");
			return tfmini::usage();
		}

	} else if (!strcmp(argv[myoptind], "stop")) {
		return tfmini::stop(device_given ? device_path : nullptr);

	} else if (!strcmp(argv[myoptind], "status")) {
		return tfmini::status();
	}

	return tfmini::usage();
}
