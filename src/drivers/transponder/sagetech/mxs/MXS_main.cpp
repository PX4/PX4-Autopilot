/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

/*
 * MXS_main.cpp
 *
 * Sagetech MXS transponder driver
 * @author Megan McCormick megan.mccormick@sagetech.com
 */

#include "MXS.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/module.h>

namespace mxs
{

MXS	*g_dev{nullptr};

static int start(const char *port, unsigned baudrate)
{
	if (g_dev != nullptr) {
		PX4_ERR("MXS driver already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new MXS(port, baudrate);

	if (g_dev == nullptr) {
		PX4_ERR("MXS driver start failed");
		return PX4_OK;
	}

	if (PX4_OK != g_dev->init()) {
		PX4_ERR("MXS driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	PX4_INFO("MXS Driver Running");

	g_dev->start();

	return PX4_OK;
}


static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->print_info();

	return PX4_OK;
}

static int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	}

	PX4_INFO("driver stopped");
	return PX4_OK;
}

static int flightID(const char *flightId)
{
	PX4_INFO("Sending flight ID: %s", flightId);
	g_dev->handle_flight_id(flightId);

	return PX4_OK;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the Sagetech MXS Transponder.

Most boards are configured to enable/start the driver on a specified UART using the TRANS_MXS_CFG parameter.

Setup/usage information: TBD

### Examples

Attempt to start driver on a specified serial device.
$ mxs start -d /dev/ttyS1 -b 230400
Stop driver
$ mxs stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mxs", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("transponder");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", "<file:dev>", "Sagetech MXS device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 57600, 600, 921600, "Baudrate", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Current status of the Driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	return PX4_OK;
}

}//namespace
extern "C" __EXPORT int mxs_main(int argc, char *argv[]);
int mxs_main(int argc, char *argv[])
{
	PX4_INFO("Running MXS");
	const char *myoptarg = nullptr;

	int ch = 0;
	int myoptind = 1;
	char ID[9] = "";


	const char *port = nullptr;
	int baud = 0;

	while ((ch = px4_getopt(argc, argv, "d:b", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			port = myoptarg;
			break;

		case 'b':
			if (px4_get_parameter_value(myoptarg, baud) != 0) {
				PX4_ERR("baudrate parsing failed");
			}
			break;

		default:
			PX4_WARN("Unknown option");
			return mxs::usage();
		}
	}

	if (myoptind >= argc) {
		return mxs::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		return mxs::start(port, baud);

	}
	else if (!strcmp(argv[myoptind], "status")) {
		return mxs::status();

	}
	else if (!strcmp(argv[myoptind], "flightId") && (argc == 3)) {
		strcpy(ID,argv[2]);
		return mxs::flightID(ID);

	}
	else if (!strcmp(argv[myoptind], "stop")) {
		return mxs::stop();
	}
	return mxs::usage();
}


