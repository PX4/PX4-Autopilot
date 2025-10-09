/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "AnelloX3.hpp"

#include <px4_platform_common/getopt.h>

/**
 * Local functions in support of the shell command.
 */
namespace anello_x3
{

AnelloX3 *g_dev{nullptr};

int start(const char *port, uint8_t rotation);
int status();
int stop();
int usage();
int enable_config_mode();
ssize_t send_message(const char *msg, size_t len);
void calculate_checksum(char *buf, size_t len, char *checksum_str);


int start(const char *port, uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new AnelloX3(port, rotation);

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


int status()
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


int start_config_mode()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->start_config_mode();

	// Stop data streaming from X3.
	send_message("APCFG,W,odr,0", strlen("APCFG,W,odr,0"));

	return PX4_OK;
}

int stop_config_mode()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->stop_config_mode();

	return PX4_OK;
}


int send_message(const char *msg, size_t len)
{
	if (g_dev == nullptr)
	{
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	if (len == 0 || msg == nullptr)
	{
		PX4_ERR("Invalid message or length");
		return PX4_ERROR;
	}

	if(g_dev->get_config_mode())
	{
		g_dev->send_x3_request(msg, len);

		px4_usleep(100000);

		char x3_response[100] = {"\0"};
		g_dev->get_x3_response(x3_response);
		PX4_INFO("Response: %s", x3_response);

		return PX4_OK;
	}
	else
	{
		PX4_ERR("Driver is not in config mode.");
		return PX4_ERROR;
	}

}


int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the Anello X3 IMU.

### Examples

Attempt to start driver on a specified serial device.
$ anello_x3 start -d /dev/ttyS1
Stop driver
$ anello_x3 stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("anello_x3", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("send_message", "Send a message/command to the X3 device");
	PRINT_MODULE_USAGE_PARAM_STRING('c', nullptr, nullptr, "Message/Command to send", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("start_config_mode", "Enter X3 configuration mode");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop_config_mode", "Exit X3 configuration mode");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test driver (basic functional tests)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");
	return PX4_OK;
}




} // namespace

extern "C" __EXPORT int anello_x3_main(int argc, char *argv[])
{
	int ch = 0;
	uint8_t rotation = ROTATION_NONE;
	const char *device_path = nullptr;
	int myoptind = 1;
	const char *msg = nullptr;
	const char *myoptarg = "/dev/ttyUSB0";

	while ((ch = px4_getopt(argc, argv, "R:d:c:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'd':
			device_path = myoptarg;
			break;

		case 'c':
			msg = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return PX4_ERROR;
		}
	}

	if (myoptind >= argc) {
		PX4_ERR("unrecognized command");
		return anello_x3::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		if (strcmp(device_path, "") != 0) {
			return anello_x3::start(device_path, rotation);

		} else {
			PX4_WARN("Please specify device path!");
			return anello_x3::usage();
		}

	} else if (!strcmp(argv[myoptind], "stop")) {
		return anello_x3::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return anello_x3::status();
	} else if (!strcmp(argv[myoptind], "start_config_mode")) {
		return anello_x3::start_config_mode();
	} else if (!strcmp(argv[myoptind], "stop_config_mode")) {
		return anello_x3::stop_config_mode();
	} else if (!strcmp(argv[myoptind], "send_message")) {
		return anello_x3::send_message(msg, strlen(msg));
	}

	return anello_x3::usage();
}
