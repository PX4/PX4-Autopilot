/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
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
 * @file brushless_rpm_main.cpp
 * @author Jerry Mailloux <jerry@envgo.com>
 *
 * Interface for the brushless RPM sensor that outputs a PWM signal corresponding
 * to spin rate in Hz.
 */

#include <cstdlib>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <systemlib/err.h>

#include <board_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include "brushless_rpm.hpp"

/**
 * @brief Local functions in support of the shell command.
 */
namespace brushless_rpm
{

BrushlessRPMPWM *instance = nullptr;

/**
 * Start the pwm driver.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_rpm_sensor(void)
{
	if (instance != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	instance = new BrushlessRPMPWM();

	if (instance == nullptr) {
		PX4_ERR("Failed to instantiate the driver");
		return PX4_ERROR;
	}

	// Initialize the sensor.
	if (instance->init() != PX4_OK) {
		PX4_ERR("Failed to initialize brushless RPM pwm.");
		delete instance;
		instance = nullptr;
		return PX4_ERROR;
	}

	// Start the driver.
	instance->start();

	PX4_INFO("driver started");
	return PX4_OK;
}

/**
 * @brief Prints status info about the driver.
 */
int
status()
{
	if (instance == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	instance->print_info();
	return PX4_OK;
}

/**
 * @brief Stops the driver
 */
int
stop()
{
	if (instance != nullptr) {
		delete instance;
		instance = nullptr;
	}

	return PX4_OK;
}

/**
 * @brief Displays driver usage at the console.
 */
int
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

PWM driver for brushless RPM sensor.

Setup/usage information: https://www.offshoreelectrics.com/proddetail.php?prod=ose-83279-011
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("brushless_rpm", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Print driver status information");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	return PX4_OK;
}

} // namespace brushless_rpm


/**
 * @brief Driver 'main' command.
 */
extern "C" __EXPORT int brushless_rpm_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (myoptind >= argc) {
		return brushless_rpm::usage();
	}

	// Start the driver.
	if (!strcmp(argv[myoptind], "start")) {
		return brushless_rpm::start_rpm_sensor();
	}

	// Print the driver status.
	if (!strcmp(argv[myoptind], "status")) {
		return brushless_rpm::status();
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return brushless_rpm::stop();
	}

	// Print driver usage information.
	return brushless_rpm::usage();
}
