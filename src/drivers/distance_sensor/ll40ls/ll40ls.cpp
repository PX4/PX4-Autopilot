/****************************************************************************
 *
 *   Copyright (c) 2014-2019 PX4 Development Team. All rights reserved.
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
 * @file ll40ls.cpp
 * @author Allyson Kreft
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author James Goppert <james.goppert@gmail.com>
 *
 * Interface for the PulsedLight Lidar-Lite range finders.
 */

#include <cstdlib>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <systemlib/err.h>

#include <board_config.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include "LidarLiteI2C.h"
#include "LidarLitePWM.h"


/**
 * @brief Local functions in support of the shell command.
 */
namespace ll40ls
{

LidarLite *instance = nullptr;

int print_regs();
int start(const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int start_bus(const int bus = PX4_I2C_BUS_EXPANSION,
	      const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int start_pwm(const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int status();
int stop();
int usage();

/**
 * @brief Prints register information to the console.
 */
int
print_regs()
{
	if (!instance) {
		PX4_ERR("No ll40ls driver running");
		return PX4_ERROR;
	}

	instance->print_registers();
	return PX4_OK;
}

/**
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 */
int
start(const uint8_t rotation)
{
	if (instance != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	for (size_t i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(i2c_bus_options[i], rotation) == PX4_OK) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_bus(const int bus, const uint8_t rotation)
{
	if (instance != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	instance = new LidarLiteI2C(bus, rotation);

	if (instance == nullptr) {
		PX4_ERR("Failed to instantiate the driver");
		delete instance;
		return PX4_ERROR;
	}

	// Initialize the sensor.
	if (instance->init() != PX4_OK) {
		PX4_ERR("Failed to initialize LidarLite on bus = %u", bus);
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
 * Start the pwm driver.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_pwm(const uint8_t rotation)
{
	if (instance != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	instance = new LidarLitePWM(rotation);

	if (instance == nullptr) {
		PX4_ERR("Failed to instantiate the driver");
		delete instance;
		return PX4_ERROR;
	}

	// Initialize the sensor.
	if (instance->init() != PX4_OK) {
		PX4_ERR("Failed to initialize LidarLite pwm.");
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

	PX4_INFO("driver stopped");
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

I2C bus driver for LidarLite rangefinders.

The sensor/driver must be enabled using the parameter SENS_EN_LL40LS.

Setup/usage information: https://docs.px4.io/en/sensor/lidar_lite.html

### Examples

Start driver on any bus (start on bus where first sensor found).
$ ll40ls start i2c -a
Start driver on specified bus
$ ll40ls start i2c -b 1
Stop driver
$ ll40ls stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ll40ls", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("print_regs","Print the register values");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("pwm", "PWM device");
	PRINT_MODULE_USAGE_COMMAND_DESCR("i2c", "I2C device");
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "Attempt to start driver on all I2C buses (first one found)", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 1, 1, 2000, "Start driver on specific I2C bus", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 1, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Print driver status information");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	return PX4_OK;
}

} // namespace ll40ls


/**
 * @brief Driver 'main' command.
 */
extern "C" __EXPORT int ll40ls_main(int argc, char *argv[])
{
	const char *myoptarg = nullptr;

	int bus = PX4_I2C_BUS_EXPANSION;
	int ch = 0;
	int myoptind = 1;

	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	bool start_i2c_all = false;
	bool start_pwm = false;

	while ((ch = px4_getopt(argc, argv, "ab:R", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			start_i2c_all = true;
			break;

		case 'b':
			bus = atoi(myoptarg);
			break;

		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			PX4_INFO("Setting Lidar orientation to %d", (int)rotation);
			break;

		default:
			return ll40ls::usage();
		}
	}

	// Determine protocol.
	if (argc > myoptind + 1) {
		const char *protocol = argv[myoptind + 1];

		if (!strcmp(protocol, "i2c")) {
			PX4_INFO("protocol %s", protocol);

		} else if (!strcmp(protocol, "pwm")) {
			PX4_INFO("protocol %s", protocol);
			start_pwm = true;

		} else {
			PX4_INFO("unknown protocol, choose pwm or i2c");
			return ll40ls::usage();
		}
	}

	if (myoptind >= argc) {
		return ll40ls::usage();
	}

	// Print the sensor register values.
	if (!strcmp(argv[myoptind], "print_regs")) {
		return ll40ls::print_regs();
	}

	// Start the driver.
	if (!strcmp(argv[myoptind], "start")) {
		if (start_i2c_all) {
			PX4_INFO("starting all i2c busses");
			return ll40ls::start(rotation);

		} else if (start_pwm) {
			PX4_INFO("starting pwm");
			return ll40ls::start_pwm(rotation);

		} else {
			return ll40ls::start_bus(bus, rotation);
		}
	}

	// Print the driver status.
	if (!strcmp(argv[myoptind], "status")) {
		return ll40ls::status();
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return ll40ls::stop();
	}

	// Print driver usage information.
	return ll40ls::usage();
}
