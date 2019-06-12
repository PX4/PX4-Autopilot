/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>

#include <string.h>

#include "AnalogDifferentialPressure.hpp"

// Local functions in support of the shell command.
namespace analog_diffpres
{
AnalogDifferentialPressure *g_dev = nullptr;

int start();
int stop();
void usage();

/**
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 *
 */
int
start()
{
	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (start_bus(i2c_bus_options[i], I2C_ADDRESS_MS4525DO) == PX4_OK) {
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
start_bus(uint8_t address)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return PX4_ERROR;
	}

	// create the driver
	g_dev = new AnalogDifferentialPressure(adc_channel);

	if (g_dev == nullptr) {
		return PX4_ERROR;
	}

	// try to initialize
	if (g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

// stop the driver
int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

void usage()
{
	PX4_INFO("usage: analog_diffpres command");
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop");
}

} // namespace analog_diffpres

// Driver 'main' command.
extern "C" __EXPORT int
analog_diffpres_main(int argc, char *argv[])
{
	if (myoptind >= argc) {
		analog_diffpres::usage();
		return -1;
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return analog_diffpres::start();

		} else if (device_type == DEVICE_TYPE_MS4515) {
			return analog_diffpres::start_bus(i2c_bus, I2C_ADDRESS_MS4515DO);

		} else if (device_type == DEVICE_TYPE_MS4525) {
			return analog_diffpres::start_bus(i2c_bus, I2C_ADDRESS_MS4525DO);
		}
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return analog_diffpres::stop();
	}

	analog_diffpres::usage();
	return 0;
}
