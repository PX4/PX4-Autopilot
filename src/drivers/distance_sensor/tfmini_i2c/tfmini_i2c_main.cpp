/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "TFMINI_I2C.hpp"
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

namespace tfmini_i2c
{

TFMINI_I2C *g_dev{nullptr};

static int start_bus(const int i2c_bus, const uint8_t orientation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

        int32_t hw_model = 0;
        int Address;
        param_get(param_find("SENS_EN_TFMINI2C"), &hw_model);

        switch (hw_model) {
        case 0: // Disabled
                PX4_WARN("Disabled. Please set the parameter SENS_EN_TFmini_I2C.");
                return PX4_ERROR;

        case 1: // Autodetect - assume default TFmini-S
                switch (orientation) {
                case distance_sensor_s::ROTATION_DOWNWARD_FACING:
                    Address = TFMINI_S_I2C_BASEADDR;
                    break;
                case distance_sensor_s::ROTATION_FORWARD_FACING:
                    Address = TFMINI_S_I2C_BASEADDR+1;
                    break;
                case distance_sensor_s::ROTATION_LEFT_FACING:
                    Address = TFMINI_S_I2C_BASEADDR+2;
                    break;
                case distance_sensor_s::ROTATION_RIGHT_FACING:
                    Address = TFMINI_S_I2C_BASEADDR+3;
                    break;
                case distance_sensor_s::ROTATION_UPWARD_FACING:
                    Address = TFMINI_S_I2C_BASEADDR+4;
                    break;
                case distance_sensor_s::ROTATION_BACKWARD_FACING:
                    Address = TFMINI_S_I2C_BASEADDR+5;
                    break;
                default:
                    PX4_ERR("invalid orientation %u", orientation);
                    return PX4_ERROR;
                }

                break;

        case 2: // TFmini.
                Address = TFMINI_I2C_BASEADDR;
                break;

        case 3: // TFmini_Plus.
                Address = TFMINI_PLUS_I2C_BASEADDR;
                break;

        default:
                PX4_ERR("invalid HW model %d.", hw_model);
                return PX4_ERROR;
        }

	// Instantiate the driver.
        g_dev = new TFMINI_I2C(i2c_bus, Address, orientation);

	if (g_dev == nullptr) {
		delete g_dev;
		return PX4_ERROR;
	}

	// Initialize the sensor.
        if (g_dev->init() != PX4_OK) {
		delete g_dev;
                g_dev = nullptr;
		return PX4_ERROR;
        }

	return PX4_OK;
}

static int start(const uint8_t orientation)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	for (size_t i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
                if (start_bus(i2c_bus_options[i], orientation) == PX4_OK) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
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

static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->print_info();

	return PX4_OK;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for TFmini rangefinders.

The sensor/driver must be enabled using the parameter SENS_EN_TFMINI.

### Examples
Start driver on any bus (start on bus where first sensor found).
$ teraranger start -a
Start driver on specified bus
$ teraranger start -b 1
Stop driver
$ teraranger stop
)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("tfmini_i2c", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "Attempt to start driver on all I2C buses (first one found)", true);
        PRINT_MODULE_USAGE_PARAM_INT('b', 1, 1, 2000, "Start driver on specific I2C bus (1-2000)", true);
        PRINT_MODULE_USAGE_PARAM_INT('R', 25, 1, 25, "Sensor rotation (1-25) - downward facing by default (25)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Print driver information");

	return PX4_OK;
}

} // namespace

extern "C" __EXPORT int tfmini_i2c_main(int argc, char *argv[])
{
	const char *myoptarg = nullptr;

	int ch;
	int i2c_bus = PX4_I2C_BUS_EXPANSION;
	int myoptind = 1;

        uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	bool start_all = false;

        while ((ch = px4_getopt(argc, argv, "ab:R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			start_all = true;
			break;

		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

                case 'R':
                        rotation = (uint8_t)atoi(myoptarg);
                        break;

		default:
			PX4_WARN("Unknown option!");
                        return tfmini_i2c::usage();
		}
	}

	if (myoptind >= argc) {
                return tfmini_i2c::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {

		if (start_all) {
                        return tfmini_i2c::start(rotation);

		} else {
                        return tfmini_i2c::start_bus(i2c_bus, rotation);
		}
	} else if (!strcmp(argv[myoptind], "status")) {
                return tfmini_i2c::status();
	} else if (!strcmp(argv[myoptind], "stop")) {
                return tfmini_i2c::stop();
	}

        return tfmini_i2c::usage();
}
