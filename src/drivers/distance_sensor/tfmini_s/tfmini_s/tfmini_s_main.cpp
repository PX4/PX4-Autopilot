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

#include "tfmini_s.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <lib/parameters/param.h>

namespace tfmini_s
{

TFmini_s *g_dev_d{nullptr};
TFmini_s *g_dev_f{nullptr};
TFmini_s *g_dev_l{nullptr};
TFmini_s *g_dev_r{nullptr};
TFmini_s *g_dev_u{nullptr};
TFmini_s *g_dev_b{nullptr};

static int start_bus(const int i2c_bus)
{
        // Instantiate the driver Downward.
        int32_t hw_model_D = 0;
        param_get(param_find("SENS_EN_TFMINI_D"), &hw_model_D);
        if (hw_model_D){
            if (g_dev_d != nullptr) {
                    PX4_ERR("already started");
                    return PX4_OK;
            }

            g_dev_d = new TFmini_s(i2c_bus, TFMINI_S_D_ADDR, distance_sensor_s::ROTATION_DOWNWARD_FACING);
            if (g_dev_d == nullptr) {
                    delete g_dev_d;
                    return PX4_ERROR;
            }

            // Initialize the sensor.
            if (g_dev_d->init() != PX4_OK) {
                    delete g_dev_d;
                    g_dev_d = nullptr;
                    return PX4_ERROR;
            }
        }

        // Instantiate the driver Forward.
        int32_t hw_model_F = 0;
        param_get(param_find("SENS_EN_TFMINI_F"), &hw_model_F);
        if (hw_model_F){
            if (g_dev_f != nullptr) {
                    PX4_ERR("already started");
                    return PX4_OK;
            }

            g_dev_f = new TFmini_s(i2c_bus, TFMINI_S_F_ADDR, distance_sensor_s::ROTATION_FORWARD_FACING);
            if (g_dev_f == nullptr) {
                    delete g_dev_f;
                    return PX4_ERROR;
            }

            // Initialize the sensor.
            if (g_dev_f->init() != PX4_OK) {
                    delete g_dev_f;
                    g_dev_f = nullptr;
                    return PX4_ERROR;
            }
        }

        // Instantiate the driver Left-ward.
        int32_t hw_model_L = 0;
        param_get(param_find("SENS_EN_TFMINI_L"), &hw_model_L);
        if (hw_model_L){
            if (g_dev_l != nullptr) {
                    PX4_ERR("already started");
                    return PX4_OK;
            }

            g_dev_l = new TFmini_s(i2c_bus, TFMINI_S_L_ADDR, distance_sensor_s::ROTATION_LEFT_FACING);
            if (g_dev_l == nullptr) {
                    delete g_dev_l;
                    return PX4_ERROR;
            }

            // Initialize the sensor.
            if (g_dev_l->init() != PX4_OK) {
                    delete g_dev_l;
                    g_dev_l = nullptr;
                    return PX4_ERROR;
            }
        }

        // Instantiate the driver Right-ward.
        int32_t hw_model_R = 0;
        param_get(param_find("SENS_EN_TFMINI_R"), &hw_model_R);
        if (hw_model_R){
            if (g_dev_r != nullptr) {
                    PX4_ERR("already started");
                    return PX4_OK;
            }

            g_dev_r = new TFmini_s(i2c_bus, TFMINI_S_R_ADDR, distance_sensor_s::ROTATION_RIGHT_FACING);
            if (g_dev_r == nullptr) {
                    delete g_dev_r;
                    return PX4_ERROR;
            }

            // Initialize the sensor.
            if (g_dev_r->init() != PX4_OK) {
                    delete g_dev_r;
                    g_dev_r = nullptr;
                    return PX4_ERROR;
            }
        }

        // Instantiate the driver Upward.
        int32_t hw_model_U = 0;
        param_get(param_find("SENS_EN_TFMINI_U"), &hw_model_U);
        if (hw_model_U){
            if (g_dev_u != nullptr) {
                    PX4_ERR("already started");
                    return PX4_OK;
            }

            g_dev_u = new TFmini_s(i2c_bus, TFMINI_S_U_ADDR, distance_sensor_s::ROTATION_UPWARD_FACING);
            if (g_dev_u == nullptr) {
                    delete g_dev_u;
                    return PX4_ERROR;
            }

            // Initialize the sensor.
            if (g_dev_u->init() != PX4_OK) {
                    delete g_dev_u;
                    g_dev_u = nullptr;
                    return PX4_ERROR;
            }
        }

        // Instantiate the driver Backward.
        int32_t hw_model_B = 0;
        param_get(param_find("SENS_EN_TFMINI_B"), &hw_model_B);
        if (hw_model_B){
            if (g_dev_b != nullptr) {
                    PX4_ERR("already started");
                    return PX4_OK;
            }

            g_dev_b = new TFmini_s(i2c_bus, TFMINI_S_B_ADDR, distance_sensor_s::ROTATION_BACKWARD_FACING);
            if (g_dev_b == nullptr) {
                    delete g_dev_b;
                    return PX4_ERROR;
            }

            // Initialize the sensor.
            if (g_dev_b->init() != PX4_OK) {
                    delete g_dev_b;
                    g_dev_b = nullptr;
                    return PX4_ERROR;
            }
        }
	return PX4_OK;
}

static int start()
{
        if (g_dev_d != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}
        if (g_dev_f != nullptr) {
                PX4_ERR("already started");
                return PX4_ERROR;
        }
        if (g_dev_l != nullptr) {
                PX4_ERR("already started");
                return PX4_ERROR;
        }
        if (g_dev_r != nullptr) {
                PX4_ERR("already started");
                return PX4_ERROR;
        }
        if (g_dev_u != nullptr) {
                PX4_ERR("already started");
                return PX4_ERROR;
        }
        if (g_dev_d != nullptr) {
                PX4_ERR("already started");
                return PX4_ERROR;
        }

	for (size_t i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
                if (start_bus(i2c_bus_options[i]) == PX4_OK) {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

static int stop()
{
        if (g_dev_d != nullptr) {
                delete g_dev_d;
                g_dev_d = nullptr;
	}
        if (g_dev_f != nullptr) {
                delete g_dev_f;
                g_dev_f = nullptr;
        }
        if (g_dev_l != nullptr) {
                delete g_dev_l;
                g_dev_l = nullptr;
        }
        if (g_dev_r != nullptr) {
                delete g_dev_r;
                g_dev_r = nullptr;
        }
        if (g_dev_u != nullptr) {
                delete g_dev_u;
                g_dev_u = nullptr;
        }
        if (g_dev_b != nullptr) {
                delete g_dev_b;
                g_dev_b = nullptr;
        }

	PX4_INFO("driver stopped");
	return PX4_OK;
}

static int status()
{
        if (g_dev_d == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}
        g_dev_d->print_info();

        if (g_dev_f == nullptr) {
                PX4_ERR("driver not running");
                return PX4_ERROR;
        }
        g_dev_f->print_info();

        if (g_dev_l == nullptr) {
                PX4_ERR("driver not running");
                return PX4_ERROR;
        }
        g_dev_l->print_info();

        if (g_dev_r == nullptr) {
                PX4_ERR("driver not running");
                return PX4_ERROR;
        }
        g_dev_r->print_info();

        if (g_dev_u == nullptr) {
                PX4_ERR("driver not running");
                return PX4_ERROR;
        }
        g_dev_u->print_info();

        if (g_dev_b == nullptr) {
                PX4_ERR("driver not running");
                return PX4_ERROR;
        }
        g_dev_b->print_info();

	return PX4_OK;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for TFmini_s rangefinders.

The sensor/driver must be enabled using the parameter SENS_EN_TFMINI_D, SENS_EN_TFMINI_F, SENS_EN_TFMINI_L, SENS_EN_TFMINI_R, SENS_EN_TFMINI_U, SENS_EN_TFMINI_B.

Setup/usage information: https://docs.px4.io/en/sensor/rangefinders.html#teraranger-rangefinders

### Examples
Start driver on any bus (start on bus where first sensor found).
$ tfmini_s start -a
Start driver on specified bus
$ tfmini_s start -b 1
Stop driver
$ tfmini_s stop
)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("tfmini_s", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
	PRINT_MODULE_USAGE_PARAM_FLAG('a', "Attempt to start driver on all I2C buses (first one found)", true);
        PRINT_MODULE_USAGE_PARAM_INT('b', 1, 1, 2000, "Start driver on specific I2C bus", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status","Print driver information");

	return PX4_OK;
}

} // namespace

extern "C" __EXPORT int tfmini_s_main(int argc, char *argv[])
{
	const char *myoptarg = nullptr;

	int ch;
	int i2c_bus = PX4_I2C_BUS_EXPANSION;
	int myoptind = 1;

	bool start_all = false;

	while ((ch = px4_getopt(argc, argv, "ab:R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			start_all = true;
			break;

		case 'b':
			i2c_bus = atoi(myoptarg);
			break;

		default:
			PX4_WARN("Unknown option!");
                        return tfmini_s::usage();
		}
	}

	if (myoptind >= argc) {
                return tfmini_s::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {

		if (start_all) {
                        return tfmini_s::start();

		} else {
                        return tfmini_s::start_bus(i2c_bus);
		}
	} else if (!strcmp(argv[myoptind], "status")) {
                return tfmini_s::status();
	} else if (!strcmp(argv[myoptind], "stop")) {
                return tfmini_s::stop();
	}

        return tfmini_s::usage();
}
