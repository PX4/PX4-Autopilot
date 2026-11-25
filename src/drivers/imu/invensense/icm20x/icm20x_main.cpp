/****************************************************************************
 *
 *   Copyright (c) 2020-2026 PX4 Development Team. All rights reserved.
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
 * @file icm20x_main.cpp
 * @brief Module entry point and profile table for icm20x (ICM20649 and ICM20948 6-axis).
 */

#include "ICM20x.hpp"

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>

namespace
{
constexpr imu::SpiModel makeSpi(const char *name, uint16_t device_type)
{
	imu::SpiModel model {};
	model.name               = name;
	model.device_type        = device_type;
	model.frequency          = 7 * imu::MHz;
	model.data_frequency     = 7 * imu::MHz;
	model.allow_mode0        = false;
	model.max_transfer_bytes = ICM20x::maxTransferSize();
	model.data_prefix_bytes  = 3;
	return model;
}

constexpr ICM20x::Profile kModels[] {
	{
		makeSpi("icm20649", DRV_IMU_DEVTYPE_ICM20649),
		0xe1,   // whoami
		30.f,   // accel_range_g
		1024.f, // accel_lsb_per_g
		4000.f, // gyro_range_dps
	},
	{
		makeSpi("icm20948", DRV_IMU_DEVTYPE_ICM20948),
		0xea,   // whoami
		16.f,   // accel_range_g
		2048.f, // accel_lsb_per_g
		2000.f, // gyro_range_dps
	},
};
} // namespace

void ICM20x::print_usage()
{
	PRINT_MODULE_DESCRIPTION(R"DESCR(
### Description
SPI driver for ICM20649 and the six-axis IMU part of ICM20948.
Bank selection, fixed FIFO acquisition and native accel/gyro publication are shared.
This driver does not expose ICM20948's magnetometer or auxiliary-bus bypass;
retain the original ICM20948 driver when those features are required.

Start requires an exact -T model; stop/status may omit -T to visit all running
family instances matching the native bus selectors.

### Examples
```
icm20x -T icm20649 -s start
icm20x -T icm20948 status
icm20x stop
```
)DESCR");
	PRINT_MODULE_USAGE_NAME("icm20x", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('T', nullptr,
					"icm20649 | icm20948",
					"Exact model (required on start, optional on stop/status)", false);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, ROTATION_MAX - 1, "Rotation", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop instances; omit -T for all family models");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print instances; omit -T for all family models");
}

extern "C" int icm20x_main(int argc, char *argv[])
{
	return imu::spiFamilyMain<ICM20x>(argc, argv, MODULE_NAME, kModels);
}
