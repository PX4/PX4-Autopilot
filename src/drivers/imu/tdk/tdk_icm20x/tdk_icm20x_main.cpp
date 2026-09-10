/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "TdkIcm20x.hpp"
#include <drivers/drv_hrt.h>

using namespace frequency_literals;

namespace
{
#if defined(CONFIG_TDK_ICM20X_ICM20649)
constexpr TdkIcm20x::Profile icm20649Profile()
{
	TdkIcm20x::Profile profile {};

	profile.device.name                 = "icm20649";
	profile.device.device_type          = DRV_IMU_DEVTYPE_ICM20649;
	profile.device.frequency            = 7_MHz;
	profile.device.data_frequency       = 7_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.max_transfer_bytes   = TdkIcm20x::maxTransferSize();
	profile.device.data_prefix_bytes    = 3;
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 0;
	profile.device.continuous_data_cs   = true;

	profile.whoami          = 0xe1;
	profile.accel_range_g   = 30.f;
	profile.accel_lsb_per_g = 1024.f;
	profile.gyro_range_dps  = 4000.f;

	return profile;
}
#endif // CONFIG_TDK_ICM20X_ICM20649

#if defined(CONFIG_TDK_ICM20X_ICM20948)
constexpr TdkIcm20x::Profile icm20948Profile()
{
	TdkIcm20x::Profile profile {};

	profile.device.name                 = "icm20948";
	profile.device.device_type          = DRV_IMU_DEVTYPE_ICM20948;
	profile.device.frequency            = 7_MHz;
	profile.device.data_frequency       = 7_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.max_transfer_bytes   = TdkIcm20x::maxTransferSize();
	profile.device.data_prefix_bytes    = 3;
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 0;
	profile.device.continuous_data_cs   = true;

	profile.whoami          = 0xea;
	profile.accel_range_g   = 16.f;
	profile.accel_lsb_per_g = 2048.f;
	profile.gyro_range_dps  = 2000.f;

	return profile;
}
#endif // CONFIG_TDK_ICM20X_ICM20948

constexpr TdkIcm20x::Profile kModels[] {
#if defined(CONFIG_TDK_ICM20X_ICM20649)
	icm20649Profile(),
#endif // CONFIG_TDK_ICM20X_ICM20649

#if defined(CONFIG_TDK_ICM20X_ICM20948)
	icm20948Profile(),
#endif // CONFIG_TDK_ICM20X_ICM20948
};
static_assert(sizeof(kModels) > 0, "Select at least one banked model");
}

void TdkIcm20x::print_usage()
{
	PRINT_MODULE_DESCRIPTION(R"DESCR(
### Description
SPI driver for ICM20649 and the six-axis IMU part of ICM20948.
Bank selection, fixed FIFO acquisition and native accel/gyro publication are shared.
This driver does not expose ICM20948's magnetometer or auxiliary-bus bypass;
retain the original ICM20948 driver when those features are required.

Model availability depends on Kconfig. Start requires an exact -T model; stop/status
may omit -T to visit all compiled family instances matching the native bus selectors.
Without a bus selector, start uses board-registered internal SPI devices of the selected type.

### Examples
```
tdk_icm20x -T icm20649 start
tdk_icm20x -T icm20948 status
tdk_icm20x stop
```
)DESCR");
	PRINT_MODULE_USAGE_NAME("tdk_icm20x", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('T', nullptr,
		"icm20649 | icm20948",
		"Exact model (required for start; availability depends on Kconfig)", false);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, ROTATION_MAX - 1, "Rotation", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop instances; omit -T for all compiled family models");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print instances; omit -T for all compiled family models");

	for (const auto &model : kModels) {
		PX4_INFO("-T %s", model.device.name);
	}
}

extern "C" int tdk_icm20x_main(int argc, char *argv[])
{
	return imu::spiFamilyMain<TdkIcm20x>(argc, argv, MODULE_NAME, kModels);
}
