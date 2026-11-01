/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "TdkDirect.hpp"
#include <drivers/drv_hrt.h>

using namespace frequency_literals;

using namespace time_literals;
using namespace tdk_direct_registers;

namespace
{
using Variant = TdkDirect::Variant;


#if defined(CONFIG_TDK_DIRECT_MPU6500)
constexpr TdkDirect::Profile mpu6500Profile()
{
	TdkDirect::Profile profile {};

	profile.device.name                 = "mpu6500";
	profile.device.device_type          = DRV_IMU_DEVTYPE_MPU6500;
	profile.device.frequency            = 1_MHz;
	profile.device.data_frequency       = 10_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.max_transfer_bytes   = TdkDirect::maxTransferSize(FIFO_PACKET_SIZE_CLASSIC);
	profile.device.data_prefix_bytes    = 1;
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 0;
	profile.device.continuous_data_cs   = true;

	profile.variant              = Variant::kMpu6500;
	profile.whoami               = 0x70;
	profile.fifo_size            = 512;
	profile.fifo_packet_size     = FIFO_PACKET_SIZE_CLASSIC;
	profile.gyro_offset          = 6;
	profile.samples_per_transfer = 2;

	profile.temperature_sensitivity = 333.87f;
	profile.temperature_offset      = 21.f;

	profile.reset_pwr_value           = static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0);
	profile.reset_wait_us             = 100_ms;
	profile.configure_wait_us         = 100_ms;
	profile.check_reset_pwr           = true;
	profile.check_reset_config        = false;
	profile.has_factory_accel_offsets = true;
	profile.has_fifo_temperature      = false;

	return profile;
}
#endif // CONFIG_TDK_DIRECT_MPU6500



#if defined(CONFIG_TDK_DIRECT_ICM20608G)
constexpr TdkDirect::Profile icm20608GProfile()
{
	TdkDirect::Profile profile {};

	profile.device.name                 = "icm20608g";
	profile.device.device_type          = DRV_IMU_DEVTYPE_ICM20608G;
	profile.device.frequency            = 8_MHz;
	profile.device.data_frequency       = 8_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.max_transfer_bytes   = TdkDirect::maxTransferSize(FIFO_PACKET_SIZE_CLASSIC);
	profile.device.data_prefix_bytes    = 1;
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 0;
	profile.device.continuous_data_cs   = true;

	profile.variant              = Variant::kIcm20608G;
	profile.whoami               = 0xaf;
	profile.fifo_size            = 512;
	profile.fifo_packet_size     = FIFO_PACKET_SIZE_CLASSIC;
	profile.gyro_offset          = 6;
	profile.samples_per_transfer = 2;

	profile.temperature_sensitivity = 326.8f;
	profile.temperature_offset      = 25.f;

	profile.reset_pwr_value           = static_cast<uint8_t>(PWR_MGMT_1_BIT::SLEEP);
	profile.reset_wait_us             = 100_ms;
	profile.configure_wait_us         = 100_ms;
	profile.check_reset_pwr           = true;
	profile.check_reset_config        = false;
	profile.has_factory_accel_offsets = true;
	profile.has_fifo_temperature      = false;

	return profile;
}
#endif // CONFIG_TDK_DIRECT_ICM20608G

#if defined(CONFIG_TDK_DIRECT_ICM20689)
constexpr TdkDirect::Profile icm20689Profile()
{
	TdkDirect::Profile profile {};

	profile.device.name                 = "icm20689";
	profile.device.device_type          = DRV_IMU_DEVTYPE_ICM20689;
	profile.device.frequency            = 8_MHz;
	profile.device.data_frequency       = 8_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.max_transfer_bytes   = TdkDirect::maxTransferSize(FIFO_PACKET_SIZE_CLASSIC);
	profile.device.data_prefix_bytes    = 1;
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 0;
	profile.device.continuous_data_cs   = true;

	profile.variant              = Variant::kIcm20689;
	profile.whoami               = 0x98;
	profile.fifo_size            = 512;
	profile.fifo_packet_size     = FIFO_PACKET_SIZE_CLASSIC;
	profile.gyro_offset          = 6;
	profile.samples_per_transfer = 2;

	profile.temperature_sensitivity = 326.8f;
	profile.temperature_offset      = 25.f;

	profile.reset_pwr_value           = static_cast<uint8_t>(PWR_MGMT_1_BIT::SLEEP);
	profile.reset_wait_us             = 100_ms;
	profile.configure_wait_us         = 35_ms;
	profile.check_reset_pwr           = true;
	profile.check_reset_config        = false;
	profile.has_factory_accel_offsets = true;
	profile.has_fifo_temperature      = false;

	return profile;
}
#endif // CONFIG_TDK_DIRECT_ICM20689

#if defined(CONFIG_TDK_DIRECT_IAM20680HP)
constexpr TdkDirect::Profile iam20680HPProfile()
{
	TdkDirect::Profile profile {};

	profile.device.name                 = "iam20680hp";
	profile.device.device_type          = DRV_IMU_DEVTYPE_IAM20680HP;
	profile.device.frequency            = 8_MHz;
	profile.device.data_frequency       = 8_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.max_transfer_bytes   = TdkDirect::maxTransferSize(FIFO_PACKET_SIZE_CLASSIC);
	profile.device.data_prefix_bytes    = 1;
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 0;
	profile.device.continuous_data_cs   = true;

	profile.variant              = Variant::kIam20680HP;
	profile.whoami               = 0xf8;
	profile.fifo_size            = 512;
	profile.fifo_packet_size     = FIFO_PACKET_SIZE_CLASSIC;
	profile.gyro_offset          = 6;
	profile.samples_per_transfer = 2;

	profile.temperature_sensitivity = 326.8f;
	profile.temperature_offset      = 25.f;

	profile.reset_pwr_value           = 0;
	profile.reset_wait_us             = 100_ms;
	profile.configure_wait_us         = 35_ms;
	profile.check_reset_pwr           = false;
	profile.check_reset_config        = false;
	profile.has_factory_accel_offsets = true;
	profile.has_fifo_temperature      = false;

	return profile;
}
#endif // CONFIG_TDK_DIRECT_IAM20680HP

constexpr TdkDirect::Profile kModels[] {

#if defined(CONFIG_TDK_DIRECT_MPU6500)
	mpu6500Profile(),
#endif // CONFIG_TDK_DIRECT_MPU6500



#if defined(CONFIG_TDK_DIRECT_ICM20608G)
	icm20608GProfile(),
#endif // CONFIG_TDK_DIRECT_ICM20608G

#if defined(CONFIG_TDK_DIRECT_ICM20689)
	icm20689Profile(),
#endif // CONFIG_TDK_DIRECT_ICM20689

#if defined(CONFIG_TDK_DIRECT_IAM20680HP)
	iam20680HPProfile(),
#endif // CONFIG_TDK_DIRECT_IAM20680HP
};
static_assert(sizeof(kModels) > 0, "Select at least one direct-register model");
}

void TdkDirect::print_usage()
{
	PRINT_MODULE_DESCRIPTION(R"DESCR(
### Description
SPI driver for TDK flat-register IMUs with fixed-stride FIFOs.
The family shares acquisition and native accel/gyro FIFO publication, while
initialization, sample repetition and SPI phase limits remain model-specific.

Model availability depends on Kconfig. All commands require an exact -T model.
Native bus selectors restrict operations to matching instances of that model.
Without a bus selector, start uses board-registered internal SPI devices of the selected type.

### Examples
```
tdk_direct -T mpu6500 start
tdk_direct -T mpu6500 status
tdk_direct -T icm20689 stop
```
)DESCR");
	PRINT_MODULE_USAGE_NAME("tdk_direct", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('T', nullptr,
		"mpu6500 | icm20608g | icm20689 | iam20680hp",
		"Exact model (required for all commands; availability depends on Kconfig)", false);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, ROTATION_MAX - 1, "Rotation", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop instances of the required -T model");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print instances of the required -T model");

	for (const auto &model : kModels) {
		PX4_INFO("-T %s", model.device.name);
	}
}

extern "C" int tdk_direct_main(int argc, char *argv[])
{
	return imu::spiFamilyMain<TdkDirect>(argc, argv, MODULE_NAME, kModels);
}
