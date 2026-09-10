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

#include "BoschDirect.hpp"
#include "BoschDirectRegisters.hpp"

#include <drivers/drv_sensor.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

using namespace frequency_literals;
using namespace time_literals;
using namespace bosch_direct_model;
using namespace bosch_direct_registers;
using bosch_direct_fifo::Format;

namespace
{
#if defined(CONFIG_BOSCH_DIRECT_BMI085) \
	|| defined(CONFIG_BOSCH_DIRECT_BMI088)

#if defined(CONFIG_BOSCH_DIRECT_ACCELEROMETER_INT_NONE)
constexpr bool kAccel08xInterrupt { false };
#else
constexpr bool kAccel08xInterrupt { true };
#endif // CONFIG_BOSCH_DIRECT_ACCELEROMETER_INT_NONE

#if defined(CONFIG_BOSCH_DIRECT_GYROSCOPE_INT_NONE)
constexpr bool kGyro08xInterrupt { false };
#else
constexpr bool kGyro08xInterrupt { true };
#endif // CONFIG_BOSCH_DIRECT_GYROSCOPE_INT_NONE

#endif // CONFIG_BOSCH_DIRECT_BMI085 || CONFIG_BOSCH_DIRECT_BMI088

#if defined(CONFIG_BOSCH_DIRECT_BMI055)
constexpr Profile bmi055AccelProfile()
{
	Profile profile {};

	profile.device.name                 = "bmi055";
	profile.device.device_type          = DRV_ACC_DEVTYPE_BMI055;
	profile.device.frequency            = 10_MHz;
	profile.device.data_frequency       = 10_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.component            = 'A';
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 0;
	profile.device.data_prefix_bytes    = 1;
	profile.device.continuous_data_cs   = true;

	profile.format = Format::kFixed12Accel;

	profile.device.max_transfer_bytes = BoschDirect::maxTransferSize(profile.format, profile.device.data_prefix_bytes);

	profile.variant             = Variant::kBmi055Accel;
	profile.whoami              = 0xfa;
	profile.alternate_whoami    = 0xfa;
	profile.rate_hz             = 2000;
	profile.fifo_capacity_bytes = 192;
	profile.reset_reg           = fixed::Reset;
	profile.count_reg           = fixed::FifoStatus;
	profile.fifo_reg            = fixed::FifoData;
	profile.watermark_reg       = fixed::AccelWatermark;
	profile.temperature_reg     = fixed::Temperature;
	profile.temperature         = Temperature::kSigned8;
	profile.reset_wait_us       = 25_ms;
	profile.configure_wait_us   = 25_ms;

	profile.accel_range        = 16.f * CONSTANTS_ONE_G;
	profile.accel_scale        = CONSTANTS_ONE_G / 128.f;
	profile.gyro_range         = 0.f;
	profile.gyro_scale         = 0.f;
	profile.interrupt_enabled  = true;
	profile.reject_all_minimum = false;

	return profile;
}
#endif // CONFIG_BOSCH_DIRECT_BMI055

#if defined(CONFIG_BOSCH_DIRECT_BMI055)
constexpr Profile bmi055GyroProfile()
{
	Profile profile {};

	profile.device.name                 = "bmi055";
	profile.device.device_type          = DRV_GYR_DEVTYPE_BMI055;
	profile.device.frequency            = 10_MHz;
	profile.device.data_frequency       = 10_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.component            = 'G';
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 0;
	profile.device.data_prefix_bytes    = 1;
	profile.device.continuous_data_cs   = true;

	profile.format = Format::kFixed16Gyro;

	profile.device.max_transfer_bytes = BoschDirect::maxTransferSize(profile.format, profile.device.data_prefix_bytes);

	profile.variant             = Variant::kBmi055Gyro;
	profile.whoami              = 0x0f;
	profile.alternate_whoami    = 0x0f;
	profile.rate_hz             = 2000;
	profile.fifo_capacity_bytes = 600;
	profile.reset_reg           = fixed::Reset;
	profile.count_reg           = fixed::FifoStatus;
	profile.fifo_reg            = fixed::FifoData;
	profile.watermark_reg       = fixed::GyroWatermark;
	profile.temperature_reg     = 0;
	profile.temperature         = Temperature::kNone;
	profile.reset_wait_us       = 25_ms;
	profile.configure_wait_us   = 1_ms;

	profile.accel_range        = 0.f;
	profile.accel_scale        = 0.f;
	profile.gyro_range         = math::radians(2000.f);
	profile.gyro_scale         = math::radians(1.f / 16.384f);
	profile.interrupt_enabled  = true;
	profile.reject_all_minimum = false;

	return profile;
}
#endif // CONFIG_BOSCH_DIRECT_BMI055

#if defined(CONFIG_BOSCH_DIRECT_BMI085)
constexpr Profile bmi085AccelProfile()
{
	Profile profile {};

	profile.device.name                 = "bmi085";
	profile.device.device_type          = DRV_ACC_DEVTYPE_BMI085;
	profile.device.frequency            = 10_MHz;
	profile.device.data_frequency       = 10_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.component            = 'A';
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 1;
	profile.device.data_prefix_bytes    = 4;
	profile.device.continuous_data_cs   = true;

	profile.format = Format::kTaggedAccel;

	profile.device.max_transfer_bytes = BoschDirect::maxTransferSize(profile.format, profile.device.data_prefix_bytes);

	profile.variant             = Variant::kBmi08xAccel;
	profile.whoami              = 0x1f;
	profile.alternate_whoami    = 0x1f;
	profile.rate_hz             = 1600;
	profile.fifo_capacity_bytes = 1024;
	profile.reset_reg           = tagged::Command;
	profile.count_reg           = tagged::FifoLength;
	profile.fifo_reg            = tagged::FifoLength;
	profile.watermark_reg       = tagged::WatermarkLow;
	profile.temperature_reg     = tagged::Temperature;
	profile.temperature         = Temperature::kBmi08x11;
	profile.reset_wait_us       = 1_ms;
	profile.configure_wait_us   = 10_ms;

	profile.accel_range        = 16.f * CONSTANTS_ONE_G;
	profile.accel_scale        = 16.f * CONSTANTS_ONE_G / 32768.f;
	profile.gyro_range         = 0.f;
	profile.gyro_scale         = 0.f;
	profile.interrupt_enabled  = kAccel08xInterrupt;
	profile.reject_all_minimum = false;

	return profile;
}
#endif // CONFIG_BOSCH_DIRECT_BMI085

#if defined(CONFIG_BOSCH_DIRECT_BMI085)
constexpr Profile bmi085GyroProfile()
{
	Profile profile {};

	profile.device.name                 = "bmi085";
	profile.device.device_type          = DRV_GYR_DEVTYPE_BMI085;
	profile.device.frequency            = 10_MHz;
	profile.device.data_frequency       = 10_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.component            = 'G';
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 0;
	profile.device.data_prefix_bytes    = 1;
	profile.device.continuous_data_cs   = true;

	profile.format = Format::kFixed16Gyro;

	profile.device.max_transfer_bytes = BoschDirect::maxTransferSize(profile.format, profile.device.data_prefix_bytes);

	profile.variant             = Variant::kBmi08xGyro;
	profile.whoami              = 0x0f;
	profile.alternate_whoami    = 0x0f;
	profile.rate_hz             = 2000;
	profile.fifo_capacity_bytes = 600;
	profile.reset_reg           = fixed::Reset;
	profile.count_reg           = fixed::FifoStatus;
	profile.fifo_reg            = fixed::FifoData;
	profile.watermark_reg       = fixed::GyroWatermark;
	profile.temperature_reg     = 0;
	profile.temperature         = Temperature::kNone;
	profile.reset_wait_us       = 30_ms;
	profile.configure_wait_us   = 10_ms;

	profile.accel_range        = 0.f;
	profile.accel_scale        = 0.f;
	profile.gyro_range         = math::radians(2000.f);
	profile.gyro_scale         = math::radians(1.f / 16.384f);
	profile.interrupt_enabled  = kGyro08xInterrupt;
	profile.reject_all_minimum = false; // Preserve BMI085's original full signed data range.

	return profile;
}
#endif // CONFIG_BOSCH_DIRECT_BMI085

#if defined(CONFIG_BOSCH_DIRECT_BMI088)
constexpr Profile bmi088AccelProfile()
{
	Profile profile {};

	profile.device.name                 = "bmi088";
	profile.device.device_type          = DRV_ACC_DEVTYPE_BMI088;
	profile.device.frequency            = 10_MHz;
	profile.device.data_frequency       = 10_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.component            = 'A';
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 1;
	profile.device.data_prefix_bytes    = 4;
	profile.device.continuous_data_cs   = true;

	profile.format = Format::kTaggedAccel;

	profile.device.max_transfer_bytes = BoschDirect::maxTransferSize(profile.format, profile.device.data_prefix_bytes);

	profile.variant             = Variant::kBmi08xAccel;
	profile.whoami              = 0x1e;
	profile.alternate_whoami    = 0x1a;
	profile.rate_hz             = 1600;
	profile.fifo_capacity_bytes = 1024;
	profile.reset_reg           = tagged::Command;
	profile.count_reg           = tagged::FifoLength;
	profile.fifo_reg            = tagged::FifoLength;
	profile.watermark_reg       = tagged::WatermarkLow;
	profile.temperature_reg     = tagged::Temperature;
	profile.temperature         = Temperature::kBmi08x11;
	profile.reset_wait_us       = 1_ms;
	profile.configure_wait_us   = 10_ms;

	profile.accel_range        = 24.f * CONSTANTS_ONE_G;
	profile.accel_scale        = 24.f * CONSTANTS_ONE_G / 32768.f;
	profile.gyro_range         = 0.f;
	profile.gyro_scale         = 0.f;
	profile.interrupt_enabled  = kAccel08xInterrupt;
	profile.reject_all_minimum = false;

	return profile;
}
#endif // CONFIG_BOSCH_DIRECT_BMI088

#if defined(CONFIG_BOSCH_DIRECT_BMI088)
constexpr Profile bmi088GyroProfile()
{
	Profile profile {};

	profile.device.name                 = "bmi088";
	profile.device.device_type          = DRV_GYR_DEVTYPE_BMI088;
	profile.device.frequency            = 10_MHz;
	profile.device.data_frequency       = 10_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.component            = 'G';
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 0;
	profile.device.data_prefix_bytes    = 1;
	profile.device.continuous_data_cs   = true;

	profile.format = Format::kFixed16Gyro;

	profile.device.max_transfer_bytes = BoschDirect::maxTransferSize(profile.format, profile.device.data_prefix_bytes);

	profile.variant             = Variant::kBmi08xGyro;
	profile.whoami              = 0x0f;
	profile.alternate_whoami    = 0x0f;
	profile.rate_hz             = 2000;
	profile.fifo_capacity_bytes = 600;
	profile.reset_reg           = fixed::Reset;
	profile.count_reg           = fixed::FifoStatus;
	profile.fifo_reg            = fixed::FifoData;
	profile.watermark_reg       = fixed::GyroWatermark;
	profile.temperature_reg     = 0;
	profile.temperature         = Temperature::kNone;
	profile.reset_wait_us       = 30_ms;
	profile.configure_wait_us   = 10_ms;

	profile.accel_range        = 0.f;
	profile.accel_scale        = 0.f;
	profile.gyro_range         = math::radians(2000.f);
	profile.gyro_scale         = math::radians(1.f / 16.384f);
	profile.interrupt_enabled  = kGyro08xInterrupt;
	profile.reject_all_minimum = true;

	return profile;
}
#endif // CONFIG_BOSCH_DIRECT_BMI088

#if defined(CONFIG_BOSCH_DIRECT_BMI270)
constexpr Profile bmi270Profile()
{
	Profile profile {};

	profile.device.name                 = "bmi270";
	profile.device.device_type          = DRV_IMU_DEVTYPE_BMI270;
	profile.device.frequency            = 10_MHz;
	profile.device.data_frequency       = 10_MHz;
	profile.device.mode                 = SPIDEV_MODE3;
	profile.device.component            = 0;
	profile.device.max_clock_hz         = 0;
	profile.device.register_dummy_bytes = 1;
	profile.device.data_prefix_bytes    = 2;
	profile.device.continuous_data_cs   = true;

	profile.format = Format::kTaggedImu;

	profile.device.max_transfer_bytes = BoschDirect::maxTransferSize(profile.format, profile.device.data_prefix_bytes);

	profile.variant             = Variant::kBmi270;
	profile.whoami              = 0x24;
	profile.alternate_whoami    = 0x24;
	profile.rate_hz             = 1600;
	profile.fifo_capacity_bytes = 6144;
	profile.reset_reg           = tagged::Command;
	profile.count_reg           = tagged::FifoLength;
	profile.fifo_reg            = tagged::FifoData;
	profile.watermark_reg       = tagged::WatermarkLow;
	profile.temperature_reg     = tagged::Temperature;
	profile.temperature         = Temperature::kBmi27016;
	profile.reset_wait_us       = 1_ms;
	profile.configure_wait_us   = 450_us;

	profile.accel_range        = 16.f * CONSTANTS_ONE_G;
	profile.accel_scale        = 16.f * CONSTANTS_ONE_G / 32768.f;
	profile.gyro_range         = math::radians(2000.f);
	profile.gyro_scale         = math::radians(2000.f) / 32767.f; // Retain the original PX4 conversion.
	profile.interrupt_enabled  = true;
	profile.reject_all_minimum = false;

	return profile;
}
#endif // CONFIG_BOSCH_DIRECT_BMI270

constexpr Profile kModels[] {
#if defined(CONFIG_BOSCH_DIRECT_BMI055)
	bmi055AccelProfile(),
#endif // CONFIG_BOSCH_DIRECT_BMI055

#if defined(CONFIG_BOSCH_DIRECT_BMI055)
	bmi055GyroProfile(),
#endif // CONFIG_BOSCH_DIRECT_BMI055

#if defined(CONFIG_BOSCH_DIRECT_BMI085)
	bmi085AccelProfile(),
#endif // CONFIG_BOSCH_DIRECT_BMI085

#if defined(CONFIG_BOSCH_DIRECT_BMI085)
	bmi085GyroProfile(),
#endif // CONFIG_BOSCH_DIRECT_BMI085

#if defined(CONFIG_BOSCH_DIRECT_BMI088)
	bmi088AccelProfile(),
#endif // CONFIG_BOSCH_DIRECT_BMI088

#if defined(CONFIG_BOSCH_DIRECT_BMI088)
	bmi088GyroProfile(),
#endif // CONFIG_BOSCH_DIRECT_BMI088

#if defined(CONFIG_BOSCH_DIRECT_BMI270)
	bmi270Profile(),
#endif // CONFIG_BOSCH_DIRECT_BMI270
};
static_assert(sizeof(kModels) > 0, "Select at least one BMI model");

} // namespace

void BoschDirect::print_usage()
{
	PRINT_MODULE_DESCRIPTION(R"DESCR(
### Description
SPI driver for split BMI055/BMI085/BMI088 endpoints and the integrated BMI270.
Each instance uses one existing board SPI registration and the native PX4 publishers.
Split sensors require exactly one of -A or -G on start; BMI270 accepts neither.
Accel and gyro registrations remain independent; no synthetic pairing is required.

Model availability depends on Kconfig. Start requires an exact -T model; stop/status
may omit -T to visit all compiled family instances matching the native bus selectors.
Without a bus selector, start uses board-registered internal SPI devices of the selected type.

### Examples
```
bosch_direct -T bmi088 -A start
bosch_direct -T bmi088 -G start
bosch_direct -T bmi270 start
bosch_direct -T bmi088 status
bosch_direct stop
```
)DESCR");
	PRINT_MODULE_USAGE_NAME("bosch_direct", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('T', nullptr,
		"bmi055 | bmi085 | bmi088 | bmi270",
		"Exact model (required for start; availability depends on Kconfig)", false);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, ROTATION_MAX - 1, "Rotation", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('A', "Split accel endpoint; start requires -A or -G, neither for BMI270", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('G', "Split gyro endpoint; mutually exclusive with -A", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop instances; omit -T for all compiled family models");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print instances; omit -T for all compiled family models");
}

extern "C" __EXPORT int bosch_direct_main(int argc, char *argv[])
{
	return imu::spiFamilyMain<BoschDirect>(argc, argv, MODULE_NAME, kModels);
}
