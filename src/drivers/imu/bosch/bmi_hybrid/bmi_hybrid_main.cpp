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

/**
 * @file bmi_hybrid_main.cpp
 * @brief Module entry point and endpoint profile table for bmi_hybrid.
 */

#include "BMIHybrid.hpp"

#include <drivers/drv_sensor.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

using namespace time_literals;
using namespace bmi_hybrid_model;
using bmi_hybrid_fifo::Format;

namespace
{
#if defined(CONFIG_BMI_HYBRID_ACCELEROMETER_INT_NONE)
constexpr bool kAccel08xInterrupt { false };
#else
constexpr bool kAccel08xInterrupt { true };
#endif // CONFIG_BMI_HYBRID_ACCELEROMETER_INT_NONE

#if defined(CONFIG_BMI_HYBRID_GYROSCOPE_INT_NONE)
constexpr bool kGyro08xInterrupt { false };
#else
constexpr bool kGyro08xInterrupt { true };
#endif // CONFIG_BMI_HYBRID_GYROSCOPE_INT_NONE

constexpr imu::SpiModel makeSpi(
	const char *name,
	uint16_t device_type,
	char component,
	Format format,
	uint8_t dummy_bytes,
	uint8_t prefix_bytes)
{
	imu::SpiModel model {};
	model.name                 = name;
	model.device_type          = device_type;
	model.frequency            = 10 * imu::MHz;
	model.data_frequency       = 10 * imu::MHz;
	model.allow_mode0          = true;
	model.component            = component;
	model.max_transfer_bytes   = BMIHybrid::maxTransferSize(format, prefix_bytes);
	model.register_dummy_bytes = dummy_bytes;
	model.data_prefix_bytes    = prefix_bytes;
	return model;
}

constexpr float kGyroRange2000  { math::radians(2000.f) };
constexpr float kGyroScaleFixed { math::radians(1.f / 16.384f) };

constexpr Profile kModels[] {
	{
		makeSpi(
			"bmi055",
			DRV_ACC_DEVTYPE_BMI055,
			'A',
			Format::kFixed12Accel,
			0, // register_dummy_bytes
			1  // data_prefix_bytes
		),
		Variant::kBmi055Accel,   // variant
		Format::kFixed12Accel,   // format
		0xfa,                    // whoami
		0xfa,                    // alternate_whoami
		2000,                    // rate_hz
		192,                     // fifo_capacity_bytes
		Temperature::kSigned8,   // temperature
		25_ms,                   // reset_wait_us
		25_ms,                   // configure_wait_us
		16.f * CONSTANTS_ONE_G,  // accel_range
		CONSTANTS_ONE_G / 128.f, // accel_scale
		0.f,                     // gyro_range
		0.f,                     // gyro_scale
		true,                    // interrupt_enabled
		false,                   // reject_all_minimum
	},
	{
		makeSpi(
			"bmi055",
			DRV_GYR_DEVTYPE_BMI055,
			'G',
			Format::kFixed16Gyro,
			0, // register_dummy_bytes
			1  // data_prefix_bytes
		),
		Variant::kBmi055Gyro,  // variant
		Format::kFixed16Gyro,  // format
		0x0f,                  // whoami
		0x0f,                  // alternate_whoami
		2000,                  // rate_hz
		600,                   // fifo_capacity_bytes
		Temperature::kNone,    // temperature
		25_ms,                 // reset_wait_us
		1_ms,                  // configure_wait_us
		0.f,                   // accel_range
		0.f,                   // accel_scale
		kGyroRange2000,        // gyro_range
		kGyroScaleFixed,       // gyro_scale
		true,                  // interrupt_enabled
		false,                 // reject_all_minimum
	},
	{
		makeSpi(
			"bmi085",
			DRV_ACC_DEVTYPE_BMI085,
			'A',
			Format::kTaggedAccel,
			1, // register_dummy_bytes
			4  // data_prefix_bytes
		),
		Variant::kBmi08xAccel,            // variant
		Format::kTaggedAccel,             // format
		0x1f,                             // whoami
		0x1f,                             // alternate_whoami
		1600,                             // rate_hz
		1024,                             // fifo_capacity_bytes
		Temperature::kBmi08x11,           // temperature
		1_ms,                             // reset_wait_us
		10_ms,                            // configure_wait_us
		16.f * CONSTANTS_ONE_G,           // accel_range
		16.f *CONSTANTS_ONE_G / 32768.f,  // accel_scale
		0.f,                              // gyro_range
		0.f,                              // gyro_scale
		kAccel08xInterrupt,               // interrupt_enabled
		false,                            // reject_all_minimum
	},
	{
		makeSpi(
			"bmi085",
			DRV_GYR_DEVTYPE_BMI085,
			'G',
			Format::kFixed16Gyro,
			0, // register_dummy_bytes
			1  // data_prefix_bytes
		),
		Variant::kBmi08xGyro, // variant
		Format::kFixed16Gyro, // format
		0x0f,                 // whoami
		0x0f,                 // alternate_whoami
		2000,                 // rate_hz
		600,                  // fifo_capacity_bytes
		Temperature::kNone,   // temperature
		30_ms,                // reset_wait_us
		10_ms,                // configure_wait_us
		0.f,                  // accel_range
		0.f,                  // accel_scale
		kGyroRange2000,       // gyro_range
		kGyroScaleFixed,      // gyro_scale
		kGyro08xInterrupt,    // interrupt_enabled
		false,                // reject_all_minimum
	},
	{
		makeSpi(
			"bmi088",
			DRV_ACC_DEVTYPE_BMI088,
			'A',
			Format::kTaggedAccel,
			1, // register_dummy_bytes
			4  // data_prefix_bytes
		),
		Variant::kBmi08xAccel,            // variant
		Format::kTaggedAccel,             // format
		0x1e,                             // whoami
		0x1a,                             // alternate_whoami (BMI090L)
		1600,                             // rate_hz
		1024,                             // fifo_capacity_bytes
		Temperature::kBmi08x11,           // temperature
		1_ms,                             // reset_wait_us
		10_ms,                            // configure_wait_us
		24.f * CONSTANTS_ONE_G,           // accel_range
		24.f *CONSTANTS_ONE_G / 32768.f,  // accel_scale
		0.f,                              // gyro_range
		0.f,                              // gyro_scale
		kAccel08xInterrupt,               // interrupt_enabled
		false,                            // reject_all_minimum
	},
	{
		makeSpi(
			"bmi088",
			DRV_GYR_DEVTYPE_BMI088,
			'G',
			Format::kFixed16Gyro,
			0, // register_dummy_bytes
			1  // data_prefix_bytes
		),
		Variant::kBmi08xGyro, // variant
		Format::kFixed16Gyro, // format
		0x0f,                 // whoami
		0x0f,                 // alternate_whoami
		2000,                 // rate_hz
		600,                  // fifo_capacity_bytes
		Temperature::kNone,   // temperature
		30_ms,                // reset_wait_us
		10_ms,                // configure_wait_us
		0.f,                  // accel_range
		0.f,                  // accel_scale
		kGyroRange2000,       // gyro_range
		kGyroScaleFixed,      // gyro_scale
		kGyro08xInterrupt,    // interrupt_enabled
		true,                 // reject_all_minimum
	},
#if defined(CONFIG_BMI_HYBRID_BMI270)
	{
		makeSpi(
			"bmi270",
			DRV_IMU_DEVTYPE_BMI270,
			0,
			Format::kTaggedImu,
			1, // register_dummy_bytes
			2  // data_prefix_bytes
		),
		Variant::kBmi270,                 // variant
		Format::kTaggedImu,               // format
		0x24,                             // whoami
		0x24,                             // alternate_whoami
		1600,                             // rate_hz
		2048,                             // fifo_capacity_bytes
		Temperature::kBmi27016,           // temperature
		2_ms,                             // reset_wait_us
		450_us,                           // configure_wait_us
		16.f * CONSTANTS_ONE_G,           // accel_range
		16.f *CONSTANTS_ONE_G / 32768.f,  // accel_scale
		kGyroRange2000,                   // gyro_range
		kGyroScaleFixed,                  // gyro_scale
		true,                             // interrupt_enabled
		false,                            // reject_all_minimum
	},
#endif // CONFIG_BMI_HYBRID_BMI270
};

} // namespace

void BMIHybrid::print_usage()
{
	PRINT_MODULE_DESCRIPTION(R"DESCR(
### Description
SPI driver for split BMI055/BMI085/BMI088 endpoints and the optional integrated BMI270.
Each instance uses one existing board SPI registration and the native PX4 publishers.
Split sensors require exactly one of -A or -G on start; BMI270 accepts neither.
Accel and gyro registrations remain independent; no synthetic pairing is required.

Start requires an exact -T model; stop and status accept an optional -T filter.

### Examples
```
bmi_hybrid -T bmi088 -s -A start
bmi_hybrid -T bmi088 -s -G start
bmi_hybrid status
bmi_hybrid -T bmi088 stop
```
)DESCR");
	PRINT_MODULE_USAGE_NAME("bmi_hybrid", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('T', nullptr,
					"bmi055 | bmi085 | bmi088 | bmi270",
					"Exact model (required on start, optional filter on stop/status; bmi270 requires CONFIG_BMI_HYBRID_BMI270)", false);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, ROTATION_MAX - 1, "Rotation", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('A', "Split accel endpoint; start requires -A or -G, neither for BMI270", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('G', "Split gyro endpoint; mutually exclusive with -A", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop running instances (optionally filtered by -T, -A or -G)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print running instances (optionally filtered by -T, -A or -G)");
}

extern "C" int bmi_hybrid_main(int argc, char *argv[])
{
	return imu::spiFamilyMain<BMIHybrid>(
		       argc,
		       argv,
		       MODULE_NAME,
		       kModels,
		       imu::SpiCliCapabilities { false, true });
}
