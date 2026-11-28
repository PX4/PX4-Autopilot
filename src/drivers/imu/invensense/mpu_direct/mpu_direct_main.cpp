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
 * @file mpu_direct_main.cpp
 * @brief Module entry point and model profile table for mpu_direct.
 */

#include "MPUDirect.hpp"

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>

using namespace time_literals;
using namespace mpu_direct_registers;

namespace
{
using Variant = MPUDirect::Variant;
using Profile = MPUDirect::Profile;

constexpr imu::SpiModel makeSpi(
	const char *name,
	uint16_t device_type,
	int config_hz,
	int data_hz,
	bool allow_mode0 = true,
	uint8_t packet_size = FIFO_PACKET_SIZE_CLASSIC,
	uint8_t prefix_bytes = 1)
{
	imu::SpiModel model {};
	model.name               = name;
	model.device_type        = device_type;
	model.frequency          = config_hz;
	model.data_frequency     = data_hz;
	model.allow_mode0        = allow_mode0;
	model.max_transfer_bytes = MPUDirect::maxTransferSize(packet_size, prefix_bytes);
	model.data_prefix_bytes  = prefix_bytes;
	return model;
}

constexpr uint8_t kSleep   { static_cast<uint8_t>(PWR_MGMT_1_BIT::SLEEP) };
constexpr uint8_t kClkSel0 { static_cast<uint8_t>(PWR_MGMT_1_BIT::CLKSEL_0) };

constexpr Profile kModels[] {
	{
		makeSpi(
			"mpu6000",
			DRV_IMU_DEVTYPE_MPU6000,
			1 * imu::MHz,
			5 * imu::MHz,
			false
		),
		Variant::kMpu6000,        // variant
		0x68,                     // whoami
		1024,                     // fifo_size
		FIFO_PACKET_SIZE_CLASSIC, // fifo_packet_size
		6,                        // gyro_offset
		8,                        // samples_per_transfer
		340.f,                    // temperature_sensitivity
		36.53f,                   // temperature_offset
		kSleep,                   // reset_pwr_value
		100_ms,                   // reset_wait_us
		100_ms,                   // configure_wait_us
		true,                     // check_reset_pwr
		false,                    // check_reset_config
		false,                    // has_factory_accel_offsets
		false,                    // has_fifo_temperature
	},
	{
		makeSpi(
			"mpu6500",
			DRV_IMU_DEVTYPE_MPU6500,
			1 * imu::MHz,
			10 * imu::MHz,
			false
		),
		Variant::kMpu6500,        // variant
		0x70,                     // whoami
		512,                      // fifo_size
		FIFO_PACKET_SIZE_CLASSIC, // fifo_packet_size
		6,                        // gyro_offset
		2,                        // samples_per_transfer
		333.87f,                  // temperature_sensitivity
		21.f,                     // temperature_offset
		kClkSel0,                 // reset_pwr_value
		100_ms,                   // reset_wait_us
		100_ms,                   // configure_wait_us
		true,                     // check_reset_pwr
		false,                    // check_reset_config
		true,                     // has_factory_accel_offsets
		false,                    // has_fifo_temperature
	},
	{
		makeSpi(
			"icm20602",
			DRV_IMU_DEVTYPE_ICM20602,
			10 * imu::MHz,
			10 * imu::MHz,
			true,
			FIFO_PACKET_SIZE_ICM20602,
			3 // command + latched FIFO count prefix
		),
		Variant::kIcm20602,        // variant
		0x12,                      // whoami
		1008,                      // fifo_size
		FIFO_PACKET_SIZE_ICM20602, // fifo_packet_size
		8,                         // gyro_offset
		2,                         // samples_per_transfer
		326.8f,                    // temperature_sensitivity
		25.f,                      // temperature_offset
		kSleep | kClkSel0,         // reset_pwr_value
		2_ms,                      // reset_wait_us
		100_ms,                    // configure_wait_us (DS-000176 Table 1: 100 ms max gyro startup)
		true,                      // check_reset_pwr
		true,                      // check_reset_config
		true,                      // has_factory_accel_offsets
		true,                      // has_fifo_temperature
	},
	{
		makeSpi(
			"icm20608g",
			DRV_IMU_DEVTYPE_ICM20608G,
			8 * imu::MHz,
			8 * imu::MHz
		),
		Variant::kIcm20608G,      // variant
		0xaf,                     // whoami
		512,                      // fifo_size
		FIFO_PACKET_SIZE_CLASSIC, // fifo_packet_size
		6,                        // gyro_offset
		2,                        // samples_per_transfer
		326.8f,                   // temperature_sensitivity
		25.f,                     // temperature_offset
		kSleep,                   // reset_pwr_value
		100_ms,                   // reset_wait_us
		100_ms,                   // configure_wait_us
	},
	{
		makeSpi(
			"icm20689",
			DRV_IMU_DEVTYPE_ICM20689,
			8 * imu::MHz,
			8 * imu::MHz
		),
		Variant::kIcm20689, // variant
		0x98,               // whoami
	},
	{
		makeSpi(
			"iam20680hp",
			DRV_IMU_DEVTYPE_IAM20680HP,
			8 * imu::MHz,
			8 * imu::MHz
		),
		Variant::kIam20680HP,     // variant
		0xf8,                     // whoami
		512,                      // fifo_size
		FIFO_PACKET_SIZE_CLASSIC, // fifo_packet_size
		6,                        // gyro_offset
		2,                        // samples_per_transfer
		326.8f,                   // temperature_sensitivity
		25.f,                     // temperature_offset
		0,                        // reset_pwr_value
		100_ms,                   // reset_wait_us
		35_ms,                    // configure_wait_us
		false,                    // check_reset_pwr
	},
};
} // namespace

void MPUDirect::print_usage()
{
	PRINT_MODULE_DESCRIPTION(R"DESCR(
### Description
SPI driver for TDK flat-register six-axis IMUs with fixed-stride FIFOs
(MPU6000, MPU6500, ICM20602, ICM20608G, ICM20689, IAM20680HP).
The family shares acquisition and native accel/gyro FIFO publication, while
initialization, sample repetition and SPI timing limits remain model-specific.

Start requires an exact -T model; stop/status may omit -T to visit all running
family instances matching the native bus selectors.

### Examples
```
mpu_direct -T mpu6000 -s start
mpu_direct -T icm20602 status
mpu_direct stop
```
)DESCR");
	PRINT_MODULE_USAGE_NAME("mpu_direct", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('T', nullptr,
					"mpu6000 | mpu6500 | icm20602 | icm20608g | icm20689 | iam20680hp",
					"Exact model (required on start, optional on stop/status)", false);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, ROTATION_MAX - 1, "Rotation", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop instances; omit -T for all family models");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print instances; omit -T for all family models");
}

extern "C" int mpu_direct_main(int argc, char *argv[])
{
	return imu::spiFamilyMain<MPUDirect>(argc, argv, MODULE_NAME, kModels);
}
