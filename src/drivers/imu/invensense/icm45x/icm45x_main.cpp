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
 * @file icm45x_main.cpp
 * @brief Module entry point and model profile table for icm45x.
 */

#include "ICM45x.hpp"
#include "ICM45xRegisters.hpp"

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>

namespace
{

using Variant = ICM45x::Variant;
using namespace icm45x_registers;

constexpr imu::SpiModel makeSpi(
	const char *name,
	uint16_t device_type,
	int frequency_hz,
	uint8_t packet_size,
	int min_clk_hz = 0,
	int max_clk_hz = 0)
{
	imu::SpiModel model {};
	model.name               = name;
	model.device_type        = device_type;
	model.frequency          = frequency_hz;
	model.data_frequency     = frequency_hz;
	model.allow_mode0        = true;
	model.min_clock_hz       = min_clk_hz;
	model.max_clock_hz       = max_clk_hz;
	model.max_transfer_bytes = ICM45x::maxTransferSize(packet_size, 1);
	model.data_prefix_bytes  = 1;
	return model;
}

constexpr ICM45x::Profile kModels[] {
	{
		makeSpi(
			"icm45686",
			DRV_IMU_DEVTYPE_ICM45686,
			24 * imu::MHz,
			icm45686::FIFO_PACKET_BYTES,
			20 * imu::kHz, // min_clock_hz
			40 * imu::kHz  // max_clock_hz
		),
		Variant::kIcm45686,                                   // variant
		icm45686::WHOAMI,                                     // whoami
		icm45686::FIFO_SIZE,                                  // fifo_capacity
		6400,                                                 // output_data_rate_hz
		icm45686::FIFO_PACKET_BYTES,                          // packet_size
		32.f,                                                 // accel_range_g
		4000.f,                                               // gyro_range_dps
		TEMPERATURE_SENSITIVITY,                              // temperature_sensitivity
		TEMPERATURE_OFFSET,                                   // temperature_offset
		static_cast<uint8_t>(icm45686::BANK_0::FIFO_COUNT_0), // fifo_count_reg
		static_cast<uint8_t>(icm45686::BANK_0::FIFO_DATA),    // fifo_data_reg
		static_cast<uint8_t>(icm45686::BANK_0::TEMP_DATA1_UI), // temperature_reg
		true,                                                 // double_read_fifo_count
		false,                                                // fifo_temperature
		false,                                                // periodic_register_check
		true,                                                 // clock_input
		false,                                                // supports_int2
	},
	{
		makeSpi(
			"icm56686",
			DRV_IMU_DEVTYPE_ICM56686,
			8 * imu::MHz,
			icm56686::FIFO_PACKET_BYTES
		),
		Variant::kIcm56686,                                   // variant
		icm56686::WHOAMI,                                     // whoami
		icm56686::FIFO_SIZE,                                  // fifo_capacity
		6400,                                                 // output_data_rate_hz
		icm56686::FIFO_PACKET_BYTES,                          // packet_size
		32.f,                                                 // accel_range_g
		4000.f,                                               // gyro_range_dps
		TEMPERATURE_SENSITIVITY,                              // temperature_sensitivity
		TEMPERATURE_OFFSET,                                   // temperature_offset
		static_cast<uint8_t>(icm56686::BANK_0::FIFO_COUNT_0), // fifo_count_reg
		static_cast<uint8_t>(icm56686::BANK_0::FIFO_DATA),    // fifo_data_reg
		0,                                                    // temperature_reg
		false,                                                // double_read_fifo_count
		true,                                                 // fifo_temperature
		true,                                                 // periodic_register_check
		false,                                                // clock_input
		true,                                                 // supports_int2
	},
};

} // namespace

void ICM45x::print_usage()
{
	PRINT_MODULE_DESCRIPTION(R"DESCR(
### Description
SPI driver for the Gen-4 IREG little-endian FIFO TDK ICM-45686 and ICM-56686 family.
Start requires an exact -T model; stop and status accept an optional -T filter.
)DESCR");
	PRINT_MODULE_USAGE_NAME("icm45x", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('T', nullptr, "icm45686|icm56686",
					"Exact model (required on start, optional on stop/status)", false);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, ROTATION_MAX - 1, "Rotation", true);
	PRINT_MODULE_USAGE_PARAM_INT('C', 0, 0, 1 * imu::MHz,
				     "External clock frequency in Hz (ICM-45686: 20000..40000, 0: disabled)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('2', "Route the FIFO interrupt to INT2 instead of INT1 (ICM-56686 only)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int icm45x_main(int argc, char *argv[])
{
	bool int2 = false;

	return imu::spiFamilyMainCustom<ICM45x>(
		       argc,
		       argv,
		       MODULE_NAME,
		       kModels,
		       imu::SpiCliCapabilities { true, false },
		       "2",
	[&int2](int ch, const char *) {
		if (ch == '2') {
			int2 = true;
			return true;
		}

		return false;
	},
	[&int2](const ICM45x::Profile & model, imu::SpiCommand command) {
		if (command != imu::SpiCommand::kStart && int2) {
			PX4_ERR("-2 INT2 routing option applies only to start");
			return imu::SpiFilterResult::kInvalid;
		}

		if (command == imu::SpiCommand::kStart && int2 && !model.supports_int2) {
			PX4_ERR("%s does not support -2 INT2 interrupt routing", model.device.name);
			return imu::SpiFilterResult::kInvalid;
		}

		return imu::SpiFilterResult::kMatch;
	},
	[&int2](const ICM45x::Profile & model, const imu::SpiOptions &) {
		return ICM45xOptions { &model, int2 };
	});
}
