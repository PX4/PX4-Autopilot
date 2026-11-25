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
 * @file icm42x_main.cpp
 * @brief Module entry point and model profile table for icm42x.
 */

#include "ICM42x.hpp"

#include "../InvenSense_AAF.hpp"
#include "ICM42xRegisters.hpp"

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>

namespace
{

using Variant = ICM42x::Variant;
using PacketFormat = ICM42x::PacketFormat;
using Profile = ICM42x::Profile;
using namespace icm42x_registers;

constexpr imu::SpiModel makeSpi(const char *name, uint16_t device_type, uint8_t packet_size, bool clkin)
{
	imu::SpiModel model {};
	model.name               = name;
	model.device_type        = device_type;
	model.frequency          = 24 * imu::MHz;
	model.data_frequency     = 24 * imu::MHz;
	model.allow_mode0        = true;
	model.min_clock_hz       = clkin ? 31 * imu::kHz : 0;
	model.max_clock_hz       = clkin ? 50 * imu::kHz : 0;
	model.max_transfer_bytes = ICM42x::maxTransferSize(packet_size);
	model.data_prefix_bytes  = icm42x_config::kFifoTransferPrefix;
	return model;
}

constexpr Profile kModels[] {
	{
		makeSpi(
			"icm40609d",
			DRV_IMU_DEVTYPE_ICM40609D,
			FIFO::STANDARD_PACKET_SIZE,
			false // clkin
		),
		Variant::kIcm40609D,        // variant
		0x3b,                       // whoami
		PacketFormat::kStandard16,  // packet_format
		FIFO::STANDARD_PACKET_SIZE, // packet_size
		32.f,                       // accel_range_g
		2000.f,                     // gyro_range_dps
		false,                      // check_reset_done
		true,                       // fifo_count_is_records
		false,                      // clock_input
	},
	{
		makeSpi(
			"icm42605",
			DRV_IMU_DEVTYPE_ICM42605,
			FIFO::STANDARD_PACKET_SIZE,
			false // clkin
		),
		Variant::kIcm42605,         // variant
		0x42,                       // whoami
		PacketFormat::kStandard16,  // packet_format
		FIFO::STANDARD_PACKET_SIZE, // packet_size
		16.f,                       // accel_range_g
		2000.f,                     // gyro_range_dps
		true,                       // check_reset_done
		false,                      // fifo_count_is_records
		false,                      // clock_input
	},
	{
		makeSpi(
			"icm42688p",
			DRV_IMU_DEVTYPE_ICM42688P,
			FIFO::PACKET_SIZE,
			true // clkin
		),
		Variant::kIcm42688P,       // variant
		0x47,                      // whoami
		PacketFormat::kHighRes20,  // packet_format
		FIFO::PACKET_SIZE,         // packet_size
		16.f,                      // accel_range_g
		2000.f,                    // gyro_range_dps
	},
	{
		makeSpi(
			"icm42686p",
			DRV_IMU_DEVTYPE_ICM42686P,
			FIFO::PACKET_SIZE,
			true // clkin
		),
		Variant::kIcm42686P,       // variant
		0x44,                      // whoami
		PacketFormat::kHighRes20,  // packet_format
		FIFO::PACKET_SIZE,         // packet_size
		32.f,                      // accel_range_g
		4000.f,                    // gyro_range_dps
	},
	{
		makeSpi(
			"iim42652",
			DRV_IMU_DEVTYPE_IIM42652,
			FIFO::PACKET_SIZE,
			true // clkin
		),
		Variant::kIim42652,        // variant
		0x6f,                      // whoami
		PacketFormat::kHighRes20,  // packet_format
		FIFO::PACKET_SIZE,         // packet_size
		16.f,                      // accel_range_g
		2000.f,                    // gyro_range_dps
	},
	{
		makeSpi(
			"iim42653",
			DRV_IMU_DEVTYPE_IIM42653,
			FIFO::PACKET_SIZE,
			true // clkin
		),
		Variant::kIim42653,        // variant
		0x56,                      // whoami
		PacketFormat::kHighRes20,  // packet_format
		FIFO::PACKET_SIZE,         // packet_size
		32.f,                      // accel_range_g
		4000.f,                    // gyro_range_dps
	},
};

} // namespace

void ICM42x::print_usage()
{
	PRINT_MODULE_DESCRIPTION(R"DESCR(
### Description
SPI driver for the banked TDK ICM-40609-D, ICM-42605, ICM-4268x and IIM-4265x family.
Start requires an exact -T model; stop and status accept an optional -T filter.
)DESCR");
	PRINT_MODULE_USAGE_NAME("icm42x", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('T', nullptr,
					"icm40609d|icm42605|icm42688p|icm42686p|iim42652|iim42653",
					"Exact model (required on start, optional on stop/status)", false);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, ROTATION_MAX - 1, "Rotation", true);
	PRINT_MODULE_USAGE_PARAM_INT('C', 0, 0, 1 * imu::MHz,
				     "External clock frequency in Hz (ICM-4268x/IIM-4265x: 31000..50000, 0: disabled)", true);
	PRINT_MODULE_USAGE_PARAM_INT('B', 0, 0, 394,
				     "Anti-alias filter bandwidth (ICM-4268x/IIM-4265x: 126, 258 or 394 Hz; 0: default 585 Hz)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int icm42x_main(int argc, char *argv[])
{
	int  aaf_hz        = 0;
	bool aaf_specified = false;

	return imu::spiFamilyMainCustom<ICM42x>(
		       argc,
		       argv,
		       MODULE_NAME,
		       kModels,
		       imu::SpiCliCapabilities { true, false },
		       "B:",
	[&aaf_hz, &aaf_specified](int ch, const char *arg) {
		if (ch == 'B') {
			aaf_specified = true;
			return imu::parseInteger(arg, 0, INT32_MAX, aaf_hz);
		}

		return false;
	},
	[&aaf_hz, &aaf_specified](const ICM42x::Profile & model, imu::SpiCommand command) {
		if (command != imu::SpiCommand::kStart && aaf_specified) {
			PX4_ERR("-B anti-alias filter option applies only to start");
			return imu::SpiFilterResult::kInvalid;
		}

		if (command == imu::SpiCommand::kStart && aaf_hz != 0) {
			if (InvenSense_AAF::preset(static_cast<uint32_t>(aaf_hz)) == nullptr) {
				PX4_ERR("unsupported anti-alias filter bandwidth %d Hz (expected 0, 126, 258 or 394)", aaf_hz);
				return imu::SpiFilterResult::kInvalid;
			}

			if (model.packet_format != PacketFormat::kHighRes20) {
				PX4_ERR("%s does not support -B anti-alias filter presets", model.device.name);
				return imu::SpiFilterResult::kInvalid;
			}
		}

		return imu::SpiFilterResult::kMatch;
	},
	[&aaf_hz](const ICM42x::Profile & model, const imu::SpiOptions &) {
		return ICM42xOptions { &model, static_cast<uint16_t>(aaf_hz) };
	});
}
