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
 * @file adisburst_main.cpp
 * @brief Module entry point and ADIS16470/16477/1650x/1657x profile table for adisburst.
 */

#include "ADISBurst.hpp"

#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>

using namespace time_literals;
using adis_burst_decoder::Format;

namespace
{
constexpr uint16_t kRange125  { 1u << 0 };
constexpr uint16_t kRange500  { 1u << 1 };
constexpr uint16_t kRange2000 { 1u << 3 };

constexpr imu::SpiModel makeSpi(
	const char *name,
	const char *instance_key,
	uint16_t device_type,
	int config_hz,
	int data_hz,
	Format format)
{
	imu::SpiModel model {};
	model.name               = name;
	model.instance_key       = instance_key;
	model.device_type        = device_type;
	model.frequency          = config_hz;
	model.data_frequency     = data_hz;
	model.max_transfer_bytes = adis_burst_decoder::size(format);
	model.data_prefix_bytes  = adis_burst_decoder::kCommandBytes;
	return model;
}

constexpr ADISBurstProfile kModels[] {
	{
		makeSpi(
			"adis16470",
			"adisburst:16470",
			DRV_IMU_DEVTYPE_ADIS16470,
			2 * imu::MHz,
			1 * imu::MHz,
			Format::kBurst16
		),
		ADISBurstVariant::kAdis16470, // variant
		0x4056,                       // product_id
		Format::kBurst16,             // format
		40.f * CONSTANTS_ONE_G,       // accel_range
		CONSTANTS_ONE_G / 800.f,      // accel_scale
		kRange2000,                   // gyro_range_mask
		0,                            // accel_base_delay_us
		0,                            // gyro_base_delay_us
		16,                           // stall_us
		false,                        // range_450_dps
		193_ms,                       // reset_wait_us
		14_ms,                        // self_test_wait_us
		{
			nullptr,
			nullptr,
			nullptr,
			"ADIS16470 (gyro +-2000 deg/s)",
		},
	},
	{
		makeSpi(
			"adis16477",
			"adisburst:16477",
			DRV_IMU_DEVTYPE_ADIS16477,
			2 * imu::MHz,
			1 * imu::MHz,
			Format::kBurst16
		),
		ADISBurstVariant::kAdis16477,       // variant
		0x405d,                             // product_id
		Format::kBurst16,                   // format
		40.f * CONSTANTS_ONE_G,             // accel_range
		CONSTANTS_ONE_G / 800.f,            // accel_scale
		kRange125 | kRange500 | kRange2000, // gyro_range_mask
		0,                                  // accel_base_delay_us
		0,                                  // gyro_base_delay_us
		16,                                 // stall_us
		false,                              // range_450_dps
		193_ms,                             // reset_wait_us
		14_ms,                              // self_test_wait_us
		{
			"ADIS16477-1BMLZ (gyro +-125 deg/s)",
			"ADIS16477-2BMLZ (gyro +-500 deg/s)",
			nullptr,
			"ADIS16477-3BMLZ (gyro +-2000 deg/s)",
		},
	},
	{
		makeSpi(
			"adis16500",
			"adisburst:16500",
			DRV_IMU_DEVTYPE_ADIS1650X,
			2100 * imu::kHz,
			1 * imu::MHz,
			Format::kBurst32
		),
		ADISBurstVariant::kAdis165xx, // variant
		0x4074,                       // product_id
		Format::kBurst32,             // format
		40.f * CONSTANTS_ONE_G,       // accel_range
		CONSTANTS_ONE_G / 52428800.f, // accel_scale
		kRange2000,                   // gyro_range_mask
		1570,                         // accel_base_delay_us
		1437,                         // gyro_base_delay_us
		16,                           // stall_us
		false,                        // range_450_dps
		350_ms,                       // reset_wait_us
		24_ms,                        // self_test_wait_us
		{
			nullptr,
			nullptr,
			nullptr,
			"ADIS16500 (gyro +-2000 deg/s)",
		},
	},
	{
		makeSpi(
			"adis16501",
			"adisburst:16501",
			DRV_IMU_DEVTYPE_ADIS1650X,
			10 * imu::MHz,
			1 * imu::MHz,
			Format::kBurst32
		),
		ADISBurstVariant::kAdis165xx, // variant
		0x5fb5,                       // product_id (Rev. B identifies as 24501)
		Format::kBurst32,             // format
		14.f * CONSTANTS_ONE_G,       // accel_range
		CONSTANTS_ONE_G / 52428800.f, // accel_scale
		kRange500,                    // gyro_range_mask
		1370,                         // accel_base_delay_us (Table 8: 1.37 ms)
		1487,                         // gyro_base_delay_us (Table 8: 1.487 ms)
		16,                           // stall_us
		false,                        // range_450_dps
		350_ms,                       // reset_wait_us
		50_ms,                        // self_test_wait_us
		{
			nullptr,
			"ADIS16501 (gyro +-500 deg/s)",
			nullptr,
			nullptr,
		},
	},
	{
		makeSpi(
			"adis16505",
			"adisburst:16505",
			DRV_IMU_DEVTYPE_ADIS1650X,
			2100 * imu::kHz,
			1 * imu::MHz,
			Format::kBurst32
		),
		ADISBurstVariant::kAdis165xx,    // variant
		0x4079,                          // product_id
		Format::kBurst32,                // format
		8.f * CONSTANTS_ONE_G,           // accel_range
		CONSTANTS_ONE_G / 262144000.f,   // accel_scale
		kRange125 | kRange500 | kRange2000, // gyro_range_mask
		1570,                            // accel_base_delay_us
		1437,                            // gyro_base_delay_us
		16,                              // stall_us
		false,                           // range_450_dps
		350_ms,                          // reset_wait_us
		24_ms,                           // self_test_wait_us
		{
			"ADIS16505-1BMLZ (gyro +-125 deg/s)",
			"ADIS16505-2BMLZ (gyro +-500 deg/s)",
			nullptr,
			"ADIS16505-3BMLZ (gyro +-2000 deg/s)",
		},
	},
	{
		makeSpi(
			"adis16507",
			"adisburst:16507_32",
			DRV_IMU_DEVTYPE_ADIS1650X,
			2100 * imu::kHz,
			1 * imu::MHz,
			Format::kBurst32
		),
		ADISBurstVariant::kAdis165xx,       // variant
		0x407b,                             // product_id
		Format::kBurst32,                   // format
		40.f * CONSTANTS_ONE_G,             // accel_range
		CONSTANTS_ONE_G / 52428800.f,       // accel_scale
		kRange125 | kRange500 | kRange2000, // gyro_range_mask
		1570,                               // accel_base_delay_us
		1437,                               // gyro_base_delay_us
		16,                                 // stall_us
		false,                              // range_450_dps
		350_ms,                             // reset_wait_us
		24_ms,                              // self_test_wait_us
		{
			"ADIS16507-1BMLZ (gyro +-125 deg/s)",
			"ADIS16507-2BMLZ (gyro +-500 deg/s)",
			nullptr,
			"ADIS16507-3BMLZ (gyro +-2000 deg/s)",
		},
	},
	{
		makeSpi(
			"adis16507",
			"adisburst:16507_16",
			DRV_IMU_DEVTYPE_ADIS16507,
			2 * imu::MHz,
			1 * imu::MHz,
			Format::kBurst16
		),
		ADISBurstVariant::kAdis165xx,       // variant
		0x407b,                             // product_id
		Format::kBurst16,                   // format
		392.f,                              // accel_range
		392.f / 32000.f,                    // accel_scale
		kRange125 | kRange500 | kRange2000, // gyro_range_mask
		1570,                               // accel_base_delay_us
		(1510 + 1510 + 1290) / 3,           // gyro_base_delay_us
		16,                                 // stall_us
		false,                              // range_450_dps
		350_ms,                             // reset_wait_us
		50_ms,                              // self_test_wait_us
		{
			"ADIS16507-1BMLZ (gyro +-125 deg/s)",
			"ADIS16507-2BMLZ (gyro +-500 deg/s)",
			nullptr,
			"ADIS16507-3BMLZ (gyro +-2000 deg/s)",
		},
	},
	{
		makeSpi(
			"adis16575",
			"adisburst:16575",
			DRV_IMU_DEVTYPE_ADIS1657X,
			2 * imu::MHz,
			2 * imu::MHz,
			Format::kTimestamp32
		),
		ADISBurstVariant::kAdis165xx,  // variant
		0x40bf,                        // product_id
		Format::kTimestamp32,          // format
		8.f * CONSTANTS_ONE_G,         // accel_range
		CONSTANTS_ONE_G / 262144000.f, // accel_scale
		kRange500,                     // gyro_range_mask
		1570,                          // accel_base_delay_us
		1437,                          // gyro_base_delay_us
		5,                             // stall_us
		true,                          // range_450_dps
		350_ms,                        // reset_wait_us
		50_ms,                         // self_test_wait_us
		{
			nullptr,
			"ADIS16575-2BMLZ (gyro +-450 deg/s)",
			nullptr,
			nullptr,
		},
	},
	{
		makeSpi(
			"adis16576",
			"adisburst:16576",
			DRV_IMU_DEVTYPE_ADIS1657X,
			2 * imu::MHz,
			2 * imu::MHz,
			Format::kTimestamp32
		),
		ADISBurstVariant::kAdis165xx, // variant
		0x40c0,                       // product_id
		Format::kTimestamp32,         // format
		14.f * CONSTANTS_ONE_G,       // accel_range
		CONSTANTS_ONE_G / 52428800.f, // accel_scale
		kRange500 | kRange2000,       // gyro_range_mask
		1570,                         // accel_base_delay_us
		1437,                         // gyro_base_delay_us
		5,                            // stall_us
		true,                         // range_450_dps
		350_ms,                       // reset_wait_us
		50_ms,                        // self_test_wait_us
		{
			nullptr,
			"ADIS16576-2BMLZ (gyro +-450 deg/s)",
			nullptr,
			"ADIS16576-3BMLZ (gyro +-2000 deg/s)",
		},
	},
	{
		makeSpi(
			"adis16577",
			"adisburst:16577",
			DRV_IMU_DEVTYPE_ADIS1657X,
			2 * imu::MHz,
			2 * imu::MHz,
			Format::kTimestamp32
		),
		ADISBurstVariant::kAdis165xx, // variant
		0x40c1,                       // product_id
		Format::kTimestamp32,         // format
		40.f * CONSTANTS_ONE_G,       // accel_range
		CONSTANTS_ONE_G / 52428800.f, // accel_scale
		kRange500 | kRange2000,       // gyro_range_mask
		1570,                         // accel_base_delay_us
		1437,                         // gyro_base_delay_us
		5,                            // stall_us
		true,                         // range_450_dps
		350_ms,                       // reset_wait_us
		50_ms,                        // self_test_wait_us
		{
			nullptr,
			"ADIS16577-2BMLZ (gyro +-450 deg/s)",
			nullptr,
			"ADIS16577-3BMLZ (gyro +-2000 deg/s)",
		},
	},
};
} // namespace

void ADISBurst::print_usage()
{
	PRINT_MODULE_DESCRIPTION(R"DESCR(
### Description
SPI single-sample burst driver for ADIS16470, ADIS16477, ADIS1650x and ADIS1657x.
Burst validation and native single-sample publication are shared.
ADIS16470 and ADIS16477 use 16-bit bursts; ADIS1650x and ADIS1657x start in 32-bit mode.
ADIS16507 defaults to 32-bit mode (-B 32) and also supports explicit -B 16 compatibility.
-r decimation and -F filter options apply only to ADIS165xx 32-bit modes.

Start requires an exact -T model; stop and status accept optional -T and -B filters.

### Examples
```
adisburst -T adis16507 -s start
adisburst -T adis16507 -B 16 -s start
adisburst -T adis16507 status
adisburst stop
```
)DESCR");
	PRINT_MODULE_USAGE_NAME("adisburst", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('T', nullptr,
					"adis16470 | adis16477 | adis16500 | adis16501 | adis16505 | adis16507 | adis16575 | adis16576 | adis16577",
					"Exact model (required on start; optional filter on stop/status)", false);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, ROTATION_MAX - 1, "Rotation", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 1999, "ADIS165xx 32-bit only: ODR = 2000/(r+1) Hz", true);
	PRINT_MODULE_USAGE_PARAM_INT('F', 0, 0, 6, "ADIS165xx 32-bit only: Bartlett filter setting", true);
	PRINT_MODULE_USAGE_PARAM_STRING('B', nullptr, "16 | 32", "Burst bits (omit for model default)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop matching family instances");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status of matching family instances");
}

extern "C" int adisburst_main(int argc, char *argv[])
{
	int  bits          = 0;
	int  decimation    = 0;
	int  filter        = 0;
	bool rate_option   = false;
	bool filter_option = false;

	return imu::spiFamilyMainCustom<ADISBurst>(
		       argc,
		       argv,
		       MODULE_NAME,
		       kModels,
		       imu::SpiCliCapabilities { false, false },
		       "B:r:F:",
	[&](int ch, const char *arg) {
		switch (ch) {
		case 'B': {
				return imu::parseInteger(arg, 16, 32, bits) && (bits == 16 || bits == 32);
			}

		case 'r': {
				rate_option = true;
				return imu::parseInteger(arg, 0, 1999, decimation);
			}

		case 'F': {
				filter_option = true;
				return imu::parseInteger(arg, 0, 6, filter);
			}

		default: {
				return false;
			}
		}
	},
	[&](const ADISBurstProfile & model, imu::SpiCommand command) {
		if (command != imu::SpiCommand::kStart && (rate_option || filter_option)) {
			PX4_ERR("-r and -F options apply only to start");
			return imu::SpiFilterResult::kInvalid;
		}

		const int model_bits = model.format == Format::kBurst16 ? 16 : 32;

		if (bits != 0 && bits != model_bits) {
			return imu::SpiFilterResult::kSkip;
		}

		// Without -B, ADIS16507 start requires the 32-bit profile; never fall back to 16-bit silently.
		if (command == imu::SpiCommand::kStart
		    && bits == 0
		    && model.device.device_type == DRV_IMU_DEVTYPE_ADIS16507) {
			return imu::SpiFilterResult::kSkip;
		}

		if (command == imu::SpiCommand::kStart
		    && (rate_option || filter_option)
		    && model.format == Format::kBurst16) {
			PX4_ERR("filter and decimation options apply to ADIS165xx 32-bit modes");
			return imu::SpiFilterResult::kInvalid;
		}

		return imu::SpiFilterResult::kMatch;
	},
	[&](const ADISBurstProfile & model, const imu::SpiOptions &) {
		return ADISBurstOptions { &model, decimation, filter };
	});
}
