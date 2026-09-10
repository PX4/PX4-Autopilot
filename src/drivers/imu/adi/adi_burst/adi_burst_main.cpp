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

#include "AdiBurst.hpp"
#include <drivers/drv_hrt.h>

using namespace frequency_literals;
using namespace time_literals;
using adi_burst_decoder::Format;

namespace
{
constexpr uint16_t kRange125  { 1u << 0 };
constexpr uint16_t kRange500  { 1u << 1 };
constexpr uint16_t kRange2000 { 1u << 3 };

#if defined(CONFIG_ADI_BURST_ADIS16500)
constexpr AdiBurstModel adis16500Profile()
{
	AdiBurstModel model {};

	model.device.name		= "adis16500";
	model.device.device_type	= DRV_IMU_DEVTYPE_ADIS1650X;
	model.device.frequency		= 1100_kHz;
	model.device.data_frequency	= 1100_kHz;
	model.device.mode		= SPIDEV_MODE3;

	model.format = Format::kBurst32;

	model.device.max_transfer_bytes		= adi_burst_decoder::size(model.format);
	model.device.data_prefix_bytes		= adi_burst_decoder::kCommandBytes;
	model.device.max_clock_hz		= 0;
	model.device.register_dummy_bytes	= 0;
	model.device.continuous_data_cs		= true;

	model.instance_key = "adis16500";
	model.product_id   = 0x4074;

	model.accel_range = 40.f * CONSTANTS_ONE_G; // m/s^2
	model.accel_scale	= CONSTANTS_ONE_G / 52428800.f; // (m/s^2) per raw count

	model.gyro_range_mask	= kRange2000;
	model.stall_us		= 16;
	model.range_450_dps	= false;

	model.self_test_wait_us = 24_ms; // Original ADIS1650x probe self-test.

	return model;
}
#endif // CONFIG_ADI_BURST_ADIS16500

#if defined(CONFIG_ADI_BURST_ADIS16501)
constexpr AdiBurstModel adis16501Profile()
{
	AdiBurstModel model {};

	model.device.name           = "adis16501";
	model.device.device_type    = DRV_IMU_DEVTYPE_ADIS1650X;
	model.device.frequency      = 1100_kHz;
	model.device.data_frequency = 1100_kHz;
	model.device.mode           = SPIDEV_MODE3;

	model.format = Format::kBurst32;

	model.device.max_transfer_bytes   = adi_burst_decoder::size(model.format);
	model.device.data_prefix_bytes    = adi_burst_decoder::kCommandBytes;
	model.device.max_clock_hz         = 0;
	model.device.register_dummy_bytes = 0;
	model.device.continuous_data_cs   = true;

	model.instance_key = "adis16501";
	model.product_id   = 0x5fb5; // ADIS16501 Rev. B identifies as 24501.

	model.accel_range = 14.f * CONSTANTS_ONE_G; // m/s^2
	model.accel_scale = CONSTANTS_ONE_G / 52428800.f; // (m/s^2) per raw count

	model.gyro_range_mask = kRange500;
	model.stall_us        = 16;
	model.range_450_dps   = false;

	model.self_test_wait_us = 50_ms; // Rev. B specifies 29 ms typical; allow startup margin before reading diagnostics.

	return model;
}
#endif // CONFIG_ADI_BURST_ADIS16501

#if defined(CONFIG_ADI_BURST_ADIS16505)
constexpr AdiBurstModel adis16505Profile()
{
	AdiBurstModel model {};

	model.device.name           = "adis16505";
	model.device.device_type    = DRV_IMU_DEVTYPE_ADIS1650X;
	model.device.frequency      = 1100_kHz;
	model.device.data_frequency = 1100_kHz;
	model.device.mode           = SPIDEV_MODE3;

	model.format = Format::kBurst32;

	model.device.max_transfer_bytes   = adi_burst_decoder::size(model.format);
	model.device.data_prefix_bytes    = adi_burst_decoder::kCommandBytes;
	model.device.max_clock_hz         = 0;
	model.device.register_dummy_bytes = 0;
	model.device.continuous_data_cs   = true;

	model.instance_key = "adis16505";
	model.product_id   = 0x4079;

	model.accel_range = 8.f * CONSTANTS_ONE_G; // m/s^2
	model.accel_scale = CONSTANTS_ONE_G / 262144000.f; // (m/s^2) per raw count

	model.gyro_range_mask = kRange125 | kRange500 | kRange2000;
	model.stall_us        = 16;
	model.range_450_dps   = false;

	model.self_test_wait_us = 24_ms; // Original ADIS1650x probe self-test.

	return model;
}
#endif // CONFIG_ADI_BURST_ADIS16505

#if defined(CONFIG_ADI_BURST_ADIS16507)
constexpr AdiBurstModel adis16507Profile()
{
	AdiBurstModel model {};

	model.device.name           = "adis16507";
	model.device.device_type    = DRV_IMU_DEVTYPE_ADIS1650X;
	model.device.frequency      = 1100_kHz;
	model.device.data_frequency = 1100_kHz;
	model.device.mode           = SPIDEV_MODE3;

	model.format = Format::kBurst32;

	model.device.max_transfer_bytes   = adi_burst_decoder::size(model.format);
	model.device.data_prefix_bytes    = adi_burst_decoder::kCommandBytes;
	model.device.max_clock_hz         = 0;
	model.device.register_dummy_bytes = 0;
	model.device.continuous_data_cs   = true;

	model.instance_key = "adis16507_32";
	model.product_id   = 0x407b;

	model.accel_range = 40.f * CONSTANTS_ONE_G; // m/s^2
	model.accel_scale = CONSTANTS_ONE_G / 52428800.f; // (m/s^2) per raw count

	model.gyro_range_mask = kRange125 | kRange500 | kRange2000;
	model.stall_us        = 16;
	model.range_450_dps   = false;

	model.self_test_wait_us = 24_ms; // Original ADIS1650x probe self-test.

	return model;
}
#endif // CONFIG_ADI_BURST_ADIS16507

#if defined(CONFIG_ADI_BURST_ADIS16507_16)
constexpr AdiBurstModel adis16507Mode16Profile()
{
	AdiBurstModel model {};

	model.device.name           = "adis16507";
	model.device.device_type    = DRV_IMU_DEVTYPE_ADIS16507;
	model.device.frequency      = 2_MHz;
	model.device.data_frequency = 1_MHz;
	model.device.mode           = SPIDEV_MODE3;

	model.format = Format::kBurst16;

	model.device.max_transfer_bytes   = adi_burst_decoder::size(model.format);
	model.device.data_prefix_bytes    = adi_burst_decoder::kCommandBytes;
	model.device.max_clock_hz         = 0;
	model.device.register_dummy_bytes = 0;
	model.device.continuous_data_cs   = true;

	model.instance_key = "adis16507_16";
	model.product_id   = 0x407b;

	model.accel_range = 392.f; // m/s^2
	model.accel_scale = 392.f / 32000.f; // (m/s^2) per raw count

	model.gyro_range_mask = kRange125 | kRange500 | kRange2000;
	model.stall_us        = 16;
	model.range_450_dps   = false;

	return model;
}
#endif // CONFIG_ADI_BURST_ADIS16507_16

#if defined(CONFIG_ADI_BURST_ADIS16575)
constexpr AdiBurstModel adis16575Profile()
{
	AdiBurstModel model {};

	model.device.name           = "adis16575";
	model.device.device_type    = DRV_IMU_DEVTYPE_ADIS1657X;
	model.device.frequency      = 2_MHz;
	model.device.data_frequency = 2_MHz;
	model.device.mode           = SPIDEV_MODE3;

	model.format = Format::kTimestamp32;

	model.device.max_transfer_bytes   = adi_burst_decoder::size(model.format);
	model.device.data_prefix_bytes    = adi_burst_decoder::kCommandBytes;
	model.device.max_clock_hz         = 0;
	model.device.register_dummy_bytes = 0;
	model.device.continuous_data_cs   = true;

	model.instance_key    = "adis16575";
	model.product_id      = 0x40bf;
	model.hard_fault_mask = 0xbf65; // MCU, six sensor axes, memory/self-test, flash update and sensor-init faults.

	model.accel_range = 8.f * CONSTANTS_ONE_G; // m/s^2
	model.accel_scale = CONSTANTS_ONE_G / 262144000.f; // (m/s^2) per raw count

	model.gyro_range_mask = kRange500;
	model.stall_us        = 5;
	model.range_450_dps   = true;

	model.self_test_wait_us = 50_ms; // Host guard above the 19 ms typical time; no completion flag is specified.

	return model;
}
#endif // CONFIG_ADI_BURST_ADIS16575

#if defined(CONFIG_ADI_BURST_ADIS16576)
constexpr AdiBurstModel adis16576Profile()
{
	AdiBurstModel model {};

	model.device.name           = "adis16576";
	model.device.device_type    = DRV_IMU_DEVTYPE_ADIS1657X;
	model.device.frequency      = 2_MHz;
	model.device.data_frequency = 2_MHz;
	model.device.mode           = SPIDEV_MODE3;

	model.format = Format::kTimestamp32;

	model.device.max_transfer_bytes   = adi_burst_decoder::size(model.format);
	model.device.data_prefix_bytes    = adi_burst_decoder::kCommandBytes;
	model.device.max_clock_hz         = 0;
	model.device.register_dummy_bytes = 0;
	model.device.continuous_data_cs   = true;

	model.instance_key    = "adis16576";
	model.product_id      = 0x40c0;
	model.hard_fault_mask = 0xbf65; // ADIS1657x DIAG_STAT, independent of the selected wire width.

	model.accel_range = 14.f * CONSTANTS_ONE_G; // Rev. A Table 5: 14 g range, independently of the 32-bit sensitivity.
	model.accel_scale = CONSTANTS_ONE_G / 52428800.f; // (m/s^2) per raw count

	model.gyro_range_mask = kRange500 | kRange2000; // ADIS16576-2 (450 dps) and ADIS16576-3 (2000 dps).
	model.stall_us        = 5;
	model.range_450_dps   = true;

	model.self_test_wait_us = 50_ms; // Host guard above the 19 ms typical time; no completion flag is specified.

	return model;
}
#endif // CONFIG_ADI_BURST_ADIS16576

#if defined(CONFIG_ADI_BURST_ADIS16577)
constexpr AdiBurstModel adis16577Profile()
{
	AdiBurstModel model {};

	model.device.name           = "adis16577";
	model.device.device_type    = DRV_IMU_DEVTYPE_ADIS1657X;
	model.device.frequency      = 2_MHz;
	model.device.data_frequency = 2_MHz;
	model.device.mode           = SPIDEV_MODE3;

	model.format = Format::kTimestamp32;

	model.device.max_transfer_bytes   = adi_burst_decoder::size(model.format);
	model.device.data_prefix_bytes    = adi_burst_decoder::kCommandBytes;
	model.device.max_clock_hz         = 0;
	model.device.register_dummy_bytes = 0;
	model.device.continuous_data_cs   = true;

	model.instance_key    = "adis16577";
	model.product_id      = 0x40c1;
	model.hard_fault_mask = 0xbf65; // ADIS1657x DIAG_STAT, independent of the selected wire width.

	model.accel_range = 40.f * CONSTANTS_ONE_G; // m/s^2
	model.accel_scale = CONSTANTS_ONE_G / 52428800.f; // (m/s^2) per raw count

	model.gyro_range_mask = kRange500 | kRange2000; // ADIS16577-2 (450 dps) and ADIS16577-3 (2000 dps).
	model.stall_us        = 5;
	model.range_450_dps   = true;

	model.self_test_wait_us = 50_ms; // Host guard above the 19 ms typical time; no completion flag is specified.

	return model;
}
#endif // CONFIG_ADI_BURST_ADIS16577

#if defined(CONFIG_ADI_BURST_ADIS16470)
constexpr AdiBurstModel adis16470Profile()
{
	AdiBurstModel model {};

	model.device.name           = "adis16470";
	model.device.device_type    = DRV_IMU_DEVTYPE_ADIS16470;
	model.device.frequency      = 2_MHz;
	model.device.data_frequency = 1_MHz;
	model.device.mode           = SPIDEV_MODE3;

	model.variant = AdiBurstVariant::kAdis16470;
	model.format  = Format::kBurst16;

	model.device.max_transfer_bytes = adi_burst_decoder::size(model.format);
	model.device.data_prefix_bytes  = adi_burst_decoder::kCommandBytes;
	model.device.continuous_data_cs = true;

	model.instance_key = "adis16470";
	model.product_id   = 0x4056;

	model.accel_range       = 40.f * CONSTANTS_ONE_G;
	model.accel_scale       = CONSTANTS_ONE_G / 800.f;
	model.stall_us          = 16;
	model.reset_wait_us     = 193_ms;
	model.self_test_wait_us = 14_ms;

	return model;
}
#endif // CONFIG_ADI_BURST_ADIS16470

#if defined(CONFIG_ADI_BURST_ADIS16477)
constexpr AdiBurstModel adis16477Profile()
{
	AdiBurstModel model {};

	model.device.name           = "adis16477";
	model.device.device_type    = DRV_IMU_DEVTYPE_ADIS16477;
	model.device.frequency      = 1_MHz;
	model.device.data_frequency = 1_MHz;
	model.device.mode           = SPIDEV_MODE3;

	model.variant = AdiBurstVariant::kAdis16477;
	model.format  = Format::kBurst16;

	model.device.max_transfer_bytes = adi_burst_decoder::size(model.format);
	model.device.data_prefix_bytes  = adi_burst_decoder::kCommandBytes;
	model.device.continuous_data_cs = true;

	model.instance_key = "adis16477";
	model.product_id   = 0x405d;

	model.accel_range       = 40.f * CONSTANTS_ONE_G;
	model.accel_scale       = CONSTANTS_ONE_G / 800.f;
	model.stall_us          = 16;
	model.reset_wait_us     = 193_ms;
	model.self_test_wait_us = 14_ms;

	return model;
}
#endif // CONFIG_ADI_BURST_ADIS16477

#if defined(CONFIG_ADI_BURST_ADIS16497)
constexpr AdiBurstModel adis16497Profile()
{
	AdiBurstModel model {};

	model.device.name           = "adis16497";
	model.device.device_type    = DRV_IMU_DEVTYPE_ADIS16497;
	model.device.frequency      = 5_MHz;
	model.device.data_frequency = 5_MHz;
	model.device.mode           = SPIDEV_MODE3;

	model.variant = AdiBurstVariant::kAdis16497;
	model.format  = Format::kCrc32;

	model.device.max_transfer_bytes = adi_burst_decoder::size(model.format);
	model.device.data_prefix_bytes  = 6; // Minimum prefix; the decoder also accepts a second BURST_ID.
	model.device.continuous_data_cs = true;

	model.instance_key    = "adis16497";
	model.product_id      = 0x4071;
	model.hard_fault_mask = (1u << 1) | (1u << 5) | (1u << 6); // SYS_E_FLAG: boot memory, sensor and flash-update faults.

	model.accel_range       = 40.f * CONSTANTS_ONE_G;
	model.accel_scale       = (1.25f * CONSTANTS_ONE_G / 1000.f) / 65536.f; // Original 1.25 mg/high-word count.
	model.stall_us          = 5;
	model.reset_wait_us     = 300_ms; // Host guard: 210 ms SPI stall; data startup is 225/265 ms typical (SW/HW reset).
	model.self_test_wait_us = 20_ms;

	return model;
}
#endif // CONFIG_ADI_BURST_ADIS16497

constexpr AdiBurstModel kModels[] {
#if defined(CONFIG_ADI_BURST_ADIS16470)
	adis16470Profile(),
#endif // CONFIG_ADI_BURST_ADIS16470

#if defined(CONFIG_ADI_BURST_ADIS16477)
	adis16477Profile(),
#endif // CONFIG_ADI_BURST_ADIS16477

#if defined(CONFIG_ADI_BURST_ADIS16497)
	adis16497Profile(),
#endif // CONFIG_ADI_BURST_ADIS16497

#if defined(CONFIG_ADI_BURST_ADIS16500)
	adis16500Profile(),
#endif // CONFIG_ADI_BURST_ADIS16500

#if defined(CONFIG_ADI_BURST_ADIS16501)
	adis16501Profile(),
#endif // CONFIG_ADI_BURST_ADIS16501

#if defined(CONFIG_ADI_BURST_ADIS16505)
	adis16505Profile(),
#endif // CONFIG_ADI_BURST_ADIS16505

#if defined(CONFIG_ADI_BURST_ADIS16507)
	adis16507Profile(),
#endif // CONFIG_ADI_BURST_ADIS16507

#if defined(CONFIG_ADI_BURST_ADIS16507_16)
	adis16507Mode16Profile(),
#endif // CONFIG_ADI_BURST_ADIS16507_16

#if defined(CONFIG_ADI_BURST_ADIS16575)
	adis16575Profile(),
#endif // CONFIG_ADI_BURST_ADIS16575

#if defined(CONFIG_ADI_BURST_ADIS16576)
	adis16576Profile(),
#endif // CONFIG_ADI_BURST_ADIS16576

#if defined(CONFIG_ADI_BURST_ADIS16577)
	adis16577Profile(),
#endif // CONFIG_ADI_BURST_ADIS16577
};
static_assert(sizeof(kModels) > 0, "Select at least one ADI burst model");
}

void AdiBurst::print_usage()
{
	PRINT_MODULE_DESCRIPTION(R"DESCR(
### Description
SPI single-sample burst driver for ADIS16470, ADIS16477, ADIS16497, ADIS1650x and ADIS1657x.
Burst validation and native single-sample publication are shared.
ADIS16470/16477 use 16-bit bursts; ADIS16497 uses a paged CRC32 burst.
ADIS165xx starts in 32-bit mode; ADIS16507 also supports explicit -B 16.
-r decimation and -F filter options apply only to ADIS165xx 32-bit modes.
On stop/status, omitting -B selects all compiled burst formats for the selected model(s).

Model availability depends on Kconfig. Start requires an exact -T model; stop/status
may omit -T to visit all compiled family instances matching the native bus selectors.
Without a bus selector, start uses board-registered internal SPI devices of the selected type.

### Examples
```
adi_burst -T adis16507 -B 32 start
adi_burst -T adis16507 -B 16 status
adi_burst stop
```
)DESCR");
	PRINT_MODULE_USAGE_NAME("adi_burst", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('T', nullptr,
		"adis16470 | adis16477 | adis16497 | adis16500 | adis16501 | adis16505 | adis16507 | adis16575 | adis16576 | adis16577",
		"Exact model (required for start; availability depends on Kconfig)", false);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, ROTATION_MAX - 1, "Rotation", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 1999, "ADIS165xx 32-bit only: ODR = 2000/(r+1) Hz", true);
	PRINT_MODULE_USAGE_PARAM_INT('F', 0, 0, 6, "ADIS165xx 32-bit only: Bartlett filter setting", true);
	PRINT_MODULE_USAGE_PARAM_STRING('B', nullptr, "16 | 32", "Burst bits (omit for model default)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop instances; omit -T for all compiled family models");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print instances; omit -T for all compiled family models");

	for (const auto &model : kModels) {
		PX4_INFO("-T %s -B %u", model.device.name, model.format == Format::kBurst16 ? 16u : 32u);
	}
}

extern "C" int adi_burst_main(int argc, char *argv[])
{
	BusCLIArguments cli { false, true };

	cli.default_spi_frequency = 0;

	const char *type         = nullptr;
	int        bits          = 0;
	bool       rate_option   = false;
	bool       filter_option = false;
	AdiBurstOptions options {};
	int ch;

	while ((ch = cli.getOpt(argc, argv, "T:R:r:F:B:")) != EOF) {
		switch (ch) {
		case 'T': {
				type = cli.optArg();
				break;
			}

		case 'R': {
				int rotation;

				if (!imu::parseInteger(cli.optArg(), 0, ROTATION_MAX - 1, rotation)) {
					return PX4_ERROR;
				}

				cli.rotation = static_cast<Rotation>(rotation);
				break;
			}

		case 'r': {
				rate_option = true;

				if (!imu::parseInteger(cli.optArg(), 0, 1999, options.decimation)) {
					return PX4_ERROR;
				}

				break;
			}

		case 'F': {
				filter_option = true;

				if (!imu::parseInteger(cli.optArg(), 0, 6, options.filter)) {
					return PX4_ERROR;
				}

				break;
			}

		case 'B': {
				if (!imu::parseInteger(cli.optArg(), 16, 32, bits) || (bits != 16 && bits != 32)) {
					return PX4_ERROR;
				}

				break;
			}

		default: {
				AdiBurst::print_usage();

				return PX4_ERROR;
			}
		}
	}

	const imu::SpiCommand command = imu::parseSpiCommand(cli.optArg());
	const bool            start   = command == imu::SpiCommand::kStart;

	if (command == imu::SpiCommand::kInvalid || (start && !type)) {
		AdiBurst::print_usage();

		return PX4_ERROR;
	}

	int result = PX4_ERROR;

	for (const auto &model : kModels) {
		if ((type && strcmp(type, model.device.name) != 0)
		    || (bits && bits != (model.format == Format::kBurst16 ? 16 : 32))) {
			continue;
		}

		if (start
		    && bits == 0
		    && model.device.device_type == DRV_IMU_DEVTYPE_ADIS16507) {
			continue; // ADIS16507 defaults to its native 32-bit path, never both modes.
		}

		if (start
		    && (model.variant != AdiBurstVariant::kAdis165xx || model.format == Format::kBurst16)
		    && (rate_option || filter_option)) {
			PX4_ERR("filter and decimation options apply to ADIS165xx 32-bit modes");

			return PX4_ERROR;
		}

		options.model = &model;
		// The constructor copies options; only the immutable model has static lifetime.
		cli.custom_data = &options;
		// Existing ADIS1650X/1657X board IDs each cover several models. Distinct
		// instance keys keep typed stop/status from affecting another exact model.
		const int ret = imu::dispatchSpiCommand<AdiBurst>(command, cli, model.instance_key, model.device);

		if (ret == PX4_OK) {
			result = PX4_OK;
		}
	}

	return result;
}
