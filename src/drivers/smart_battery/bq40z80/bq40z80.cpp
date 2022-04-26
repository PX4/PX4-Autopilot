/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file bq40z80.cpp
 *
 * Driver for TI BQ40Z80 connected via SMBus (I2C). Loosely based on legacy
 * batt_smbus driver by Jacob Dahl <dahl.jakejacob@gmail.com>, Alex Klimaj
 * <alexklimaj@gmail.com> and Bazooka Joe <BazookaJoe1900@gmail.com>
 *
 * @author Alex Mikhalev <alex@corvus-robotics.com>
 * @author Mohammed Kabir <kabir@corvus-robotics.com>
 *
 */

#include "bq40z80.h"

#include <mathlib/mathlib.h>

using namespace time_literals;

extern "C" __EXPORT int bq40z80_main(int argc, char *argv[]);

BQ40Z80::BQ40Z80(const I2CSPIDriverConfig &config, SMBus *interface) :
	SBSBattery(config, interface)
{
	_cycle = perf_alloc(PC_ELAPSED, "bq40z80_cycle");
}

BQ40Z80::~BQ40Z80()
{
}

int BQ40Z80::populate_cell_voltages(battery_status_s &data)
{
	uint8_t DAstatus1[32] = {}; // 32 bytes of data

	if (PX4_OK != manufacturer_read(BQ40Z80_MAN_DASTATUS1, DAstatus1, sizeof(DAstatus1))) {
		return PX4_ERROR;
	}

	// Cells 1-4
	for (int i = 0; i < math::min(4, int(_cell_count)); i++) {
		// convert mV -> volts
		data.voltage_cell_v[i] = ((float)((DAstatus1[2 * i + 1] << 8) | DAstatus1[2 * i]) / 1000.0f);
	}

	uint8_t DAstatus3[18] = {}; // 18 bytes of data

	if (PX4_OK != manufacturer_read(BQ40Z80_MAN_DASTATUS3, DAstatus3, sizeof(DAstatus3))) {
		return PX4_ERROR;
	}

	// Cells 5-7
	if (_cell_count >= 5) {
		data.voltage_cell_v[4] = ((float)((DAstatus3[1] << 8) | DAstatus3[0]) / 1000.0f);
	}

	if (_cell_count >= 6) {
		data.voltage_cell_v[5] = ((float)((DAstatus3[7] << 8) | DAstatus3[6]) / 1000.0f);
	}

	if (_cell_count >= 7) {
		data.voltage_cell_v[6] = ((float)((DAstatus3[13] << 8) | DAstatus3[12]) / 1000.0f);
	}

	return PX4_OK;
}

int BQ40Z80::set_in_air_protection_mode(bool enable)
{
	if (_in_air_protection_state == enable) {
		return PX4_OK;
	}

	PX4_DEBUG("Turning in air protection mode %s", enable ? "on" : "off");
	_in_air_protection_state = enable;

	// When in-air protection is _enabled_, we _disable_ certain BMS protections to avoid
	// them potentially falsely triggering in flight and causing in-flight powerloss

	// Enabled Protections A, B
	uint8_t protections[2] = {
		BQ40Z80_ENABLED_PROTECTIONS_A_DEFAULT,
		BQ40Z80_ENABLED_PROTECTIONS_B_DEFAULT
	};

	if (enable) {
		protections[0] &= ~BQ40Z80_ENABLED_PROTECTIONS_A_CUV;
		// TODO: also disable OCD1/2 (overcurrent in discharge)?
		protections[1] &= ~BQ40Z80_ENABLED_PROTECTIONS_B_CUVC;
		// TODO: also disable OTD (overtemperature in discharge)?
	}

	return dataflash_write(BQ40Z80_MAN_ENABLED_PROTECTIONS_A_ADDRESS, protections, 2);
}

int BQ40Z80::set_mfc_shutdown_mode(bool enable)
{
	int ret = PX4_OK;

	if (enable) {
		ret = manufacturer_write(BQ40Z80_MAN_MFC_SHUTDOWN_ENABLE_A, nullptr, 0);

		if (ret != PX4_OK) { return ret; }

		ret = manufacturer_write(BQ40Z80_MAN_MFC_SHUTDOWN_ENABLE_B, nullptr, 0);

	} else {
		ret = manufacturer_write(BQ40Z80_MAN_MFC_SHUTDOWN_DISABLE, nullptr, 0);

	}

	return ret;
}

int BQ40Z80::dataflash_read(const uint16_t address, void *data, const unsigned length)
{
	if (length > MAC_DATA_BUFFER_SIZE) {
		return -EINVAL;
	}

	uint8_t code = BQ40Z80_REG_MANUFACTURER_BLOCK_ACCESS;

	uint8_t tx_buf[2] = {};
	tx_buf[0] = ((uint8_t *)&address)[0];
	tx_buf[1] = ((uint8_t *)&address)[1];

	uint8_t rx_buf[MAC_DATA_BUFFER_SIZE + 2];

	int ret = _interface->block_write(code, tx_buf, 2, false);

	if (ret != PX4_OK) {
		return ret;
	}

	// Always returns 32 bytes of data
	ret = _interface->block_read(code, rx_buf, 32 + 2, true);
	memcpy(data, &rx_buf[2], length); // remove the address bytes

	return ret;
}

int BQ40Z80::dataflash_write(const uint16_t address, void *data, const unsigned length)
{
	return manufacturer_write(address, data, length);
}

int BQ40Z80::manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length)
{
	if (length > MAC_DATA_BUFFER_SIZE) {
		return -EINVAL;
	}

	uint8_t code = BQ40Z80_REG_MANUFACTURER_BLOCK_ACCESS;

	uint8_t address[2] = {};
	address[0] = ((uint8_t *)&cmd_code)[0];
	address[1] = ((uint8_t *)&cmd_code)[1];

	uint8_t rx_buf[MAC_DATA_BUFFER_SIZE + 2];

	int ret = _interface->block_write(code, address, 2, false);

	if (ret != PX4_OK) {
		return ret;
	}

	ret = _interface->block_read(code, rx_buf, length + 2, true);
	memcpy(data, &rx_buf[2], length); // remove the address bytes

	return ret;
}

int BQ40Z80::manufacturer_write(const uint16_t cmd_code, void *data, const unsigned length)
{
	uint8_t code = BQ40Z80_REG_MANUFACTURER_BLOCK_ACCESS;

	uint8_t tx_buf[MAC_DATA_BUFFER_SIZE + 2] = {};
	tx_buf[0] = cmd_code & 0xff;
	tx_buf[1] = (cmd_code >> 8) & 0xff;

	if (data != nullptr && length <= MAC_DATA_BUFFER_SIZE) {
		memcpy(&tx_buf[2], data, length);
	}

	int ret = _interface->block_write(code, tx_buf, length + 2, false);

	return ret;
}

int BQ40Z80::lifetime_flush()
{
	return manufacturer_write(BQ40Z80_MAN_LIFETIME_FLUSH, nullptr, 0);
}

int BQ40Z80::lifetime_read()
{
	uint8_t lifetime_block_one[32] = {}; // 32 bytes of data

	if (PX4_OK != manufacturer_read(BQ40Z80_MAN_LIFETIME_BLOCK_ONE, lifetime_block_one, sizeof(lifetime_block_one))) {
		return PX4_ERROR;
	}

	return PX4_OK;
}


int BQ40Z80::populate_startup_data()
{
	int ret = PX4_OK;

	uint16_t device_type = 0;
	ret |= manufacturer_read(BQ40Z80_MAN_DEVICE_TYPE, &device_type, sizeof(device_type));

	if (ret || device_type != BQ40Z80_EXPECTED_DEVICE_TYPE) {
		PX4_DEBUG("BQ40Z80 failed probe (ret: %d, device_type: 0x%04x)", ret, device_type);
		return -EIO;
	}

	// Get generic SBS startup information
	ret |= BaseClass::populate_startup_data();

	uint8_t cell_configuration;
	ret |= dataflash_read(BQ40Z80_FLASH_CELL_CONFIGURATION, &cell_configuration, sizeof(cell_configuration));

	uint16_t state_of_health;
	ret |= _interface->read_word(BQ40Z80_REG_STATE_OF_HEALTH, state_of_health);

	if (ret) {
		PX4_WARN("Failed to read startup info: %d", ret);
		return ret;
	}

	_state_of_health = state_of_health;
	_cell_count = cell_configuration & 0x07;

	// Configure some startup defaults
	ret |= set_mfc_shutdown_mode(false);
	ret |= set_in_air_protection_mode(false);

	return ret;
}

int BQ40Z80::populate_runtime_data(battery_status_s &data)
{
	int ret = PX4_OK;

	ret |= BaseClass::populate_runtime_data(data);

	ret |= populate_cell_voltages(data);

	data.state_of_health = _state_of_health;

	_vehicle_status_sub.update();
	const vehicle_status_s &vstatus = _vehicle_status_sub.get();

	const bool enable_in_air_protection = vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	_in_air_protection_hyst.set_state_and_update(enable_in_air_protection, hrt_absolute_time());
	set_in_air_protection_mode(_in_air_protection_hyst.get_state());

	uint16_t bat_status;
	ret |= _interface->read_word(BQ40Z80_REG_BATTERY_STATUS, bat_status);

	uint32_t op_status;
	ret |= manufacturer_read(BQ40Z80_MAN_OPERATION_STATUS, &op_status, sizeof(op_status));

	uint32_t chg_status;
	ret |= manufacturer_read(BQ40Z80_MAN_CHARGING_STATUS, &chg_status, sizeof(chg_status));

	uint16_t avg_time_to_full;
	ret |= _interface->read_word(SBS_REG_AVERAGE_TIME_TO_FULL, avg_time_to_full);

	if (ret == PX4_OK) {
		if (op_status & BQ40Z80_OPERATION_STATUS_XDSG) {
			// If discharge is disabled, report battery as failed.
			data.warning = battery_status_s::BATTERY_WARNING_FAILED;
		}
	}

	return ret;
}

void BQ40Z80::custom_method(const BusCLIArguments &cli)
{
	if (cli.custom1 == BQ40Z80_CCMD_READ_MAN_BLOCK) {
		uint8_t data[32];
		uint16_t address = static_cast<uint16_t>(reinterpret_cast<uintptr_t>(cli.custom_data));
		unsigned length = cli.custom2;

		int ret;

		if (0x4000 <= address && address <= 0x5FFF) {
			// If address is in dataflash, use dataflash_read since the device
			// always returns 32 bytes.
			ret = dataflash_read(address, data, length);

		} else {
			ret = manufacturer_read(address, data, length);
		}

		if (ret != PX4_OK) {
			PX4_ERR("Failed to read manufacturer data block: %d", ret);
			return;
		}

		char output_buf[32 * 2 + 1];
		unsigned i = 0;

		for (; i < length; ++i) {
			snprintf(&output_buf[2 * i], 3, "%02x", data[i]);
		}

		PX4_INFO("Manufacturer block 0x%04x: %s", address, output_buf);

	} else {
		BaseClass::custom_method(cli);
	}
}

int BQ40Z80::handle_command(const char *verb, BusCLIArguments &cli, BusInstanceIterator &iterator,
			    int argc, char *argv[])
{
	if (!strcmp(verb, "read_man_block")) {
		cli.custom1 = BQ40Z80_CCMD_READ_MAN_BLOCK;

		if (argc == 4) {
			uint16_t address = strtoul(argv[2], nullptr, 0);
			unsigned length = strtoul(argv[3], nullptr, 0);

			if (length > 32) {
				PX4_WARN("Data length out of range: Max 32 bytes");
				return 1;
			}

			cli.custom_data = reinterpret_cast<void *>(address);
			cli.custom2 = length;
			return BQ40Z80::module_custom_method(cli, iterator);
		}
	}

	return BaseClass::handle_command(verb, cli, iterator, argc, argv);
}

void BQ40Z80::print_module_description()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Smart battery driver for the TI BQ40Z80 fuel gauge IC.

)DESCR_STR");
}

void BQ40Z80::print_usage_commands()
{
	BaseClass::print_usage_commands();

	PRINT_MODULE_USAGE_COMMAND_DESCR("read_man_block", "Read manufacturer data block.");
	PRINT_MODULE_USAGE_ARG("<address> <length>", "Address and length (in bytes) of block to read", false);
}

extern "C" __EXPORT int bq40z80_main(int argc, char *argv[])
{
	return BQ40Z80::main(argc, argv);
}
