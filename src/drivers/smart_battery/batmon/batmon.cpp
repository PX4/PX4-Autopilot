/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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
 * @file batmon.cpp
 *
 * BatMon module for Smart Battery utilizing SBS 1.1 specifications
 * Setup/usage information: https://rotoye.com/batmon-tutorial/
 *
 * @author Eohan George <eohan@rotoye.com>
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 */

#include "batmon.h"
#include <mathlib/mathlib.h>

extern "C" __EXPORT int batmon_main(int argc, char *argv[]);

Batmon::Batmon(const I2CSPIDriverConfig &config, SMBus *interface):
	SBSBattery(config, interface)
{
}

void Batmon::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Smart battery driver for BatMon
Setup/usage information: https://rotoye.com/batmon-tutorial/
### Examples
To start at address 0x0B, on bus 4
$ batmon start -X -a 11 -b 4

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("batmon", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x0B);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int Batmon::populate_runtime_data(battery_status_s &data)
{
	int ret = BaseClass::populate_runtime_data(data);

	ret |= populate_cell_voltages(data);

	return ret;
}

int Batmon::populate_cell_voltages(battery_status_s &data)
{
	int ret = PX4_OK;

	float cell_voltages[MAX_CELL_COUNT] = {};

	// Making the assumption that the register value of BATT_SMBUS_CELL_1_VOLTAGE and BATT_SMBUS_CELL_10_VOLTAGE are sequential and decreasing order.
	for (int i = 0 ; i < _cell_count; i++) {
		uint16_t result;
		ret |= _interface->read_word(BATT_SMBUS_CELL_1_VOLTAGE - i, result);
		// Convert millivolts to volts.
		cell_voltages[i] = ((float)result) * 0.001f;
	}

	//Calculate max cell delta
	float min_cell_voltage = cell_voltages[0];
	float max_cell_voltage = cell_voltages[0];

	for (uint8_t i = 1; (i < _cell_count && i < (sizeof(cell_voltages) / sizeof(cell_voltages[0]))); i++) {
		min_cell_voltage = math::min(min_cell_voltage, cell_voltages[i]);
		max_cell_voltage = math::max(max_cell_voltage, cell_voltages[i]);
	}

	// Calculate the max difference between the min and max cells with complementary filter.
	data.max_cell_voltage_delta = (0.5f * (max_cell_voltage - min_cell_voltage)) +
				  (0.5f * _last_max_cell_voltage_delta);

	_last_max_cell_voltage_delta = data.max_cell_voltage_delta;

	return ret;
}

int Batmon::populate_startup_data()
{
	int ret = BaseClass::populate_startup_data();

	// Read BatMon specific data
	uint16_t num_cells;
	ret |= _interface->read_word(BATT_SMBUS_CELL_COUNT, num_cells);
	if (ret == PX4_OK) {
		_cell_count = math::min((uint8_t)num_cells, MAX_CELL_COUNT);

		int32_t temp = num_cells;
		param_set(param_find("BAT_N_CELLS"), &temp);
	} else {
		_cell_count = 0;
	}

	return ret;
}

extern "C" __EXPORT int batmon_main(int argc, char *argv[])
{
	return Batmon::main(argc, argv);
}
