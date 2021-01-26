#include "rotoye_batmon.h"
#include <lib/parameters/param.h>


int Rotoye_Batmon::get_startup_info()
{
	int result = 0;

	// The name field is 21 characters, add one for null terminator.
	const unsigned name_length = 22;

	// Read battery threshold params on startup.
	param_get(param_find("BAT_CRIT_THR"), &_crit_thr);
	param_get(param_find("BAT_LOW_THR"), &_low_thr);
	param_get(param_find("BAT_EMERGEN_THR"), &_emergency_thr);
	param_get(param_find("BAT_C_MULT"), &_c_mult);

	// Try and get battery SBS info.
	if (_manufacturer_name == nullptr) {
		char man_name[name_length] = {};
		result = manufacturer_name((uint8_t *)man_name, sizeof(man_name));

		if (result != PX4_OK) {
			PX4_DEBUG("Failed to get manufacturer name");
			return PX4_ERROR;
		}

		_manufacturer_name = new char[sizeof(man_name)];

		PX4_INFO("The manufacturer name: %s", man_name);
	}

	uint16_t serial_num;
	result = _interface->read_word(BATT_SMBUS_SERIAL_NUMBER, serial_num);

	uint16_t remaining_cap;
	result |= _interface->read_word(BATT_SMBUS_REMAINING_CAPACITY, remaining_cap);

	uint16_t cycle_count;
	result |= _interface->read_word(BATT_SMBUS_CYCLE_COUNT, cycle_count);

	uint16_t full_cap;
	result |= _interface->read_word(BATT_SMBUS_FULL_CHARGE_CAPACITY, full_cap);

	uint16_t cell_count;
	result |= _interface->read_word(BATT_SMBUS_CELL_COUNT, cell_count);

	if (!result) {
		_serial_number = serial_num;
		_batt_startup_capacity = (uint16_t)((float)remaining_cap * _c_mult);
		_cycle_count = cycle_count;
		_batt_capacity = full_cap;
		_cell_count = cell_count;
	}

	return result;
}

void Rotoye_Batmon::RunImpl()
{
	// Get the current time.
	uint64_t now = hrt_absolute_time();

	// Read data from sensor.
	battery_status_s new_report = {};

	// TODO(hyonlim): this driver should support multiple SMBUS going forward.
	new_report.id = 1;

	// Set time of reading.
	new_report.timestamp = now;

	new_report.connected = true;

	// Temporary variable for storing SMBUS reads.
	uint16_t result;

	int ret = _interface->read_word(BATT_SMBUS_VOLTAGE, result);

	ret |= get_cell_voltages();

	// Convert millivolts to volts.
	new_report.voltage_v = ((float)result) / 1000.0f;
	new_report.voltage_filtered_v = new_report.voltage_v;

	// Read current.
	ret |= _interface->read_word(BATT_SMBUS_CURRENT, result);

	new_report.current_a = (-1.0f * ((float)(*(int16_t *)&result)) / 1000.0f) * _c_mult;
	new_report.current_filtered_a = new_report.current_a;

	new_report.is_smart = true;
	// Read remaining capacity.
	ret |= _interface->read_word(BATT_SMBUS_RELATIVE_SOC, result);
	new_report.remaining = (float)result/100;

	// Read remaining capacity.
	ret |= _interface->read_word(BATT_SMBUS_REMAINING_CAPACITY, result);
	new_report.discharged_mah = _batt_startup_capacity - result;

	// Read battery temperature and covert to Celsius.
	ret |= _interface->read_word(BATT_SMBUS_TEMP, result);
	new_report.temperature = ((float)result / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

	new_report.capacity = _batt_capacity;
	new_report.cycle_count = _cycle_count;
	new_report.serial_number = _serial_number;
	new_report.max_cell_voltage_delta = _max_cell_voltage_delta;
	new_report.cell_count = _cell_count;
	for (uint8_t i = 0; i< _cell_count; i++)
	{
		new_report.voltage_cell_v[i] = _cell_voltages[i];
	}

	// Only publish if no errors.
	if (!ret) {
		orb_publish(ORB_ID(battery_status), _batt_topic, &new_report);

		_last_report = new_report;
	}
}

int BATT_SMBUS::get_cell_voltages()
{
	// Temporary variable for storing SMBUS reads.
	uint16_t result = 0;
	uint8_t ret = 0;

	// Making the assumption that the register value of BATT_SMBUS_CELL_1_VOLTAGE and BATT_SMBUS_CELL_10_VOLTAGE are sequential and decreasing order.
	for (int i = 0 ; i< _cell_count;i++)
	{
		ret |= _interface->read_word(BATT_SMBUS_CELL_1_VOLTAGE - i, result);
		// Convert millivolts to volts.
		_cell_voltages[i] = ((float)result) / 1000.0f;
	}

	//Calculate max cell delta
	_min_cell_voltage = _cell_voltages[0];
	float max_cell_voltage = _cell_voltages[0];

	for (uint8_t i = 1; (i < _cell_count && i < (sizeof(_cell_voltages) / sizeof(_cell_voltages[0]))); i++) {
		_min_cell_voltage = math::min(_min_cell_voltage, _cell_voltages[i]);
		max_cell_voltage = math::max(max_cell_voltage, _cell_voltages[i]);
	}

	// Calculate the max difference between the min and max cells with complementary filter.
	_max_cell_voltage_delta = (0.5f * (max_cell_voltage - _min_cell_voltage)) +
					(0.5f * _last_report.max_cell_voltage_delta);

	return ret;
}

void Rotoye_Batmon::custom_method(const BusCLIArguments &cli)
{

	// TODO? Implement seal, unseal, dataflash functions as warning wrappers
	//	 Such that this can be left in the parent class
	switch(cli.custom1) {
		case 1: {
			uint8_t man_name[22];
			int result = manufacturer_name(man_name, sizeof(man_name));
			PX4_INFO("The manufacturer name: %s", man_name);

			result = manufacture_date();
			PX4_INFO("The manufacturer date: %d", result);

			uint16_t serial_num = 0;
			serial_num = get_serial_number();
			PX4_INFO("The serial number: %d", serial_num);
		}
			break;
		case 2:
			PX4_WARN("Cannot unseal this device");
			break;
		case 3:
			PX4_WARN("Cannot unseal this device");
			break;
		case 4:
			suspend();
			break;
		case 5:
			resume();
			break;
		case 6:
			PX4_WARN("Cannot flash this device");
			break;
	}
}
