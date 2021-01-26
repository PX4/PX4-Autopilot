#include "rotoye_batmon.h"
#include <lib/parameters/param.h>


void Rotoye_Batmon::RunImpl()
{
	// Get the current time.
	uint64_t now = hrt_absolute_time();

	// Read data from sensor.
	battery_status_s new_report = {};

	new_report.id = 1;

	// Set time of reading.
	new_report.timestamp = now;

	new_report.connected = true;

	int ret = populate_smbus_data(new_report);

	ret |= get_cell_voltages();

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

int Rotoye_Batmon::get_cell_voltages()
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
