/**
 * @file rotoye_batmon.cpp
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for Rotoye Batmon
 *
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 * @author Eohan George <eg@.skymul.com>
 */


#include "rotoye_batmon.h"
#include <lib/parameters/param.h>

#define BATT_SMBUS_TEMP_EXTERNAL			0x07

#define BATT_SMBUS_CELL_1_VOLTAGE                       0x3F
#define BATT_SMBUS_CELL_2_VOLTAGE                       0x3E
#define BATT_SMBUS_CELL_3_VOLTAGE                       0x3D
#define BATT_SMBUS_CELL_4_VOLTAGE                       0x3C
#define BATT_SMBUS_CELL_5_VOLTAGE                       0x3B
#define BATT_SMBUS_CELL_6_VOLTAGE                       0x3A
#define BATT_SMBUS_CELL_7_VOLTAGE                       0x39
#define BATT_SMBUS_CELL_8_VOLTAGE                       0x38
#define BATT_SMBUS_CELL_9_VOLTAGE                       0x37
#define BATT_SMBUS_CELL_10_VOLTAGE                      0x36

#define BATT_SMBUS_REMAINING_CAPACITY                   0x0F            ///< predicted remaining battery capacity as mAh
#define BATT_SMBUS_RELATIVE_SOC				0x0D		///< predicted remaining battery capacity as a percentage

#define BATT_SMBUS_CELL_COUNT                           0x40            // < This is not a default register in the BQ40Z50 chip, but one that is really needed
#define BATT_SMBUS_SAFETY_ALERT                         0x50            ///32 alert bits, threshold exceeded (used for burst current check)
#define BATT_SMBUS_SAFETY_STATUS                        0x51            ///32 status bits, threshold exceeded for certain duration
#define BATT_SMBUS_PF_ALERT                             0x52            ///32 permanent fail bits, issue warranting permanent shutoff occurred (used for cell voltage imbalance check)

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

	uint16_t temp_ext;
	ret |= _interface->read_word(BATT_SMBUS_TEMP_EXTERNAL, temp_ext);

	if (temp_ext != 0) { // Sends 0 when no external therm is used
		float temp_ext_c = ((float)temp_ext / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

		if (temp_ext_c > new_report.temperature) {
			new_report.temperature = temp_ext_c;
		}
	}

	new_report.max_cell_voltage_delta = _max_cell_voltage_delta;
	new_report.cell_count = _cell_count;

	for (uint8_t i = 0; i < _cell_count; i++) {
		new_report.voltage_cell_v[i] = _cell_voltages[i];
	}

	// TODO: Check warning flags (same as BQ driver)

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
	for (int i = 0 ; i < _cell_count; i++) {
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
