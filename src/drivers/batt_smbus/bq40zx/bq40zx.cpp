/**
 * @file bq40zx.cpp
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for BQ40Z50/80
 *
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 * @author Eohan George <eg@.skymul.com>
 */


#include "bq40zx.h"
#include <lib/parameters/param.h>

#define MAC_DATA_BUFFER_SIZE                            32

#define BATT_CELL_VOLTAGE_THRESHOLD_RTL                 0.5f            ///< Threshold in volts to RTL if cells are imbalanced
#define BATT_CELL_VOLTAGE_THRESHOLD_FAILED              1.5f            ///< Threshold in volts to Land if cells are imbalanced

#define BATT_CURRENT_UNDERVOLTAGE_THRESHOLD             5.0f            ///< Threshold in amps to disable undervoltage protection
#define BATT_VOLTAGE_UNDERVOLTAGE_THRESHOLD             3.4f            ///< Threshold in volts to re-enable undervoltage protection

#define BATT_SMBUS_BQ40Z50_CELL_4_VOLTAGE               0x3C
#define BATT_SMBUS_BQ40Z50_CELL_3_VOLTAGE               0x3D
#define BATT_SMBUS_BQ40Z50_CELL_2_VOLTAGE               0x3E
#define BATT_SMBUS_BQ40Z50_CELL_1_VOLTAGE               0x3F

#define BATT_SMBUS_BQ40Z80_CELL_7_VOLTAGE               0x3C
#define BATT_SMBUS_BQ40Z80_CELL_6_VOLTAGE               0x3D
#define BATT_SMBUS_BQ40Z80_CELL_5_VOLTAGE               0x3E
#define BATT_SMBUS_BQ40Z80_CELL_4_VOLTAGE               0x3F

#define BATT_SMBUS_STATE_OF_HEALTH                      0x4F            ///< State of Health. The SOH information of the battery in percentage of Design Capacity

#define BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS            0x44

#define BATT_SMBUS_SECURITY_KEYS                        0x0035
#define BATT_SMBUS_LIFETIME_FLUSH                       0x002E
#define BATT_SMBUS_LIFETIME_BLOCK_ONE                   0x0060
#define BATT_SMBUS_ENABLED_PROTECTIONS_A_ADDRESS        0x4938
#define BATT_SMBUS_SEAL                                 0x0030
#define BATT_SMBUS_DASTATUS1                            0x0071
#define BATT_SMBUS_DASTATUS2                            0x0072
#define BATT_SMBUS_DASTATUS3                            0x007B

#define BATT_SMBUS_ENABLED_PROTECTIONS_A_DEFAULT        0xcf
#define BATT_SMBUS_ENABLED_PROTECTIONS_A_CUV_DISABLED   0xce


int BQ40ZX::get_startup_info()
{
	int ret = PX4_OK;

	// Read battery threshold params on startup.
	param_get(param_find("BAT_CRIT_THR"), &_crit_thr);
	param_get(param_find("BAT_LOW_THR"), &_low_thr);
	param_get(param_find("BAT_EMERGEN_THR"), &_emergency_thr);
	param_get(param_find("BAT_C_MULT"), &_c_mult);

	int32_t cell_count_param = 0;
	param_get(param_find("BAT_N_CELLS"), &cell_count_param);
	_cell_count = (uint8_t)cell_count_param;

	ret |= _interface->block_read(BATT_SMBUS_MANUFACTURER_NAME, _manufacturer_name, BATT_SMBUS_MANUFACTURER_NAME_SIZE,
				      true);
	_manufacturer_name[sizeof(_manufacturer_name) - 1] = '\0';

	uint16_t serial_num;
	ret |= _interface->read_word(BATT_SMBUS_SERIAL_NUMBER, serial_num);

	uint16_t remaining_cap;
	ret |= _interface->read_word(BATT_SMBUS_REMAINING_CAPACITY, remaining_cap);

	uint16_t cycle_count;
	ret |= _interface->read_word(BATT_SMBUS_CYCLE_COUNT, cycle_count);

	uint16_t full_cap;
	ret |= _interface->read_word(BATT_SMBUS_FULL_CHARGE_CAPACITY, full_cap);

	uint16_t manufacture_date;
	ret |= _interface->read_word(BATT_SMBUS_MANUFACTURE_DATE, manufacture_date);

	uint16_t state_of_health;
	ret |= _interface->read_word(BATT_SMBUS_STATE_OF_HEALTH, state_of_health);

	if (!ret) {
		_serial_number = serial_num;
		_batt_startup_capacity = (uint16_t)((float)remaining_cap * _c_mult);
		_cycle_count = cycle_count;
		_batt_capacity = (uint16_t)((float)full_cap * _c_mult);
		_manufacture_date = manufacture_date;
		_state_of_health = state_of_health;
	}

	if (lifetime_data_flush() == PX4_OK) {
		// Flush needs time to complete, otherwise device is busy. 100ms not enough, 200ms works.
		px4_usleep(200000);

		if (lifetime_read_block_one() == PX4_OK) {
			if (_lifetime_max_delta_cell_voltage > BATT_CELL_VOLTAGE_THRESHOLD_FAILED) {
				PX4_WARN("Battery Damaged Will Not Fly. Lifetime max voltage difference: %4.2f",
					 (double)_lifetime_max_delta_cell_voltage);
			}
		}

	} else {
		PX4_WARN("Failed to flush lifetime data");
	}

	return ret;
}

void BQ40ZX::RunImpl()
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

	// Temporary variable for storing SMBUS reads.
	uint16_t result;

	// Read average current.
	ret |= _interface->read_word(BATT_SMBUS_AVERAGE_CURRENT, result);

	float average_current = (-1.0f * ((float)(*(int16_t *)&result)) * 0.001f);

	new_report.average_current_a = average_current;
	// If current is high, turn under voltage protection off. This is neccessary to prevent
	// a battery from cutting off while flying with high current near the end of the packs capacity.
	set_undervoltage_protection(average_current);

	// Read run time to empty.
	ret |= _interface->read_word(BATT_SMBUS_RUN_TIME_TO_EMPTY, result);
	new_report.run_time_to_empty = result;

	// Read average time to empty.

	ret |= _interface->read_word(BATT_SMBUS_AVERAGE_TIME_TO_EMPTY, result);
	new_report.average_time_to_empty = result;

	// Check if max lifetime voltage delta is greater than allowed.
	if (_lifetime_max_delta_cell_voltage > BATT_CELL_VOLTAGE_THRESHOLD_FAILED) {
		new_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;
	}

	// Propagate warning state.
	else {
		if (new_report.remaining > _low_thr) {
			new_report.warning = battery_status_s::BATTERY_WARNING_NONE;

		} else if (new_report.remaining > _crit_thr) {
			new_report.warning = battery_status_s::BATTERY_WARNING_LOW;

		} else if (new_report.remaining > _emergency_thr) {
			new_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

		} else {
			new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
		}
	}

	// Read battery temperature and covert to Celsius.
	ret |= _interface->read_word(BATT_SMBUS_TEMP, result);
	new_report.temperature = ((float)result * 0.1f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

	new_report.capacity = _batt_capacity;
	new_report.cycle_count = _cycle_count;
	new_report.serial_number = _serial_number;
	new_report.max_cell_voltage_delta = _max_cell_voltage_delta;
	new_report.cell_count = _cell_count;

	for (uint8_t i = 0; i < _cell_count; i++) {
		new_report.voltage_cell_v[i] = _cell_voltages[i];
	}

	// Only publish if no errors.
	if (!ret) {
		orb_publish(ORB_ID(battery_status), _batt_topic, &new_report);

		_last_report = new_report;
	}
}

//@NOTE: Currently unused, could be helpful for debugging a parameter set though.
int BQ40ZX::dataflash_read(uint16_t &address, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	int result = _interface->block_write(code, &address, 2, true);

	if (result != PX4_OK) {
		return result;
	}

	result = _interface->block_read(code, data, length, true);

	return result;
}

int BQ40ZX::dataflash_write(uint16_t address, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	uint8_t tx_buf[MAC_DATA_BUFFER_SIZE + 2] = {};

	tx_buf[0] = ((uint8_t *)&address)[0];
	tx_buf[1] = ((uint8_t *)&address)[1];

	if (length > MAC_DATA_BUFFER_SIZE) {
		return PX4_ERROR;
	}

	memcpy(&tx_buf[2], data, length);

	// code (1), byte_count (1), addr(2), data(32) + pec
	int result = _interface->block_write(code, tx_buf, length + 2, false);

	return result;
}

int BQ40ZX::manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	uint8_t address[2] = {};
	address[0] = ((uint8_t *)&cmd_code)[0];
	address[1] = ((uint8_t *)&cmd_code)[1];

	int result = _interface->block_write(code, address, 2, false);

	if (result != PX4_OK) {
		return result;
	}

	result = _interface->block_read(code, data, length, true);
	memmove(data, &((uint8_t *)data)[2], length - 2); // remove the address bytes

	return result;
}

int BQ40ZX::manufacturer_write(const uint16_t cmd_code, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	uint8_t address[2] = {};
	address[0] = ((uint8_t *)&cmd_code)[0];
	address[1] = ((uint8_t *)&cmd_code)[1];

	uint8_t tx_buf[MAC_DATA_BUFFER_SIZE + 2] = {};
	memcpy(tx_buf, address, 2);

	if (data != nullptr) {
		memcpy(&tx_buf[2], data, length);
	}

	int result = _interface->block_write(code, tx_buf, length + 2, false);

	return result;
}

int BQ40ZX::unseal()
{
	// See BQ40ZX technical reference.
	uint16_t keys[2] = {0x0414, 0x3672};

	int ret = _interface->write_word(BATT_SMBUS_MANUFACTURER_ACCESS, keys[0]);

	ret |= _interface->write_word(BATT_SMBUS_MANUFACTURER_ACCESS, keys[1]);

	return ret;
}

int BQ40ZX::seal()
{
	// See BQ40ZX technical reference.
	uint16_t reg = BATT_SMBUS_SEAL;

	return manufacturer_write(reg, nullptr, 0);
}

int BQ40ZX::lifetime_data_flush()
{
	return manufacturer_write(BATT_SMBUS_LIFETIME_FLUSH, nullptr, 0);
}

void BQ40ZX::set_undervoltage_protection(float average_current)
{
	// Disable undervoltage protection if armed. Enable if disarmed and cell voltage is above limit.
	if (average_current > BATT_CURRENT_UNDERVOLTAGE_THRESHOLD) {
		if (_cell_undervoltage_protection_status != 0) {
			// Disable undervoltage protection
			uint8_t protections_a_tmp = BATT_SMBUS_ENABLED_PROTECTIONS_A_CUV_DISABLED;
			uint16_t address = BATT_SMBUS_ENABLED_PROTECTIONS_A_ADDRESS;

			if (dataflash_write(address, &protections_a_tmp, 1) == PX4_OK) {
				_cell_undervoltage_protection_status = 0;
				PX4_WARN("Disabled CUV");

			} else {
				PX4_WARN("Failed to disable CUV");
			}
		}

	} else {
		if (_cell_undervoltage_protection_status == 0) {
			if (_min_cell_voltage > BATT_VOLTAGE_UNDERVOLTAGE_THRESHOLD) {
				// Enable undervoltage protection
				uint8_t protections_a_tmp = BATT_SMBUS_ENABLED_PROTECTIONS_A_DEFAULT;
				uint16_t address = BATT_SMBUS_ENABLED_PROTECTIONS_A_ADDRESS;

				if (dataflash_write(address, &protections_a_tmp, 1) == PX4_OK) {
					_cell_undervoltage_protection_status = 1;
					PX4_WARN("Enabled CUV");

				} else {
					PX4_WARN("Failed to enable CUV");
				}
			}
		}
	}

}

int BQ40ZX::get_cell_voltages()
{
	// Temporary variable for storing SMBUS reads.
	uint16_t result = 0;
	int ret = PX4_OK;

	if (_device_type == SMBUS_DEVICE_TYPE::BQ40Z50) {
		ret |= _interface->read_word(BATT_SMBUS_BQ40Z50_CELL_1_VOLTAGE, result);
		// Convert millivolts to volts.
		_cell_voltages[0] = ((float)result) * 0.001f;

		ret |= _interface->read_word(BATT_SMBUS_BQ40Z50_CELL_2_VOLTAGE, result);
		// Convert millivolts to volts.
		_cell_voltages[1] = ((float)result) * 0.001f;

		ret |= _interface->read_word(BATT_SMBUS_BQ40Z50_CELL_3_VOLTAGE, result);
		// Convert millivolts to volts.
		_cell_voltages[2] = ((float)result) * 0.001f;

		ret |= _interface->read_word(BATT_SMBUS_BQ40Z50_CELL_4_VOLTAGE, result);
		// Convert millivolts to volts.
		_cell_voltages[3] = ((float)result) * 0.001f;
		_cell_voltages[4] = 0;
		_cell_voltages[5] = 0;
		_cell_voltages[6] = 0;

	} else if (_device_type == SMBUS_DEVICE_TYPE::BQ40Z80) {
		uint8_t DAstatus1[32 + 2] = {}; // 32 bytes of data and 2 bytes of address

		//TODO: need to consider if set voltages to 0? -1?
		if (PX4_OK != manufacturer_read(BATT_SMBUS_DASTATUS1, DAstatus1, sizeof(DAstatus1))) {
			return PX4_ERROR;
		}

		// Convert millivolts to volts.
		_cell_voltages[0] = ((float)((DAstatus1[1] << 8) | DAstatus1[0]) * 0.001f);
		_cell_voltages[1] = ((float)((DAstatus1[3] << 8) | DAstatus1[2]) * 0.001f);
		_cell_voltages[2] = ((float)((DAstatus1[5] << 8) | DAstatus1[4]) * 0.001f);
		_cell_voltages[3] = ((float)((DAstatus1[7] << 8) | DAstatus1[6]) * 0.001f);

		_pack_power = ((float)((DAstatus1[29] << 8) | DAstatus1[28]) * 0.01f); //TODO: decide if both needed
		_pack_average_power = ((float)((DAstatus1[31] << 8) | DAstatus1[30]) * 0.01f);

		uint8_t DAstatus3[18 + 2] = {}; // 18 bytes of data and 2 bytes of address

		//TODO: need to consider if set voltages to 0? -1?
		if (PX4_OK != manufacturer_read(BATT_SMBUS_DASTATUS3, DAstatus3, sizeof(DAstatus3))) {
			return PX4_ERROR;
		}

		_cell_voltages[4] = ((float)((DAstatus3[1] << 8) | DAstatus3[0]) * 0.001f);
		_cell_voltages[5] = ((float)((DAstatus3[7] << 8) | DAstatus3[6]) * 0.001f);
		_cell_voltages[6] = ((float)((DAstatus3[13] << 8) | DAstatus3[12]) * 0.001f);

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

int BQ40ZX::lifetime_read_block_one()
{
	uint8_t lifetime_block_one[32 + 2] = {}; // 32 bytes of data and 2 bytes of address

	if (PX4_OK != manufacturer_read(BATT_SMBUS_LIFETIME_BLOCK_ONE, lifetime_block_one, sizeof(lifetime_block_one))) {
		PX4_INFO("Failed to read lifetime block 1.");
		return PX4_ERROR;
	}

	//Get max cell voltage delta and convert from mV to V.
	if (_device_type == SMBUS_DEVICE_TYPE::BQ40Z50) {

		_lifetime_max_delta_cell_voltage = (float)(lifetime_block_one[17] << 8 | lifetime_block_one[16]) * 0.001f;

	} else if (_device_type == SMBUS_DEVICE_TYPE::BQ40Z80) {

		_lifetime_max_delta_cell_voltage = (float)(lifetime_block_one[29] << 8 | lifetime_block_one[28]) * 0.001f;
	}

	PX4_INFO("Max Cell Delta: %4.2f", (double)_lifetime_max_delta_cell_voltage);

	return PX4_OK;
}

void BQ40ZX::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom1) {
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
		unseal();
		break;

	case 3:
		seal();
		break;

	case 4:
		suspend();
		break;

	case 5:
		resume();
		break;

	case 6:
		if (cli.custom_data) {
			unsigned address = cli.custom2;
			uint8_t *tx_buf = (uint8_t *)cli.custom_data;
			unsigned length = tx_buf[0];

			if (PX4_OK != dataflash_write(address, tx_buf + 1, length)) {
				PX4_ERR("Dataflash write failed: %d", address);
			}

			px4_usleep(100000);
		}

		break;
	}
}
