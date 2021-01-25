
#include "bq40zx.h"
#include <lib/parameters/param.h>


int BQ40ZX::get_startup_info()
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

	return result;
}

void BQ40ZX::RunImpl()
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

	// Read average current.
	ret |= _interface->read_word(BATT_SMBUS_AVERAGE_CURRENT, result);

	float average_current = (-1.0f * ((float)(*(int16_t *)&result)) / 1000.0f);

	new_report.average_current_a = average_current;
	// If current is high, turn under voltage protection off. This is neccessary to prevent
	// a battery from cutting off while flying with high current near the end of the packs capacity.
	set_undervoltage_protection(average_current);

	// Read run time to empty.
	ret |= _interface->read_word(BATT_SMBUS_RUN_TIME_TO_EMPTY, result);
	new_report.run_time_to_empty = result;

	// Read average time to empty.

	//HACKED:
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

	new_report.is_smart = true;

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
	// See bq40z50 technical reference.
	uint16_t keys[2] = {0x0414, 0x3672};

	int ret = _interface->write_word(BATT_SMBUS_MANUFACTURER_ACCESS, keys[0]);

	ret |= _interface->write_word(BATT_SMBUS_MANUFACTURER_ACCESS, keys[1]);

	return ret;
}

int BQ40ZX::seal()
{
	// See bq40z50 technical reference.
	uint16_t reg = BATT_SMBUS_SEAL;

	return manufacturer_write(reg, nullptr, 0);
}

int BQ40ZX::lifetime_data_flush()
{
	uint16_t flush = BATT_SMBUS_LIFETIME_FLUSH;

	return manufacturer_write(flush, nullptr, 0);
}

int BQ40ZX::lifetime_read_block_one()
{
	const int buffer_size = 32 + 2; // 32 bytes of data and 2 bytes of address
	uint8_t lifetime_block_one[buffer_size] = {};

	if (PX4_OK != manufacturer_read(BATT_SMBUS_LIFETIME_BLOCK_ONE, lifetime_block_one, buffer_size)) {
		PX4_INFO("Failed to read lifetime block 1.");
		return PX4_ERROR;
	}

	//Get max cell voltage delta and convert from mV to V.
	_lifetime_max_delta_cell_voltage = (float)(lifetime_block_one[17] << 8 | lifetime_block_one[16]) / 1000.0f;

	PX4_INFO("Max Cell Delta: %4.2f", (double)_lifetime_max_delta_cell_voltage);

	return PX4_OK;
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

void BQ40ZX::custom_method(const BusCLIArguments &cli)
{
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
				uint8_t *tx_buf = (uint8_t*)cli.custom_data;
				unsigned length = tx_buf[0];

				if (PX4_OK != dataflash_write(address, tx_buf+1, length)) {
					PX4_ERR("Dataflash write failed: %d", address);
				}
				px4_usleep(100000);
			}
			break;
	}
}
