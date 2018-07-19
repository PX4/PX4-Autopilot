/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file batt_smbus.cpp
 *
 * Driver for a battery monitor connected via SMBus (I2C).
 * Designed for BQ40Z50-R1/R2
 *
 * @author Randy Mackay <rmackay9@yahoo.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Mark Sauder <mcsauder@gmail.com>
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 */

#include <px4_defines.h>

#include "batt_smbus.h"

BATT_SMBUS::BATT_SMBUS(device::Device *interface, const char *path) :
	CDev("BATT_SMBUS", path),
	_interface(interface),
	_batt_topic(nullptr),
	_batt_orb_id(nullptr),
	_batt_capacity(0),
	_batt_startup_capacity(0),
	_cycle_count(0),
	_serial_number(0),
	_crit_thr(0.0f),
	_emergency_thr(0.0f),
	_low_thr(0.0f),
	_manufacturer_name(nullptr)
{
	// Set the device type from the interface.
	_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();
}

BATT_SMBUS::~BATT_SMBUS()
{
	// Ensure we are truly inactive.
	stop();

	if (_manufacturer_name != nullptr) {
		delete[] _manufacturer_name;
	}

	PX4_WARN("Smart battery driver stopped");

	int battsource = 0;
	param_set(param_find("BAT_SOURCE"), &battsource);
}

int BATT_SMBUS::block_read(const uint8_t cmd_code, void *data, const unsigned length)
{
	unsigned byte_count = 0;
	// Length of data (32max). byte_count(1), cmd_code(2), pec(1)
	uint8_t rx_data[DATA_BUFFER_SIZE + 4];

	// If this is a ManufacturerBlockAccess() command then the first 2 data bytes will be the cmd_code.
	if (cmd_code == BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS) {
		_interface->read(cmd_code, rx_data, length + 4);

		byte_count = rx_data[0];

		// addr1, addr2, byte_count,
		memcpy(data, &rx_data[3], byte_count);

	} else {
		// byte_count(1) + data[32max] + pec(1)
		_interface->read(cmd_code, rx_data, length + 2);

		byte_count = rx_data[0];

		memcpy(data, &rx_data[1], byte_count);
	}

	// addr(wr), cmd_code, addr(r), byte_count, rx_data[]
	uint8_t device_address = get_device_address();
	uint8_t full_data_packet[DATA_BUFFER_SIZE + 4] = {0};

	full_data_packet[0] = (device_address << 1) | 0x00;
	full_data_packet[1] = cmd_code;
	full_data_packet[2] = (device_address << 1) | 0x01;
	full_data_packet[3] = byte_count;

	memcpy(&full_data_packet[4], &rx_data[1], byte_count);

	uint8_t pec = get_pec(full_data_packet, byte_count + 4);

	// First byte is byte count, followed by data.
	if (pec != ((uint8_t *)rx_data)[byte_count + 1]) {
		PX4_INFO("bad PEC from block_read");
		return PX4_ERROR;

	} else {
		return PX4_OK;
	}
}

int BATT_SMBUS::block_write(const uint8_t cmd_code, void *data, const unsigned byte_count)
{
	// cmd code, byte count, data[byte_count], pec
	uint8_t buf[byte_count + 2];

	buf[0] = cmd_code;
	buf[1] = (uint8_t)byte_count;
	memcpy(&buf[2], data, byte_count);

	uint8_t pec = get_pec(buf, sizeof(buf));
	buf[byte_count + 2] = pec;

	unsigned i = 0;

	// If block_write fails, try up to 10 times.
	while (i < 10) {
		if (PX4_OK != _interface->write(0, buf, sizeof(buf))) {
			i++;
			PX4_WARN("block_write failed: %d", i);
			usleep(100000);

		} else {
			return PX4_OK;
		}
	}

	return PX4_ERROR;
}

void BATT_SMBUS::cycle()
{
	// Get the current time.
	uint64_t now = hrt_absolute_time();

	// Read data from sensor.
	battery_status_s new_report = {};

	// Set time of reading.
	new_report.timestamp = now;

	// Don't publish if any reads fail.
	bool success = true;

	// Temporary variable for storing SMBUS reads.
	uint16_t tmp;

	if (read_word(BATT_SMBUS_VOLTAGE, &tmp) == PX4_OK) {

		new_report.connected = true;

		// Convert millivolts to volts.
		new_report.voltage_v = ((float)tmp) / 1000.0f;
		new_report.voltage_filtered_v = new_report.voltage_v;

		// Read current.
		if (read_word(BATT_SMBUS_CURRENT, &tmp) == PX4_OK) {
			new_report.current_a = (-1.0f * ((float)(*(int16_t *)&tmp)) / 1000.0f);
			new_report.current_filtered_a = new_report.current_a;

		} else {
			success = false;
		}

		// Read average current.
		if (read_word(BATT_SMBUS_AVERAGE_CURRENT, &tmp) == PX4_OK) {
			new_report.average_current_a = (-1.0f * ((float)(*(int16_t *)&tmp)) / 1000.0f);

		} else {
			success = false;
		}

		// Read run time to empty.
		if (read_word(BATT_SMBUS_RUN_TIME_TO_EMPTY, &tmp) == PX4_OK) {
			new_report.run_time_to_empty = tmp;

		} else {
			success = false;
		}

		// Read average time to empty.
		if (read_word(BATT_SMBUS_AVERAGE_TIME_TO_EMPTY, &tmp) == PX4_OK) {
			new_report.average_time_to_empty = tmp;

		} else {
			success = false;
		}

		// Read remaining capacity.
		if (read_word(BATT_SMBUS_REMAINING_CAPACITY, &tmp) == PX4_OK) {

			if (tmp > _batt_capacity) {
				PX4_WARN("Remaining capacity greater than total: Capacity:%hu \tRemaining Capacity:%hu",
					 (uint16_t)_batt_capacity, (uint16_t)tmp);
				_batt_capacity = (uint16_t)tmp;
			}

			// Calculate remaining capacity percent with complementary filter
			new_report.remaining = ((float)_last_report.remaining * 0.8f) + (0.2f * (1.0f -
					       (((float)_batt_capacity - (float)tmp) / (float)_batt_capacity)));

			// Calculate total discharged amount.
			new_report.discharged_mah = (float)_batt_startup_capacity - (float)tmp;

			// Check if remaining % is out of range.
			if ((new_report.remaining > 1.00f) || (new_report.remaining <= 0.00f)) {
				new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
				PX4_INFO("Percent out of range: %4.2f", (double)new_report.remaining);
			}

			// Check if discharged amount is greater than the starting capacity.
			else if (new_report.discharged_mah > (float)_batt_startup_capacity) {
				new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
				PX4_INFO("Discharged greater than startup capacity: %4.2f", (double)new_report.discharged_mah);
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
					uint64_t timer = hrt_absolute_time() - now;
					new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

					/* Only warn every 5 seconds */
					if (timer > 5000000) {
						PX4_WARN("Battery Warning Emergency: %4.2f", (double)new_report.remaining);
					}
				}
			}

		} else {
			success = false;
		}

		// Read battery temperature and covert to Celsius.
		if (read_word(BATT_SMBUS_TEMP, &tmp) == PX4_OK) {
			new_report.temperature = (float)(((float)tmp / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS);

		} else {
			success = false;
		}

		new_report.capacity = _batt_capacity;
		new_report.cycle_count = _cycle_count;
		new_report.serial_number = _serial_number;

		// Publish to orb.
		if (_batt_topic != nullptr && success) {

			orb_publish(_batt_orb_id, _batt_topic, &new_report);

			// Copy report for test().
			_last_report = new_report;


		} else {
			_batt_topic = orb_advertise(_batt_orb_id, &new_report);

			if (_batt_topic == nullptr) {
				PX4_ERR("ADVERT FAIL");
				return;
			}
		}

	}

	// Schedule a fresh cycle call when the measurement is done.
	work_queue(HPWORK, &_work, (worker_t)&BATT_SMBUS::cycle_trampoline, this,
		   USEC2TICK(BATT_SMBUS_MEASUREMENT_INTERVAL_US));
}

void BATT_SMBUS::cycle_trampoline(void *arg)
{
	BATT_SMBUS *dev = (BATT_SMBUS *)arg;
	dev->cycle();
}

int BATT_SMBUS::dataflash_read(uint16_t &address, void *data)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	// address is 2 bytes
	block_write(code, &address, 2);
	// @NOTE: The data buffer MUST be 32 bytes.
	block_read(code, data, DATA_BUFFER_SIZE);

	// for debug only: print out the receieved buffer
	for (unsigned i = 0; i < DATA_BUFFER_SIZE ; i++) {
		PX4_INFO("%d", ((uint8_t *)data)[i]);
	}

	return PX4_OK;
}


int BATT_SMBUS::dataflash_write(uint16_t &address, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	// @NOTE: The data buffer can be 1 - 32 bytes. The address field is 2 bytes.
	uint8_t tx_buf[DATA_BUFFER_SIZE + 2] = {0};

	tx_buf[0] = ((uint8_t *)&address)[0];
	tx_buf[1] = ((uint8_t *)&address)[1];
	memcpy(&tx_buf[2], data, length);

	// code (1), byte_count (1), addr(2), data(32) + pec
	block_write(code, tx_buf, length + 2);

	return PX4_OK;
}

uint8_t BATT_SMBUS::get_pec(uint8_t *buff, const uint8_t len)
{
	// Initialise CRC to zero.
	uint8_t crc = 0;
	uint8_t shift_register = 0;
	bool invert_crc;

	// Calculate crc for each byte in the stream.
	for (uint8_t i = 0; i < len; i++) {
		// Load next data byte into the shift register
		shift_register = buff[i];

		// Calculate crc for each bit in the current byte.
		for (uint8_t j = 0; j < 8; j++) {
			invert_crc = (crc ^ shift_register) & 0x80;
			crc <<= 1;
			shift_register <<= 1;

			if (invert_crc) {
				crc ^= BATT_SMBUS_PEC_POLYNOMIAL;
			}
		}
	}

	return crc;
}

int BATT_SMBUS::get_startup_info()
{
	int result = PX4_ERROR;
	const unsigned name_length = 22;

	// Try and get battery SBS info.
	if (_manufacturer_name == nullptr) {
		char man_name[name_length] = {0};
		result = manufacturer_name((uint8_t *)man_name, sizeof(man_name));

		if (PX4_OK != result) {
			PX4_WARN("Failed to get manufacturer name");
			return PX4_ERROR;
		}

		_manufacturer_name = new char[sizeof(man_name)];
		strcpy(_manufacturer_name, man_name);
	}

	// Temporary variable for storing SMBUS reads.
	uint16_t tmp = 0;

	// Read battery serial number on startup.
	if (read_word(BATT_SMBUS_SERIAL_NUMBER, &tmp) == PX4_OK) {
		_serial_number = tmp;
		result = PX4_OK;
	}

	// Read battery capacity on startup.
	if (read_word(BATT_SMBUS_REMAINING_CAPACITY, &tmp) == PX4_OK) {
		_batt_startup_capacity = tmp;
		result = PX4_OK;
	}

	// Read battery cycle count on startup.
	if (read_word(BATT_SMBUS_CYCLE_COUNT, &tmp) == PX4_OK) {
		_cycle_count = tmp;
		result = PX4_OK;
	}

	// Read battery design capacity on startup.
	if (read_word(BATT_SMBUS_FULL_CHARGE_CAPACITY, &tmp) == PX4_OK) {
		_batt_capacity = tmp;
		result = PX4_OK;
	}

	// Read battery threshold params on startup.
	param_get(param_find("BAT_CRIT_THR"), &_crit_thr);
	param_get(param_find("BAT_LOW_THR"), &_low_thr);
	param_get(param_find("BAT_EMERGEN_THR"), &_emergency_thr);

	return result;
}

uint16_t BATT_SMBUS::get_serial_number()
{
	uint16_t serial_num = 0;

	if (read_word(BATT_SMBUS_SERIAL_NUMBER, &serial_num) == PX4_OK) {
		return serial_num;
	}

	return PX4_ERROR;
}

int BATT_SMBUS::info()
{
	print_message(_last_report);
	return PX4_OK;
}

int BATT_SMBUS::init()
{
	if (PX4_OK != CDev::init()) {
		PX4_ERR("CDev init failed");
		return PX4_ERROR;
	}

	// Find the battery on the bus and read startup info.
	if (search_addresses() != PX4_OK) {
		PX4_ERR("Failed to init I2C");
		return PX4_ERROR;
	}

	// Retry up to 10 times to read startup info.
	for (unsigned i = 0; i < 10; i++) {
		if (PX4_OK == get_startup_info()) {
			break;
		}

		if (i == 9) {
			PX4_ERR("Failed to get battery startup info");
			return 	PX4_ERROR;
		}
	}

	// Initialize the orb ID.
	_batt_orb_id = ORB_ID(battery_status);

	// Start the work queue.
	start();

	return PX4_OK;
}

int BATT_SMBUS::manufacture_date(void *man_date)
{
	uint8_t code = BATT_SMBUS_MANUFACTURE_DATE;

	int result = read_word(code, man_date);

	if (PX4_OK != result) {
		PX4_WARN("Could not read manufacturer name");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int BATT_SMBUS::manufacturer_name(uint8_t *man_name, const uint8_t length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_NAME;
	uint8_t rx_buf[21] = {0};

	// Returns 21 bytes, add 1 byte for null terminator.
	int result = block_read(code, rx_buf, length - 1);

	memcpy(man_name, rx_buf, sizeof(rx_buf));

	man_name[21] = '\0';

	if (PX4_OK != result) {
		PX4_WARN("Could not read manufacturer name");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int BATT_SMBUS::manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	uint8_t address[2] = {0};
	address[0] = ((uint8_t *)&cmd_code)[0];
	address[1] = ((uint8_t *)&cmd_code)[1];

	block_write(code, address, sizeof(address));

	// returns the 2 bytes of addr info + data[]
	block_read(code, data, length);

	uint8_t rx_buf[DATA_BUFFER_SIZE] = {0};
	memcpy(rx_buf, data, DATA_BUFFER_SIZE);

	PX4_INFO("Received data: %d %d %d %d %d %d %d %d %d %d %d %d", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4],
		 rx_buf[5], rx_buf[6], rx_buf[7], rx_buf[8], rx_buf[9], rx_buf[10], rx_buf[11]);

	return PX4_OK;
}

int BATT_SMBUS::manufacturer_write(const uint16_t cmd_code, void *data, const unsigned length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS;

	uint8_t address[2] = {0};
	address[0] = ((uint8_t *)&cmd_code)[0];
	address[1] = ((uint8_t *)&cmd_code)[1];

	uint8_t tx_buf[DATA_BUFFER_SIZE + 2] = {0};
	memcpy(tx_buf, address, 2);
	memcpy(&tx_buf[2], data, length);

	block_write(code, tx_buf, length + 2);

	return PX4_OK;
}

int BATT_SMBUS::read_word(const uint8_t cmd_code, void *data)
{
	// 2 data bytes + pec byte
	int result = _interface->read(cmd_code, data, 3);

	if (PX4_OK == result) {
		// Check PEC.
		uint8_t addr = (get_device_address() << 1);
		uint8_t full_data_packet[5];
		full_data_packet[0] = addr | 0x00;
		full_data_packet[1] = cmd_code;
		full_data_packet[2] = addr | 0x01;
		memcpy(&full_data_packet[3], data, 2);

		uint8_t pec = get_pec(full_data_packet, sizeof(full_data_packet));

		if (pec == ((uint8_t *)data)[2]) {
			return PX4_OK;

		} else {
			return PX4_ERROR;
		}
	}

	return PX4_ERROR;
}

int BATT_SMBUS::search_addresses()
{
	uint16_t tmp;

	set_device_address(BATT_SMBUS_ADDR);

	if (PX4_OK != read_word(BATT_SMBUS_VOLTAGE, &tmp)) {
		// Search through all valid SMBus addresses.
		for (uint8_t i = BATT_SMBUS_ADDR_MIN; i < BATT_SMBUS_ADDR_MAX; i++) {
			set_device_address(i);

			if (read_word(BATT_SMBUS_VOLTAGE, &tmp) == PX4_OK) {
				if (tmp > 0) {
					break;
				}
			}

			if (i == BATT_SMBUS_ADDR_MAX - 1) {
				PX4_WARN("No smart batteries found.");
				return PX4_ERROR;
			}
		}
	}

	PX4_INFO("Smart battery found at 0x%x", get_device_address());
	PX4_INFO("Smart battery connected");

	return PX4_OK;
}

void BATT_SMBUS::start()
{
	// Schedule a cycle to start things.
	work_queue(HPWORK, &_work, (worker_t)&BATT_SMBUS::cycle_trampoline, this, 1);
}

void BATT_SMBUS::stop()
{
	// Cancel the work queue.
	work_cancel(HPWORK, &_work);
}

int BATT_SMBUS::unseal()
{
	// See pg85 of bq40z50 technical reference.
	uint16_t keys[2] = {0x0414, 0x3672};

	if (PX4_OK != manufacturer_write(keys[0], &keys[1], 2)) {
		PX4_INFO("Failed to unseal device.");
		return PX4_ERROR;
	}

	return PX4_OK;
}