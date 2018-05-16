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
 */

#include <px4_defines.h>

#include "batt_smbus.h"

/** @brief device handle. For now, we only support one BATT_SMBUS device. */
BATT_SMBUS *g_batt_smbus;

extern "C" __EXPORT int batt_smbus_main(int argc, char *argv[]);

BATT_SMBUS::BATT_SMBUS(int bus, uint16_t batt_smbus_addr) :
	I2C("batt_smbus", "/dev/batt_smbus0", bus, batt_smbus_addr, 100000),
	_enabled(false),
	_batt_topic(nullptr),
	_batt_orb_id(nullptr),
	_batt_capacity(0),
	_batt_startup_capacity(0),
	_cycle_count(0),
	_serial_number(0),
	_start_time(0),
	_crit_thr(0.0f),
	_emergency_thr(0.0f),
	_low_thr(0.0f),
	_manufacturer_name(nullptr)
{
	// Capture startup time.
	_start_time = hrt_absolute_time();
}

BATT_SMBUS::~BATT_SMBUS()
{
	// Ensure we are truly inactive.
	stop();

	if (_manufacturer_name != nullptr) {
		delete[] _manufacturer_name;
	}

	// Switch back to ADC battery measurement.
	PX4_ERR("Failed to start smart battery. Switching to ADC measurement.");

	int battsource = 0;
	param_set(param_find("BAT_SOURCE"), &battsource);
}

uint16_t
BATT_SMBUS::convert_twos_comp(uint16_t val)
{
	// Temporary variable.
	uint16_t result = 0;

	if ((val & 0x8000) == 0x8000) {
		result = ~val;
		result += 1;
		return result;
	}

	return PX4_ERROR;
}

void
BATT_SMBUS::cycle_trampoline(void *arg)
{
	BATT_SMBUS *dev = (BATT_SMBUS *)arg;
	dev->cycle();
}

void
BATT_SMBUS::cycle()
{
	// Get the current time.
	uint64_t now = hrt_absolute_time();

	// Exit without rescheduling if we have failed to find a battery after 10 seconds.
	if (!_enabled && (now - _start_time > BATT_SMBUS_TIMEOUT_US)) {
		PX4_INFO("did not find smart battery");
		return;
	}

	// Read data from sensor.
	battery_status_s new_report = {};

	if ((_last_report.remaining < 0.0f) || (_last_report.remaining > 1.0f)) {
		_last_report.remaining = 0.5f;
	}

	// Set time of reading.
	new_report.timestamp = now;

	// Don't publish if any reads fail.
	bool success = true;

	// Temporary variable for storing SMBUS reads.
	uint16_t tmp;

	if (read_reg(BATT_SMBUS_VOLTAGE, tmp) == OK) {

		new_report.connected = true;

		// Convert millivolts to volts.
		new_report.voltage_v = ((float)tmp) / 1000.0f;
		new_report.voltage_filtered_v = new_report.voltage_v;

		// Read current.
		if (read_reg(BATT_SMBUS_CURRENT, tmp) == OK) {
			new_report.current_a = ((float)convert_twos_comp(tmp)) / 1000.0f;
			new_report.current_filtered_a = new_report.current_a;

		} else {
			success = false;
		}

		// Read average current.
		if (read_reg(BATT_SMBUS_AVERAGE_CURRENT, tmp) == OK) {
			new_report.average_current_a = ((float)convert_twos_comp(tmp)) / 1000.0f;

		} else {
			success = false;
		}

		// Read run time to empty.
		if (read_reg(BATT_SMBUS_RUN_TIME_TO_EMPTY, tmp) == OK) {
			new_report.run_time_to_empty = tmp;

		} else {
			success = false;
		}

		// Read average time to empty.
		if (read_reg(BATT_SMBUS_AVERAGE_TIME_TO_EMPTY, tmp) == OK) {
			new_report.average_time_to_empty = tmp;

		} else {
			success = false;
		}

		// Read remaining capacity.
		if (read_reg(BATT_SMBUS_REMAINING_CAPACITY, tmp) == OK) {

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
					PX4_WARN("Battery Warning Emergency: %4.2f", (double)new_report.remaining);
					new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
				}
			}

		} else {
			success = false;
		}

		// Read battery temperature and covert to Celsius.
		if (read_reg(BATT_SMBUS_TEMP, tmp) == OK) {
			new_report.temperature = (float)(((float)tmp / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS);

		} else {
			success = false;
		}

		new_report.capacity = _batt_capacity;
		new_report.cycle_count = _cycle_count;
		new_report.serial_number = _serial_number;

		// Publish to orb.
		if (_batt_topic != nullptr) {
			if (success == true) {
				orb_publish(_batt_orb_id, _batt_topic, &new_report);

				// Copy report for test().
				_last_report = new_report;
			}

		} else {
			_batt_topic = orb_advertise(_batt_orb_id, &new_report);

			if (_batt_topic == nullptr) {
				PX4_ERR("ADVERT FAIL");
				return;
			}
		}

		// Record we are working.
		_enabled = true;
	}

	// schedule a fresh cycle call when the measurement is done
	work_queue(HPWORK, &_work, (worker_t)&BATT_SMBUS::cycle_trampoline, this,
		   USEC2TICK(BATT_SMBUS_MEASUREMENT_INTERVAL_US));
}

int
BATT_SMBUS::init()
{
	int result = PX4_ERROR;

	// attempt to initialize I2C bus
	result = I2C::init();

	if (result != OK) {
		PX4_ERR("Failed to init I2C");
		return result;
	}

	// Find the battery on the bus and read startup info
	if (search_addresses() != PX4_OK) {
		PX4_ERR("Failed to init I2C");
		return result;
	}

	// Retry up to 10 times to read startup info
	for (size_t i = 0; i < 10; i++) {
		result = get_startup_info();

		if (result == PX4_OK) {
			break;
		}
	}

	if (result != PX4_OK) {
		PX4_ERR("Failed to get battery startup info");
		return result;
	}

	// Start the work queue.
	start();

	// Initialize the orb ID.
	_batt_orb_id = ORB_ID(battery_status);

	return result;
}

uint16_t
BATT_SMBUS::manufacture_date()
{
	uint16_t mfg_date;

	if (read_reg(BATT_SMBUS_MANUFACTURE_DATE, mfg_date) == OK) {
		return mfg_date;
	}

	// Return an error if could not read the date correctly.
	return PX4_ERROR;
}

uint8_t
BATT_SMBUS::manufacturer_name(uint8_t *mfr_name, uint8_t max_length)
{
	uint8_t length = read_block(BATT_SMBUS_MANUFACTURER_NAME, mfr_name, max_length, false);

	if (length > 0) {
		if (length >= max_length - 1) {
			mfr_name[max_length - 1] = 0;

		} else {
			mfr_name[length] = 0;
		}
	}

	return length;
}

int
BATT_SMBUS::search_addresses()
{
	bool found_slave = false;
	uint16_t tmp;
	uint8_t orig_addr = get_device_address();

	int result = PX4_ERROR;

	// search through all valid SMBus addresses
	for (uint8_t i = BATT_SMBUS_ADDR_MIN; i < BATT_SMBUS_ADDR_MAX; i++) {
		set_device_address(i);

		if (read_reg(BATT_SMBUS_VOLTAGE, tmp) == OK) {
			if (tmp > 0) {
				PX4_INFO("battery found at 0x%x", get_device_address());
				found_slave = true;
				break;
			}
		}

		// short sleep
		usleep(1);
	}

	if (found_slave == false) {
		// restore original i2c address
		set_device_address(orig_addr);
	}

	// display completion message
	if (found_slave) {
		PX4_INFO("smart battery connected");
		result = PX4_OK;
	} else {
		PX4_WARN("No smart batteries found.");
	}

	return result;
}

uint16_t
BATT_SMBUS::serial_number()
{
	uint16_t serial_num = 0;

	if (read_reg(BATT_SMBUS_SERIAL_NUMBER, serial_num) == OK) {
		return serial_num;
	}

	return PX4_ERROR;
}

int
BATT_SMBUS::probe()
{
	// Always return OK if the BATT_SMBUS object exists.
	return PX4_OK;
}

void
BATT_SMBUS::start()
{
	// Schedule a cycle to start things.
	work_queue(HPWORK, &_work, (worker_t)&BATT_SMBUS::cycle_trampoline, this, 1);
}

void
BATT_SMBUS::stop()
{
	// Cancel the work queue.
	work_cancel(HPWORK, &_work);
}

int
BATT_SMBUS::test()
{
	int sub = orb_subscribe(ORB_ID(battery_status));
	bool updated = false;
	struct battery_status_s status;
	uint64_t start_time = hrt_absolute_time();

	// Loop for 3 seconds
	while ((hrt_absolute_time() - start_time) < 3000000) {

		// Display new info that has arrived from the orb
		orb_check(sub, &updated);

		if (updated) {
			if (orb_copy(ORB_ID(battery_status), sub, &status) == OK) {
				print_message(status);
			}
		}

		// Sleep for 0.2 seconds
		usleep(200000);
	}

	return OK;
}

int
BATT_SMBUS::read_reg(uint8_t reg, uint16_t &val)
{
	// Buffer to hold 2 bytes of data.
	uint8_t buffer[3];

	// Read from register.
	int result = transfer(&reg, 1, buffer, 3);

	if (result == OK) {
		// Check PEC.
		uint8_t pec = get_PEC(reg, true, buffer, 2);

		if (pec == buffer[2]) {
			val = (uint16_t)buffer[1] << 8 | (uint16_t)buffer[0];

		} else {
			PX4_WARN("BATT_SMBUS PEC Check Failed");
			result = PX4_ERROR;
		}
	}

	// Return success or failure.
	return result;
}

uint8_t
BATT_SMBUS::read_block(uint8_t reg, uint8_t *data, uint8_t max_length, bool append_zero)
{
	uint8_t buffer[max_length + 2];  // buffer to hold results

	// Read bytes including PEC.
	int result = transfer(&reg, 1, buffer, max_length + 2);

	// Return zero on failure.
	if (result != PX4_OK) {
		PX4_WARN("Read register block data transfer failed.");
		return PX4_ERROR;
	}

	// Initialize the buffer length.
	uint8_t buffer_length = buffer[0];

	// Sanity check length returned by smbus.
	if (buffer_length == 0 ||
	    buffer_length > max_length) {
		PX4_WARN("Buffer length error.");
		return PX4_ERROR;
	}

	// Check PEC.
	uint8_t pec = get_PEC(reg, true, buffer, buffer_length + 1);

	if (pec != buffer[buffer_length + 1]) {
		PX4_WARN("PEC failure.");
		return PX4_ERROR;
	}

	// Copy data.
	memcpy(data, &buffer[1], buffer_length);

	// Optionally add zero to end.
	if (append_zero) {
		data[buffer_length] = '\0';
	}

	// Return success.
	return buffer_length;
}

int
BATT_SMBUS::write_reg(uint8_t reg, uint16_t val)
{
	uint8_t buff[4];  // reg + 2 bytes of data + PEC

	buff[0] = reg;
	buff[2] = uint8_t(val << 8) & 0xff;
	buff[1] = (uint8_t)val;
	buff[3] = get_PEC(reg, false, &buff[1],  2); // Append PEC

	// write bytes to register
	int ret = transfer(buff, 3, nullptr, 0);

	if (ret != OK) {
		PX4_DEBUG("Register write ERROR");
	}

	// return success or failure
	return ret;
}

uint8_t
BATT_SMBUS::write_block(uint8_t reg, uint8_t *data, uint8_t length)
{
	// buffer to hold results
	uint8_t buff[length + 3];
	int result = PX4_ERROR;

	usleep(1);

	buff[0] = reg;
	buff[1] = length;
	memcpy(&buff[2], data, length);

	// Append PEC
	buff[length + 2] = get_PEC(reg, false, &buff[1],  length + 1);

	// Send bytes
	result = transfer(buff, length + 3, nullptr, 0);

	// Return zero on failure
	if (result != OK) {
		PX4_DEBUG("Block write ERROR");
		return 0;
	}

	// Return success.
	return result;
}

uint8_t
BATT_SMBUS::get_PEC(uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len)
{
	// Exit immediately if no data
	if (len <= 0) {
		return 0;
	}

	/**
	 *  Note: The PEC is calculated on all the message bytes.
	 *        See http://cache.freescale.com/files/32bit/doc/app_note/AN4471.pdf
	 *        and http://www.ti.com/lit/an/sloa132/sloa132.pdf for more details.
	 */

	// Prepare temp buffer for calculating crc.
	uint8_t tmp_buff_len;

	if (reading) {
		tmp_buff_len = len + 3;

	} else {
		tmp_buff_len = len + 2;
	}

	uint8_t tmp_buff[tmp_buff_len];
	tmp_buff[0] = (uint8_t)get_device_address() << 1;
	tmp_buff[1] = cmd;

	if (reading) {
		tmp_buff[2] = tmp_buff[0] | (uint8_t)reading;
		memcpy(&tmp_buff[3], buff, len);

	} else {
		memcpy(&tmp_buff[2], buff, len);
	}

	// Initialise crc to zero
	uint8_t crc = 0;
	uint8_t shift_register = 0;
	bool do_invert;

	// Calculate crc for each byte in the stream
	for (uint8_t i = 0; i < sizeof(tmp_buff); i++) {
		// Load next data byte into the shift register
		shift_register = tmp_buff[i];

		// Calculate crc for each bit in the current byte
		for (uint8_t j = 0; j < 8; j++) {
			do_invert = (crc ^ shift_register) & 0x80;
			crc <<= 1;
			shift_register <<= 1;

			if (do_invert) {
				crc ^= BATT_SMBUS_PEC_POLYNOMIAL;
			}
		}
	}

	// Return result
	return crc;
}

uint8_t
BATT_SMBUS::get_startup_info()
{
	int result = PX4_ERROR;

	// Try and get battery SBS info.
	if (_manufacturer_name == nullptr) {
		char man_name[21];
		uint8_t len = manufacturer_name((uint8_t *)man_name, sizeof(man_name));

		if (len > 0) {
			_manufacturer_name = new char[len + 1];
			strcpy(_manufacturer_name, man_name);
		}
	}

	// Temporary variable for storing SMBUS reads.
	uint16_t tmp;

	// Read battery serial number on startup
	if (_serial_number == 0) {
		if (read_reg(BATT_SMBUS_SERIAL_NUMBER, tmp) == OK) {
			_serial_number = tmp;
			result = PX4_OK;
		}
	}

	// Read battery capacity on startup
	if (_batt_startup_capacity == 0) {
		if (read_reg(BATT_SMBUS_REMAINING_CAPACITY, tmp) == OK) {
			_batt_startup_capacity = tmp;
			result = PX4_OK;
		}
	}

	// Read battery cycle count on startup
	if (_cycle_count == 0) {
		if (read_reg(BATT_SMBUS_CYCLE_COUNT, tmp) == OK) {
			_cycle_count = tmp;
			result = PX4_OK;
		}
	}

	// Read battery design capacity on startup
	if (_batt_capacity == 0) {
		if (read_reg(BATT_SMBUS_FULL_CHARGE_CAPACITY, tmp) == OK) {
			_batt_capacity = tmp;
			result = PX4_OK;
		}
	}

	// Read battery threshold params on startup
	if (_crit_thr < 0.01f) {
		param_get(param_find("BAT_CRIT_THR"), &_crit_thr);
	}

	if (_low_thr < 0.01f) {
		param_get(param_find("BAT_LOW_THR"), &_low_thr);
	}

	if (_emergency_thr < 0.01f) {
		param_get(param_find("BAT_EMERGEN_THR"), &_emergency_thr);
	}

	return result;
}

uint8_t
BATT_SMBUS::manufacturer_access(uint16_t cmd)
{
	// write bytes to Manufacturer Access
	int result = write_reg(BATT_SMBUS_MANUFACTURER_ACCESS, cmd);

	if (result != OK) {
		PX4_WARN("Manufacturer Access ERROR");
	}

	return result;
}

///////////////////////// shell functions ///////////////////////

void
batt_smbus_usage()
{
	PX4_INFO("missing command: try 'start', 'test', 'stop', 'search', 'man_name', 'man_date', 'dev_name', 'serial_num', 'dev_chem',  'sbs_info'");
	PX4_INFO("options:");
	PX4_INFO("    -b i2cbus (%d)", BATT_SMBUS_I2C_BUS);
	PX4_INFO("    -a addr (0x%x)", BATT_SMBUS_ADDR);
}

int
manufacture_date()
{
	uint16_t man_date = g_batt_smbus->manufacture_date();

	int result = PX4_ERROR;

	if (man_date > 0) {
		// Convert the uint16_t into human-readable date format
		uint16_t year = ((man_date >> 9) & 0xFF) + 1980;
		uint8_t month = (man_date >> 5) & 0xF;
		uint8_t day = man_date & 0x1F;
		PX4_INFO("The manufacturer date is: %d which is %4d-%02d-%02d", man_date, year, month, day);
		result = PX4_OK;

	} else {
		PX4_WARN("Unable to read the manufacturer date.");
	}

	return result;
}

int
manufacturer_name()
{
	int result = PX4_ERROR;

	uint8_t man_name[21];
	uint8_t length = g_batt_smbus->manufacturer_name(man_name, sizeof(man_name));

	if (length > 0) {
		PX4_INFO("The manufacturer name: %s", man_name);
		result = PX4_OK;

	} else {
		PX4_WARN("Unable to read manufacturer name.");
	}

	return result;
}

int
serial_number()
{
	uint16_t serial_num = g_batt_smbus->serial_number();
	PX4_INFO("The serial number: 0x%04x (%d in decimal)", serial_num, serial_num);

	return PX4_OK;
}

int
batt_smbus_main(int argc, char *argv[])
{
	int i2cdevice = BATT_SMBUS_I2C_BUS;
	int batt_smbusadr = BATT_SMBUS_ADDR; // 7bit address

	int ch;

	// Jump over start/off/etc and look at options first
	while ((ch = getopt(argc, argv, "a:b")) != EOF) {
		switch (ch) {
		case 'a':
			batt_smbusadr = strtol(optarg, nullptr, 0);
			break;

		case 'b':
			i2cdevice = strtol(optarg, nullptr, 0);
			break;

		default:
			batt_smbus_usage();
			return 0;
		}
	}

	if (optind >= argc) {
		batt_smbus_usage();
		return 1;
	}

	const char *verb = argv[optind];

	if (!strcmp(verb, "start")) {
		if (g_batt_smbus != nullptr) {
			PX4_ERR("already started");
			return 1;

		} else {
			// create new global object
			g_batt_smbus = new BATT_SMBUS(i2cdevice, batt_smbusadr);

			if (g_batt_smbus == nullptr) {
				PX4_ERR("new failed");
				return 1;
			}

			if (OK != g_batt_smbus->init()) {
				delete g_batt_smbus;
				g_batt_smbus = nullptr;
				PX4_ERR("init failed");
				return 1;
			}
		}

		return 0;
	}

	// need the driver past this point
	if (g_batt_smbus == nullptr) {
		PX4_INFO("not started");
		batt_smbus_usage();
		return 1;
	}

	if (!strcmp(verb, "test")) {
		g_batt_smbus->test();
		return 0;
	}

	if (!strcmp(verb, "stop")) {
		delete g_batt_smbus;
		g_batt_smbus = nullptr;
		return 0;
	}

	if (!strcmp(verb, "search")) {
		g_batt_smbus->search_addresses();
		return 0;
	}

	if (!strcmp(verb, "man_name")) {
		manufacturer_name();
		return 0;
	}

	if (!strcmp(verb, "man_date")) {
		manufacture_date();
		return 0;
	}

	if (!strcmp(verb, "serial_num")) {
		serial_number();
		return 0;
	}

	if (!strcmp(verb, "sbs_info")) {
		manufacturer_name();
		manufacture_date();
		serial_number();
		return 0;
	}

	batt_smbus_usage();
	return 0;
}
