/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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
 * @file batt_smbus.h
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for BQ40Z50-R1/R2 and BQ40Z80
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Bazooka Joe <BazookaJoe1900@gmail.com>
 * @author Nick Belanger <nbelanger@mail.skymul.com>
 * @author Eohan George <eg@.skymul.com>
 *
 */

#include "batt_smbus.h"
#include "rotoye_batmon/rotoye_batmon.h"
#include "bq40zx/bq40zx.h"

extern "C" __EXPORT int batt_smbus_main(int argc, char *argv[]);

BATT_SMBUS::BATT_SMBUS(I2CSPIBusOption bus_option, const int bus, SMBus *interface) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus),
	_interface(interface)
{
	battery_status_s new_report = {};
	_batt_topic = orb_advertise(ORB_ID(battery_status), &new_report);

	int32_t battsource = 1;
	int32_t batt_device_type = (int)SMBUS_DEVICE_TYPE::UNDEFINED;

	param_set(param_find("BAT_SOURCE"), &battsource);
	param_get(param_find("BAT_SMBUS_MODEL"), &batt_device_type);


	//TODO: probe the device and autodetect its type
	_device_type = (SMBUS_DEVICE_TYPE)batt_device_type;

	_interface->init();
	// unseal() here to allow an external config script to write to protected flash.
	// This is neccessary to avoid bus errors due to using standard i2c mode instead of SMbus mode.
	// The external config script should then seal() the device.

	// TODO: implement this somewhere in BQ-specific class
	//unseal();


}
BATT_SMBUS::~BATT_SMBUS()
{
	orb_unadvertise(_batt_topic);
	perf_free(_cycle);

	if (_interface != nullptr) {
		delete _interface;
	}

	int battsource = 0;
	param_set(param_find("BAT_SOURCE"), &battsource);
}

void BATT_SMBUS::suspend()
{
	ScheduleClear();
}

void BATT_SMBUS::resume()
{
	ScheduleOnInterval(BATT_SMBUS_MEASUREMENT_INTERVAL_US);
}

uint16_t BATT_SMBUS::get_serial_number()
{
	uint16_t serial_num = 0;

	if (_interface->read_word(BATT_SMBUS_SERIAL_NUMBER, serial_num) == PX4_OK) {
		return serial_num;
	}

	return PX4_ERROR;
}

int BATT_SMBUS::get_startup_info()
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

	if (!ret) {
		_serial_number = serial_num;
		_batt_startup_capacity = (uint16_t)((float)remaining_cap * _c_mult);
		_cycle_count = cycle_count;
		_batt_capacity = (uint16_t)((float)full_cap * _c_mult);
		_manufacture_date = manufacture_date;
	}

	return ret;
}

int BATT_SMBUS::populate_smbus_data(battery_status_s &data) {

	// Temporary variable for storing SMBUS reads.
	uint16_t result;

	int ret = _interface->read_word(BATT_SMBUS_VOLTAGE, result);

	// Convert millivolts to volts.
	data.voltage_v = ((float)result) / 1000.0f;
	data.voltage_filtered_v = data.voltage_v;

	// Read current.
	ret |= _interface->read_word(BATT_SMBUS_CURRENT, result);

	data.current_a = (-1.0f * ((float)(*(int16_t *)&result)) / 1000.0f) * _c_mult;
	data.current_filtered_a = data.current_a;

	// Read remaining capacity.
	ret |= _interface->read_word(BATT_SMBUS_RELATIVE_SOC, result);
	data.remaining = (float)result/100;

	// Read remaining capacity.
	ret |= _interface->read_word(BATT_SMBUS_REMAINING_CAPACITY, result);
	data.discharged_mah = _batt_startup_capacity - result;

	// Read full capacity.
	ret |= _interface->read_word(BATT_SMBUS_FULL_CHARGE_CAPACITY, result);
	data.capacity = result;

	// Read cycle count.
	ret |= _interface->read_word(BATT_SMBUS_CYCLE_COUNT, result);
	data.cycle_count = result;

	// Read serial number.
	ret |= _interface->read_word(BATT_SMBUS_SERIAL_NUMBER, result);
	data.serial_number = result;

	// Read battery temperature and covert to Celsius.
	ret |= _interface->read_word(BATT_SMBUS_TEMP, result);
	data.temperature = ((float)result / 10.0f) + CONSTANTS_ABSOLUTE_NULL_CELSIUS;

	return ret;

}

void BATT_SMBUS::RunImpl() {

	// Get the current time.
	uint64_t now = hrt_absolute_time();

	// Read data from sensor.
	battery_status_s new_report = {};

	new_report.id = 1;

	// Set time of reading.
	new_report.timestamp = now;

	new_report.connected = true;

	int ret = populate_smbus_data(new_report);

	new_report.cell_count = _cell_count;

	// Only publish if no errors.
	if (!ret) {
		orb_publish(ORB_ID(battery_status), _batt_topic, &new_report);

		_last_report = new_report;
	}
}


int BATT_SMBUS::manufacture_date()
{
	uint16_t date;
	int result = _interface->read_word(BATT_SMBUS_MANUFACTURE_DATE, date);

	if (result != PX4_OK) {
		return result;
	}

	return date;
}

int BATT_SMBUS::manufacturer_name(uint8_t *man_name, const uint8_t length)
{
	uint8_t code = BATT_SMBUS_MANUFACTURER_NAME;
	uint8_t rx_buf[21] = {};

	// Returns 21 bytes, add 1 byte for null terminator.
	int result = _interface->block_read(code, rx_buf, length - 1, true);

	memcpy(man_name, rx_buf, sizeof(rx_buf));

	man_name[21] = '\0';

	return result;
}

void BATT_SMBUS::custom_method(const BusCLIArguments &cli)
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

void BATT_SMBUS::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Smart battery driver for the SMBus-enabled batteries

### Examples
To write to flash to set parameters. address, number_of_bytes, byte0, ... , byteN
$ batt_smbus -X write_flash 19069 2 27 0

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("batt_smbus", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x0B);

	PRINT_MODULE_USAGE_COMMAND_DESCR("man_info", "Prints manufacturer info.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("unseal", "Unseals the devices flash memory to enable write_flash commands.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("seal", "Seals the devices flash memory to disbale write_flash commands.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("suspend", "Suspends the driver from rescheduling the cycle.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("resume", "Resumes the driver from suspension.");

	PRINT_MODULE_USAGE_COMMAND_DESCR("write_flash", "Writes to flash. The device must first be unsealed with the unseal command.");
	PRINT_MODULE_USAGE_ARG("address", "The address to start writing.", true);
	PRINT_MODULE_USAGE_ARG("number of bytes", "Number of bytes to send.", true);
	PRINT_MODULE_USAGE_ARG("data[0]...data[n]", "One byte of data at a time separated by spaces.", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *BATT_SMBUS::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	SMBus *interface = new SMBus(iterator.bus(), cli.i2c_address);
	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	int32_t batt_device_type = (int)SMBUS_DEVICE_TYPE::UNDEFINED;
	param_get(param_find("BAT_SMBUS_MODEL"), &batt_device_type);

	BATT_SMBUS *instance = nullptr;

	switch((SMBUS_DEVICE_TYPE)batt_device_type) {

		// TODO: implement seperate classes
		//       for the BQ40zx devices
		case SMBUS_DEVICE_TYPE::BQ40Z80:
		instance = new BQ40ZX(iterator.configuredBusOption(), iterator.bus(), interface);
		break;

		case SMBUS_DEVICE_TYPE::BQ40Z50:
		instance = new BQ40ZX(iterator.configuredBusOption(), iterator.bus(), interface);
		break;

		case SMBUS_DEVICE_TYPE::ROTOYE_BATMON:
		instance = new Rotoye_Batmon(iterator.configuredBusOption(), iterator.bus(), interface);
		break;

		default:
		instance = new BATT_SMBUS(iterator.configuredBusOption(), iterator.bus(), interface);
		break;
	}

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	int ret = instance->get_startup_info();

	if (ret != PX4_OK) {
		delete instance;
		return nullptr;
	}

	instance->ScheduleOnInterval(BATT_SMBUS_MEASUREMENT_INTERVAL_US);

	return instance;
}

extern "C" __EXPORT int batt_smbus_main(int argc, char *argv[])
{
	using ThisDriver = BATT_SMBUS;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = BATT_SMBUS_ADDR;

	const char *verb = cli.parseDefaultArguments(argc, argv);
	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_BAT_DEVTYPE_SMBUS);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "man_info")) {
		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}
	if (!strcmp(verb, "unseal")) {
		cli.custom1 = 2;
		return ThisDriver::module_custom_method(cli, iterator);
	}
	if (!strcmp(verb, "seal")) {
		cli.custom1 = 3;
		return ThisDriver::module_custom_method(cli, iterator);
	}
	if (!strcmp(verb, "suspend")) {
		cli.custom1 = 4;
		return ThisDriver::module_custom_method(cli, iterator);
	}
	if (!strcmp(verb, "resume")) {
		cli.custom1 = 5;
		return ThisDriver::module_custom_method(cli, iterator);
	}
	if (!strcmp(verb, "write_flash")) {
		cli.custom1 = 6;
		if (argc >= 3) {
			uint16_t address = atoi(argv[1]);
			unsigned length = atoi(argv[2]);
			uint8_t tx_buf[33];
			cli.custom_data = &tx_buf;

			if (length > 32) {
				PX4_WARN("Data length out of range: Max 32 bytes");
				return 1;
			}

			tx_buf[0] = length;
			// Data needs to be fed in 1 byte (0x01) at a time.
			for (unsigned i = 0; i < length; i++) {
				if ((unsigned)argc <= 3 + i) {
					tx_buf[i+1] = atoi(argv[3 + i]);
				}
			}
			cli.custom2 = address;
			return ThisDriver::module_custom_method(cli, iterator);
		}
	}

	ThisDriver::print_usage();
	return -1;
}
