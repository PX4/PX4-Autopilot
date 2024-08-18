/****************************************************************************
 *
 *   Copyright (c) 2021-2023 PX4 Development Team. All rights reserved.
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
 * @file sht3x.cpp
 *
 * I2C driver for temperature/humidity sensor SHT3x by senserion
 *
 * @author Roman Dvorak <dvorakroman@thunderfly.cz>
 *
 */

#include "sht3x.h"

SHT3X::SHT3X(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
{
}

uint8_t SHT3X::calc_crc(uint8_t data[2])
{
	uint8_t crc = 0xFF;

	for (int i = 0; i < 2; i++) {
		crc ^= data[i];

		for (uint8_t bit = 8; bit > 0; --bit) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x31u;

			} else {
				crc = (crc << 1);
			}
		}
	}

	return crc;
}


int SHT3X::set_pointer(uint16_t command)
{
	if (_last_command != command) {
		uint8_t cmd[2];
		cmd[0] = static_cast<uint8_t>(command >> 8);
		cmd[1] = static_cast<uint8_t>(command & 0xff);
		_last_command = command;
		return transfer(&cmd[0], 2, nullptr, 0);

	} else {
		return 0;
	}
}

int SHT3X::read_data(uint16_t command, uint8_t *data_ptr, uint8_t length)
{
	set_pointer(command);

	uint8_t raw_data[length];
	transfer(nullptr, 0, &raw_data[0], length);

	uint8_t crc_err = 0;

	for (int i = 0; i < length / 3; ++i) {
		uint8_t crc_data[2] = {raw_data[i * 3], raw_data[i * 3 + 1]};

		if (raw_data[i * 3 + 2] != calc_crc(crc_data)) {
			crc_err ++;
		}

		*(data_ptr + i * 2) = raw_data[i * 3];
		*(data_ptr + i * 2 + 1) = raw_data[i * 3 + 1];
	}

	return crc_err;
}

int SHT3X::write_data(uint16_t command, uint8_t buffer[], uint8_t length)
{
	_last_command = command;

	uint8_t cmd[2 + 3 * length / 2];
	cmd[0] = static_cast<uint8_t>(command >> 8);
	cmd[1] = static_cast<uint8_t>(command & 0xff);

	for (int i = 0; i < length / 2; ++i) {
		cmd[2 + 3 * i] = buffer[2 * i];
		cmd[2 + 3 * i + 1] = buffer[2 * i + 1];
		uint8_t crc_data[2] = {buffer[2 * i], buffer[2 * i + 1]};
		cmd[2 + 3 * i + 2] = calc_crc(crc_data);
	}

	return transfer(&cmd[0], sizeof(cmd), nullptr, 0);
}


void SHT3X::sensor_compouse_msg(bool send)
{
	uint8_t data[4];
	int error = read_data(SHT3x_CMD_FETCH_DATA, &data[0], 6);

	if (error == PX4_OK) {
		measurement_time = hrt_absolute_time();
		measurement_index ++;

		measured_temperature = (float) 175 * (data[0] << 8 | data[1]) / 65535 - 45;
		measured_humidity = (float) 100 * (data[2] << 8 | data[3]) / 65535;

		if (send) {
			sensor_hygrometer_s msg{};
			msg.timestamp = hrt_absolute_time();
			msg.timestamp_sample = measurement_time;
			msg.temperature = measured_temperature;
			msg.humidity = measured_humidity;
			msg.device_id = _sht_info.serial_number;
			_sensor_hygrometer_pub.publish(msg);
		}
	}
}


int
SHT3X::probe()
{
	uint8_t type[2];
	uint8_t nvalid;

	nvalid = read_data(SHT3X_CMD_READ_STATUS, &type[0], 3);
	return (!(nvalid == PX4_OK));
	// 0 means I can see sensor
}

int SHT3X::init()
{
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	_sensor_hygrometer_pub.advertise();
	ScheduleOnInterval(50000);
	return PX4_OK;
}



int SHT3X::init_sensor()
{
	set_pointer(SHT3X_CMD_CLEAR_STATUS);
	px4_usleep(2000);

	// Get sensor serial number
	uint8_t serial[4];
	read_data(SHT3x_CMD_READ_SN, &serial[0], 6);

	_sht_info.serial_number = ((uint32_t)serial[0] << 24
				   | (uint32_t)serial[1] << 16
				   | (uint32_t)serial[2] << 8
				   | (uint32_t)serial[3]);

	set_pointer(SHT3x_CMD_PERIODIC_2HZ_MEDIUM);
	px4_usleep(2000);
	probe();

	PX4_INFO("Connected to SHT3x sensor, SN: %ld", _sht_info.serial_number);

	// Modify ALERT range values to minimalize ALERT led heating.

	// def temp(t):
	//     return ((t+45)*(2**16-1))/175
	// def hum(h):
	//	return (h/100)*(2**16-1)
	//
	// hex((int(hum(80))>>9<<9) | (int(temp(60))>>7))

	uint8_t sh[] {0xff, 0xe2};
	uint8_t ch[] {0xfb, 0xdc};
	uint8_t sl[] {0x00, 0x0e};
	uint8_t cl[] {0x04, 0x14};
	write_data(SHT3x_CMD_ALERT_W_SET_HIGH, sh, 2);		// 100%, 120C
	write_data(SHT3x_CMD_ALERT_W_CLEAR_HIGH, ch, 2);	// 98%, 118C
	write_data(SHT3x_CMD_ALERT_W_SET_LOW, sl, 2);		// 0%, -40C
	write_data(SHT3x_CMD_ALERT_W_CLEAR_LOW, cl, 2);		// 2%, -38C

	return PX4_OK;
}

void SHT3X::RunImpl()
{
	switch (_state) {
	case sht3x_state::INIT:
		probe();
		init_sensor();
		_state = sht3x_state::MEASUREMENT;
		break;

	case sht3x_state::MEASUREMENT:
		if ((hrt_absolute_time() - measurement_time) > 200000) {
			sensor_compouse_msg(1);
		}

		if ((hrt_absolute_time() - measurement_time) > 3000000) {
			_state = sht3x_state::ERROR_READOUT;
		}

		break;

	case sht3x_state::ERROR_GENERAL:
	case sht3x_state::ERROR_READOUT: {
			if (_last_state != _state) {
				PX4_INFO("I cant get new data. The sensor may be disconnected.");
			}

			if (probe() == PX4_OK) {
				_state = sht3x_state::INIT;
			}
		}
		break;
	}

	if (_last_state != _state) {
		_time_in_state = hrt_absolute_time();
		_last_state = _state;
	}
}


void
SHT3X::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom1) {

	case 1: {
			PX4_INFO("Last measured values (%.3fs ago, #%d)", (double)(hrt_absolute_time() - measurement_time) / 1000000.0,
				 measurement_index);
			PX4_INFO("Temp: %.3f, Hum: %.3f", (double)measured_temperature, (double)measured_humidity);

		}
		break;

	case 2: {
			_state = sht3x_state::INIT;
		}
		break;
	}
}


void SHT3X::print_status()
{
	PX4_INFO("SHT3X sensor");
	I2CSPIDriverBase::print_status();
	PX4_INFO("SN: %ld", _sht_info.serial_number);
	PX4_INFO("Status: %s", sht_state_names[_state]);
}


void SHT3X::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
SHT3x Temperature and Humidity Sensor Driver by Senserion.

### Examples
CLI usage example:
$ sht3x start -X
  Start the sensor driver on the external bus

$ sht3x status
  Print driver status

$ sht3x values
  Print last measured values

$ sht3x reset
  Reinitialize senzor, reset flags

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sht3x", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x44);

	PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG();
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	PRINT_MODULE_USAGE_COMMAND_DESCR("values", "Print actual data");
    PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reinitialize sensor");

}

int sht3x_main(int argc, char *argv[])
{
	using ThisDriver = SHT3X;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = 0x44;
	cli.support_keep_running = true;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_HYGRO_DEVTYPE_SHT3X);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "values")) {
		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	if (!strcmp(verb, "reset")) {
		cli.custom1 = 2;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	ThisDriver::print_usage();
	return -1;
}
