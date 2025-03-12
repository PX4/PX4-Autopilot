/****************************************************************************
 *
 *   Copyright (C) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file ads1115_main.cpp
 * @author SalimTerryLi
 *
 * Driver for the ADS1115 connected via I2C.
 */

#include "ADS1115.h"
#include <px4_platform_common/module.h>
#include <drivers/drv_adc.h>

ADS1115::ADS1115(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
	_adc_report.device_id = this->get_device_id();
	_adc_report.resolution = 32768;
	_adc_report.v_ref = 6.144f;

	for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
		_adc_report.channel_id[i] = -1;
	}

}

ADS1115::~ADS1115()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);
}

void ADS1115::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();	// nothing to do
}

void ADS1115::RunImpl()
{
	if (should_exit()) {
		PX4_INFO("stopping");
		return;	// stop and return immediately to avoid unexpected schedule from stopping procedure
	}

	perf_begin(_cycle_perf);

	const int ready = isSampleReady();

	if (ready == 1) {
		// I2C transaction success and status register reported conversion as finished
		if (_ready_counter == 0) { PX4_INFO("ADS1115: reported ready"); }

		if (_ready_counter < MAX_READY_COUNTER) { _ready_counter++; }

		int16_t value;
		Channel ch = getMeasurement(&value);

		if (ch != Channel::Invalid) {
			// Store current readings and mark channel as read
			const unsigned index{ch2u(ch)};
			_adc_report.channel_id[index] = index;
			_adc_report.raw_data[index] = value;
			_channel_cycle_mask |= 1u << index;

		} else {
			// we will retry the same channel again
			perf_count(_comms_errors);
		}

		// Find the next unread channel in the bitmask
		uint8_t next_index{0};

		for (; next_index < 4 && (_channel_cycle_mask & (1u << next_index)); next_index++) {}

		readChannel(u2ch(next_index));

		if (_channel_cycle_mask == 0b1111) {
			_channel_cycle_mask = 0;
			_to_adc_report.publish(_adc_report);
		}

	} else if (ready == 0) {
		// I2C transaction success but status register reported conversion still in progress
		perf_count(_comms_errors);
		// Reset the channel to unstick the device
		readChannel(Channel::A0);

	} else if (ready == -1) {
		if (_ready_counter == 1) { PX4_ERR("ADS1115: device lost"); }

		if (_ready_counter > 0) { _ready_counter--; }

		perf_count(_comms_errors);
		// Reset the channel to unstick the device
		readChannel(Channel::A0);
	}

	perf_end(_cycle_perf);
}

void ADS1115::print_usage()
{

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Driver to enable an external [ADS1115](https://www.adafruit.com/product/1085) ADC connected via I2C.

The driver is included by default in firmware for boards that do not have an internal analog to digital converter,
such as [PilotPi](../flight_controller/raspberry_pi_pilotpi.md) or [CUAV Nora](../flight_controller/cuav_nora.md)
(search for `CONFIG_DRIVERS_ADC_ADS1115` in board configuration files).

It is enabled/disabled using the
[ADC_ADS1115_EN](../advanced_config/parameter_reference.md#ADC_ADS1115_EN)
parameter, and is disabled by default.
If enabled, internal ADCs are not used.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ads1115", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x48);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void ADS1115::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
	perf_print_counter(_comms_errors);
}

extern "C" int ads1115_main(int argc, char *argv[])
{
	using ThisDriver = ADS1115;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = 0x48;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ADC_DEVTYPE_ADS1115);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
