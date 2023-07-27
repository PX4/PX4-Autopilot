/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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
 * Driver for the MCP23009 connected via I2C.
 */

#include "mcp23009.h"
#include <px4_platform_common/module.h>

MCP23009::MCP23009(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample"))
{
}

MCP23009::~MCP23009()
{
	ScheduleClear();
	perf_free(_cycle_perf);
}

int MCP23009::init_uorb()
{
	if (!_gpio_config_sub.registerCallback() ||
	    !_gpio_request_sub.registerCallback() ||
	    !_gpio_out_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return -1;
	}

	return PX4_OK;
}

void MCP23009::exit_and_cleanup()
{
	_gpio_config_sub.unregisterCallback();
	_gpio_request_sub.unregisterCallback();
	_gpio_out_sub.unregisterCallback();
}

void MCP23009::RunImpl()
{
	perf_begin(_cycle_perf);

	gpio_config_s config;

	if (_gpio_config_sub.update(&config) && config.device_id == get_device_id()) {
		PinType type = PinType::Input;

		switch (config.config) {
		case config.INPUT_PULLUP:	type = PinType::InputPullUp; break;

		case config.OUTPUT:		type = PinType::Output; break;
		}

		write(config.state, config.mask);
		configure(config.mask, type);
	}

	gpio_out_s output;

	if (_gpio_out_sub.update(&output) && output.device_id == get_device_id()) {
		write(output.state, output.mask);
	}

	// read every time we run, either when requested or when scheduled on interval
	{
		gpio_in_s _gpio_in;
		_gpio_in.timestamp = hrt_absolute_time();
		_gpio_in.device_id = get_device_id();
		uint8_t input;
		read(&input);
		_gpio_in.state = input;
		_to_gpio_in.publish(_gpio_in);
	}

	perf_end(_cycle_perf);
}

void MCP23009::print_usage()
{
	PRINT_MODULE_USAGE_NAME("MCP23009", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x25);
	PRINT_MODULE_USAGE_PARAM_INT('D', 0, 0, 255, "Direction", true);
	PRINT_MODULE_USAGE_PARAM_INT('O', 0, 0, 255, "Output", true);
	PRINT_MODULE_USAGE_PARAM_INT('P', 0, 0, 255, "Pullups", true);
	PRINT_MODULE_USAGE_PARAM_INT('U', 0, 0, 1000, "Update Interval [ms]", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void MCP23009::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
}

struct init_config_t {
	uint16_t interval;
	uint8_t direction;
	uint8_t state;
	uint8_t pullup;
};

I2CSPIDriverBase *MCP23009::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	auto *init = (const init_config_t *)config.custom_data;
	auto *instance = new MCP23009(config);

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (OK != instance->init(init->direction, init->state, init->pullup)) {
		delete instance;
		return nullptr;
	}

	if (init->interval) {
		instance->ScheduleOnInterval(init->interval * 1000);
	}

	return instance;
}

extern "C" int mcp23009_main(int argc, char *argv[])
{
	using ThisDriver = MCP23009;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = 0x25;
	init_config_t config_data{};

	int ch;

	while ((ch = cli.getOpt(argc, argv, "D:O:P:U:")) != EOF) {
		switch (ch) {
		case 'D':
			config_data.direction = (int)strtol(cli.optArg(), nullptr, 0);
			break;

		case 'O':
			config_data.state = (int)strtol(cli.optArg(), nullptr, 0);
			break;

		case 'P':
			config_data.pullup = (int)strtol(cli.optArg(), nullptr, 0);
			break;

		case 'U':
			config_data.interval = atoi(cli.optArg());
			break;
		}
	}

	cli.custom_data = &config_data;

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_GPIO_DEVTYPE_MCP23009);

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
