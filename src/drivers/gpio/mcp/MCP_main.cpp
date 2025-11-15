/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
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

#include "MCP.hpp"
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include "MCP23009.hpp"
#include "MCP23017.hpp"


void MCP::print_usage()
{
	PRINT_MODULE_USAGE_NAME("MCP", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x27);
	PRINT_MODULE_USAGE_PARAM_INT('D', 0, 0, 65535, "Direction (1=Input, 0=Output)", true);
	PRINT_MODULE_USAGE_PARAM_INT('O', 0, 0, 65535, "Output", true);
	PRINT_MODULE_USAGE_PARAM_INT('P', 0, 0, 65535, "Pullups", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 65535, "Interrupt pins enable", true);
	PRINT_MODULE_USAGE_PARAM_INT('U', 0, 0, 1000, "Update Interval [ms]", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('A', "Use MCP23009", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('B', "Use MCP23017", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

struct init_config_t {
	uint16_t interval;
	uint16_t direction;
	uint16_t state;
	uint16_t pullup;
	uint16_t int_en;
	uint16_t ref_vals;
	bool split_int;
};

I2CSPIDriverBase *MCP::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	printf("MCP_main: instatiate called \n");
	auto *init = (const init_config_t *)config.custom_data;
	MCP *instance = nullptr;

	if(config.devid_driver_index == DRV_GPIO_DEVTYPE_MCP23009){

		instance = new MCP23009(config);

	} else if (config.devid_driver_index == DRV_GPIO_DEVTYPE_MCP23017){

		instance = new MCP23017(config);
	}

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

	instance->init_uorb();

	return instance;
}


extern "C" int mcp_main(int argc, char *argv[])
{
	printf("MCP_main: main called \n");
	using ThisDriver = MCP;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = I2C_ADDRESS_MCP23009;
	init_config_t config_data{};

	config_data.split_int = false;
	config_data.ref_vals = 0x0000;

	int ch;
	uint16_t device_type = 0;
	const char *name = MODULE_NAME;

	printf("name: %s \n", name);



	while ((ch = cli.getOpt(argc, argv, "ABD:O:P:U:R:")) != EOF) {
		switch (ch) {
		case 'A':
			printf("you chose to use MCP23009 \n");
			device_type = DRV_GPIO_DEVTYPE_MCP23009;
			name = MODULE_NAME "23009";
			cli.i2c_address = I2C_ADDRESS_MCP23009;
			break;

		case 'B':
			printf("you chose to use MCP23017 \n");
			device_type = DRV_GPIO_DEVTYPE_MCP23017;
			name = MODULE_NAME "23017";
			cli.i2c_address = I2C_ADDRESS_MCP23017;
			break;
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

		case 'R':
			config_data.int_en = (int)strtol(cli.optArg(), nullptr, 0);
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb || device_type == 0) {
		ThisDriver::print_usage();
		return -1;
	}

	printf("12321: Name: %s, device_type: %d \n", name, device_type);

	cli.custom_data = &config_data;
	BusInstanceIterator iterator(name, cli, device_type);

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
