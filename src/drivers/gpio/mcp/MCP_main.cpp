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
	PRINT_MODULE_USAGE_PARAM_INT('U', 0, 0, 1000, "Update Interval [ms]", true);
	PRINT_MODULE_USAGE_PARAM_INT('M', 0, 0, 255, "First minor number", true);
	PRINT_MODULE_USAGE_PARAM_INT('Q', 0, 0, 255, "I2C Bus", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('A', "Use MCP23009", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('B', "Use MCP23017", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

struct init_config_t {
	uint16_t device_type;

	uint16_t direction = 0xFFFF; // READ ON ALL PINS
	uint16_t state = 0x0000;
	uint16_t pullup = 0x0000;
	uint16_t interval = 1;
	int first_minor = 0;
	uint8_t i2c_addr = 0;
	uint8_t i2c_bus = 0;
};

I2CSPIDriverBase *MCP::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	printf("MCP_main: instatiate called \n");
	auto *init = (const init_config_t *)config.custom_data;
	MCP *instance = nullptr;

	switch (config.devid_driver_index) {
	case DRV_GPIO_DEVTYPE_MCP23009:
		instance = new MCP23009(config);
		break;

	case DRV_GPIO_DEVTYPE_MCP23017:
		instance = new MCP23017(config);
		break;

	default:
		instance = nullptr;
		break;
	}

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (OK != instance->init()) {
		delete instance;
		return nullptr;
	}

	instance->_iodir = init->direction;
	instance->_olat = init->state;
	instance->_gppu = init->pullup;

	instance->mcp_config.bus = init->i2c_bus;
	instance->mcp_config.i2c_addr = init->i2c_addr;
	instance->mcp_config.device_type = init->device_type;
	instance->mcp_config.first_minor = init->first_minor;
	instance->mcp_config.interval = init->interval;

	return instance;
}

extern "C" int mcp_main(int argc, char *argv[])
{

	using ThisDriver = MCP;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = I2C_ADDRESS_MCP23009;
	init_config_t config_data{};

	int ch;
	uint16_t device_type = 0;
	const char *name = MODULE_NAME;

	while ((ch = cli.getOpt(argc, argv, "ABD:O:P:U:R:M:Q:")) != EOF) {
		switch (ch) {
		case 'A':
			printf("you chose to use MCP23009 \n");
			device_type = DRV_GPIO_DEVTYPE_MCP23009;
			name = MODULE_NAME "23009";
			cli.i2c_address = I2C_ADDRESS_MCP23009;
			config_data.i2c_addr = I2C_ADDRESS_MCP23009;
			break;

		case 'B':
			printf("you chose to use MCP23017 \n");
			device_type = DRV_GPIO_DEVTYPE_MCP23017;
			name = MODULE_NAME "23017";
			cli.i2c_address = I2C_ADDRESS_MCP23017;
			config_data.i2c_addr = I2C_ADDRESS_MCP23017;
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

		case 'M':
			config_data.first_minor = (uint8_t)atoi(cli.optArg());
			break;

		case 'Q':
			int tmp_bus = (int)atoi(cli.optArg());
			cli.requested_bus = tmp_bus;
			config_data.i2c_bus = (uint8_t) tmp_bus;
			break;

		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

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
