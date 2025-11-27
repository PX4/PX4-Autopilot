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
#include <lib/drivers/mcp_common/MCP.hpp>
#include "MCP23009.hpp"

extern "C" int mcp23009_main(int argc, char *argv[])
{
	using ThisDriver = MCP23009;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = 0x25;

	MCP230XX_config_t mcp_config{};
	uint16_t device_type = DRV_GPIO_DEVTYPE_MCP23009;
	const char *name = "MCP23009";

	int ch;

	while ((ch = cli.getOpt(argc, argv, "D:O:P:U:R:M:")) != EOF) {
		switch (ch) {

		case 'D':
			mcp_config.direction = (int)strtol(cli.optArg(), nullptr, 0);
			break;

		case 'O':
			mcp_config.state = (int)strtol(cli.optArg(), nullptr, 0);
			break;

		case 'P':
			mcp_config.pullup = (int)strtol(cli.optArg(), nullptr, 0);
			break;

		case 'U':
			mcp_config.interval = (uint16_t)atoi(cli.optArg());
			break;

		case 'M':
			mcp_config.first_minor = (uint8_t)atoi(cli.optArg());
			break;
		}
	}

	mcp_config.i2c_bus = cli.requested_bus;
	mcp_config.i2c_addr = cli.i2c_address;
	mcp_config.device_type = device_type;

	const char *verb = cli.optArg();

	if (!verb) {
		MCP230XX::print_usage();
		return -1;
	}

	cli.custom_data = &mcp_config;

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

	MCP230XX::print_usage();
	return -1;
}
