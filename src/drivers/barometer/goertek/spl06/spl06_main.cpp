/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include "SPL06.hpp"

#include <drivers/drv_sensor.h>

extern "C" { __EXPORT int spl06_main(int argc, char *argv[]); }

void
SPL06::print_usage()
{
	PRINT_MODULE_USAGE_NAME("spl06", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("baro");
	PRINT_MODULE_USAGE_COMMAND("start");
#if defined(CONFIG_I2C)
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, true);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x76);
#else
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
#endif
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *SPL06::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	spl06::ISPL06 *interface = nullptr;

#if defined(CONFIG_I2C)

	if (config.bus_type == BOARD_I2C_BUS) {
		interface = spl06_i2c_interface(config.bus, config.i2c_address, config.bus_frequency);

	}

#endif // CONFIG_I2C
#if defined(CONFIG_SPI)

	if (config.bus_type == BOARD_SPI_BUS) {
		interface = spl06_spi_interface(config.bus, config.spi_devid, config.bus_frequency, config.spi_mode);
	}

#endif // CONFIG_SPI

	if (interface == nullptr) {
		PX4_ERR("failed creating interface for bus %i", config.bus);
		return nullptr;
	}

	if (interface->init() != OK) {
		delete interface;
		PX4_DEBUG("no device on bus %i", config.bus);
		return nullptr;
	}

	SPL06 *dev = new SPL06(config, interface);

	if (dev == nullptr) {
		delete interface;
		return nullptr;
	}

	if (OK != dev->init()) {
		delete dev;
		return nullptr;
	}

	return dev;
}

int
spl06_main(int argc, char *argv[])
{
	using ThisDriver = SPL06;
	BusCLIArguments cli{true, true};
#if defined(CONFIG_I2C)
	cli.i2c_address = 0x76;
	cli.default_i2c_frequency = 100 * 1000;
#endif // CONFIG_I2C
#if defined(CONFIG_SPI)
	cli.default_spi_frequency = 10 * 1000 * 1000;
#endif // CONFIG_SPI

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_BARO_DEVTYPE_SPL06);

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
