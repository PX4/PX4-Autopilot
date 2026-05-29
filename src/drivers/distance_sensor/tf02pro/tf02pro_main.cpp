/****************************************************************************
 *
 *   Copyright (c) 2023-2026 PX4 Development Team. All rights reserved.
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
 * @file tf02pro_main.cpp
 *
 * Supports I2C (existing I2CSPIDriver path) and UART (new TF02PRO_UART path).
 * Interface selected by parameter SENS_EN_TF02PRO:
 *   0 = Disabled, 1 = I2C, 2 = UART
 */

#include "TF02PRO.hpp"
#include "TF02PRO_UART.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <lib/parameters/param.h>

namespace tf02pro
{

/* UART global instance */
TF02PRO_UART *g_dev_uart{nullptr};

int uart_start(const char *port, uint8_t rotation)
{
	if (g_dev_uart != nullptr) {
		PX4_ERR("UART driver already started");
		return PX4_OK;
	}

	g_dev_uart = new TF02PRO_UART(port, rotation);

	if (g_dev_uart == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	if (g_dev_uart->init() != PX4_OK) {
		PX4_ERR("UART init failed");
		delete g_dev_uart;
		g_dev_uart = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int uart_stop()
{
	if (g_dev_uart == nullptr) {
		PX4_ERR("UART driver not running");
		return 1;
	}

	delete g_dev_uart;
	g_dev_uart = nullptr;
	return PX4_OK;
}

int uart_status()
{
	if (g_dev_uart == nullptr) {
		PX4_ERR("UART driver not running");
		return 1;
	}

	g_dev_uart->print_info();
	return PX4_OK;
}

} // namespace tf02pro

void TF02PRO::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
TF02 Pro distance sensor driver.
Supports I2C and UART, selected by parameter SENS_EN_TF02PRO.

I2C mode (SENS_EN_TF02PRO=1):
  tf02pro start -X -a 0x10

UART mode (SENS_EN_TF02PRO=2, port via SENS_TF02PRO_CFG):
  tf02pro start -d /dev/ttyS3
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tf02pro", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x10);
	PRINT_MODULE_USAGE_PARAM_STRING('d', TF02PRO_DEFAULT_PORT, "<file:dev>",
					"Serial port (UART mode)", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation (25=downward)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int tf02pro_main(int argc, char *argv[])
{
	if (argc < 2) {
		TF02PRO::print_usage();
		return PX4_ERROR;
	}

	const char *verb = argv[1];

	// Administrative routing first: allow stop/status even if SENS_EN_TF02PRO was
	// changed to Disabled after a UART instance was already started.
	if (tf02pro::g_dev_uart != nullptr) {
		if (!strcmp(verb, "stop"))   { return tf02pro::uart_stop(); }
		if (!strcmp(verb, "status")) { return tf02pro::uart_status(); }
	}

	// Now read the interface selector
	int32_t iface = 0;
	param_get(param_find("SENS_EN_TF02PRO"), &iface);

	if (iface == 0) {
		if (!strcmp(verb, "start")) {
			PX4_WARN("Driver disabled via SENS_EN_TF02PRO parameter.");
		}
		return PX4_OK;
	}

	int         ch;
	uint8_t     rotation  = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *dev_path  = TF02PRO_DEFAULT_PORT;
	int         myoptind  = 1;
	const char *myoptarg  = nullptr;

	if (iface == 2) {
		/* ---- UART path: px4_getopt + g_dev pattern (TFMINI style) ---- */
		while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'R':
				rotation = (uint8_t)atoi(myoptarg);
				break;
			case 'd':
				dev_path = myoptarg;
				break;
			default:
				PX4_WARN("Unknown option!");
				return PX4_ERROR;
			}
		}

		if (myoptind >= argc) {
			TF02PRO::print_usage();
			return -1;
		}

		const char *uart_verb = argv[myoptind];

		if (!strcmp(uart_verb, "start"))  { return tf02pro::uart_start(dev_path, rotation); }
		if (!strcmp(uart_verb, "stop"))   { return tf02pro::uart_stop(); }
		if (!strcmp(uart_verb, "status")) { return tf02pro::uart_status(); }

		TF02PRO::print_usage();
		return -1;

	} else {
#if defined(CONFIG_I2C)
		/* ---- I2C path: existing BusCLIArguments + I2CSPIDriver::module_start() ---- */
		using ThisDriver = TF02PRO;
		BusCLIArguments cli{true, false};
		cli.rotation              = (Rotation)distance_sensor_s::ROTATION_DOWNWARD_FACING;
		cli.default_i2c_frequency = 400000;
		cli.i2c_address           = TF02PRO_BASEADDR;

		while ((ch = cli.getOpt(argc, argv, "R:d:")) != EOF) {
			switch (ch) {
			case 'R':
				cli.rotation = (Rotation)atoi(cli.optArg());
				break;
			case 'd':
				/* ignored in I2C mode */
				break;
			}
		}

		const char *i2c_verb = cli.optArg();

		if (!i2c_verb) {
			ThisDriver::print_usage();
			return -1;
		}

		BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_TF02PRO);

		if (!strcmp(i2c_verb, "start"))  { return ThisDriver::module_start(cli, iterator); }
		if (!strcmp(i2c_verb, "stop"))   { return ThisDriver::module_stop(iterator); }
		if (!strcmp(i2c_verb, "status")) { return ThisDriver::module_status(iterator); }

		ThisDriver::print_usage();
		return -1;
#else
		PX4_ERR("I2C mode not supported on this platform.");
		return -1;
#endif
	}
}
