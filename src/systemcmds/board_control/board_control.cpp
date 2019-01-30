/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <px4_config.h>
#include <px4_log.h>
#include <px4_module.h>

#include <drivers/device/device.h>
#include <drivers/device/i2c.h>

extern "C" __EXPORT int board_control_main(int argc, char *argv[]);

static int print_usage(const char *reason = nullptr);

int
board_control_main(int argc, char *argv[])
{
	const char *verb = argv[0];

	/* does not operate on a FMU instance */
	if (!strcmp(verb, "i2c")) {
		if (argc > 2) {
			int bus = strtol(argv[1], 0, 0);
			int clock_hz = strtol(argv[2], 0, 0);
			int ret = device::I2C::set_bus_clock(bus, clock_hz);

			if (ret) {
				PX4_ERR("setting I2C clock failed");
			}

			return ret;
		}

		return print_usage("not enough arguments");
	}

	if (!strcmp(verb, "sensor_reset")) {
		if (argc > 1) {
			int reset_time = strtol(argv[1], nullptr, 0);
			board_spi_reset(reset_time);

		} else {
			board_spi_reset(10);
			PX4_INFO("reset default time");
		}

		return 0;
	}

	if (!strcmp(verb, "peripheral_reset")) {
		if (argc > 2) {
			int reset_time = strtol(argv[2], 0, 0);
			board_peripheral_reset(reset_time);

		} else {
			board_peripheral_reset(10);
			PX4_INFO("reset default time");
		}

		return 0;
	}

	return print_usage();
}

static int print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("board control (sensor bus power, etc)");

	PRINT_MODULE_USAGE_NAME("board", "command");

	PRINT_MODULE_USAGE_COMMAND_DESCR("sensor_reset", "Do a sensor reset (SPI bus)");
	PRINT_MODULE_USAGE_ARG("<ms>", "Delay time in ms between reset and re-enabling", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("peripheral_reset", "Reset board peripherals");
	PRINT_MODULE_USAGE_ARG("<ms>", "Delay time in ms between reset and re-enabling", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("i2c", "Configure I2C clock rate");
	PRINT_MODULE_USAGE_ARG("<bus_id> <rate>", "Specify the bus id (>=0) and rate in Hz", false);

	return 0;
}
