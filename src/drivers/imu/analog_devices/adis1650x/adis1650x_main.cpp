/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "ADIS1650x.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

void ADIS1650x::print_usage()
{
	PRINT_MODULE_USAGE_NAME("adis1650x", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 1, 0, 1999,
				     "Decimation rate: sample_rate = 2000/(dec_rate+1). 0=2000Hz 1=1000Hz 3=500Hz",
				     true);
	PRINT_MODULE_USAGE_PARAM_INT('F', 0, 0, 6,
				     "Hardware Bartlett FIR filter tap size: 0=bypass 1-6=increasing strength",
				     true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int adis1650x_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = ADIS1650x;
	BusCLIArguments cli{false, true};
	cli.default_spi_frequency = SPI_SPEED;

	int dec_rate  = 0;   // default 2000 Hz
	int filt_size = 0;

	while ((ch = cli.getOpt(argc, argv, "R:r:F:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = static_cast<enum Rotation>(atoi(cli.optArg()));
			break;

		case 'r':
			dec_rate = atoi(cli.optArg());

			if (dec_rate < 0 || dec_rate > 1999) {
				PX4_ERR("Invalid dec_rate %d (valid: 0-1999)", dec_rate);
				return -1;
			}

			break;

		case 'F':
			filt_size = atoi(cli.optArg());

			if (filt_size < 0 || filt_size > 6) {
				PX4_ERR("Invalid filt_size %d (valid: 0-6)", filt_size);
				return -1;
			}

			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	ThisDriver::selected_dec_rate  = dec_rate;
	ThisDriver::selected_filt_size = filt_size;

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_IMU_DEVTYPE_ADIS1650X);

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
