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

#include "TdkMpu9250.hpp"
#include <drivers/drv_sensor.h>

using namespace frequency_literals;
using namespace time_literals;
using namespace tdk_direct_registers;

void TdkMpu9250::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("type: %s", spiModel().name);

	imu::printSpiStatus(spiModel(),
			    _register_frequency,
			    _data_frequency,
			    get_frequency());

	PX4_INFO("FIFO empty interval: %u us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	_transfer_perf.print();
	_fifo_perf.print();
	_drdy_missed_perf.print();
}

void TdkMpu9250::print_usage()
{
	PRINT_MODULE_DESCRIPTION(R"DESCR(
SPI-only MPU9250 accelerometer and gyroscope driver.
The auxiliary magnetometer and I2C transport remain available in the original driver.
Without -T, only this exact model's board-registered SPI endpoints are selected.
An explicit -T must match mpu9250; stop/status use the native bus filters.
)DESCR");
	PRINT_MODULE_USAGE_NAME("mpu9250", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('T', nullptr, "mpu9250", "Optional exact model", true);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, ROTATION_MAX - 1, "Rotation", true);
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
}

extern "C" int mpu9250_main(int argc, char *argv[])
{
	return imu::spiSingleMain<TdkMpu9250>(argc, argv, MODULE_NAME);
}
