/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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


#include "PCT2075TP.hpp"

using namespace NXP_PCT2075TP;

PCT2075TP::PCT2075TP(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
}

PCT2075TP::~PCT2075TP()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int PCT2075TP::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_WARN("I2C::init failed (%i)", ret);
		return ret;
	}

	// Wake up the device
	if (_wake() != PX4_OK) {
		DEVICE_DEBUG("Failed to wake device");
		return -EIO;
	}

	// Wait a bit for device to stabilize
	px4_usleep(10000);

	// Verify device is responding by reading configuration back
	uint8_t config_readback = 0;

	if (_read_config(&config_readback) != PX4_OK) {
		DEVICE_DEBUG("Failed to read configuration");
		return -EIO;
	}

	ScheduleOnInterval(SAMPLING_INTERVAL_USEC);

	return PX4_OK;
}

void PCT2075TP::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	printf("Last temperature reading: %.2f C\n", (double)_sensor_temp.temperature);
}

int PCT2075TP::probe()
{
	uint8_t config = 0;

	if (_read_config(&config) != PX4_OK) {
		DEVICE_DEBUG("probe: failed to read config");
		return -EIO;
	}

	return PX4_OK;
}

void PCT2075TP::RunImpl()
{
	float temperature = 0.0f;

	perf_begin(_sample_perf);

	if (_read_sample(&temperature) != PX4_OK) {
		perf_count(_comms_errors);

	} else {
		_sensor_temp.device_id = get_device_id();
		_sensor_temp.temperature = temperature;
		_sensor_temp.timestamp = hrt_absolute_time();

		_sensor_temp_pub.publish(_sensor_temp);
	}

	perf_end(_sample_perf);
}

int PCT2075TP::_wake()
{
	// Wake up the device (clear shutdown bit) and set normal operation mode
	// All bits cleared = normal operation, comparator mode, active low
	return _write_config(0x00);
}

int PCT2075TP::_read_config(uint8_t *config)
{
	uint8_t cmd = Register::CONFIGURATION;
	return transfer(&cmd, 1, config, 1);
}

int PCT2075TP::_write_config(uint8_t config)
{
	uint8_t config_data[2];
	config_data[0] = Register::CONFIGURATION;
	config_data[1] = config;

	return transfer(config_data, sizeof(config_data), nullptr, 0);
}

int PCT2075TP::_read_sample(float *temperature)
{
	// Read temperature register (16-bit value)
	uint8_t temp_data[2];
	uint8_t cmd = Register::TEMPERATURE;

	int ret = transfer(&cmd, 1, temp_data, 2);

	if (ret != PX4_OK) {
		return ret;
	}

	// Combine bytes into 16-bit signed value
	int16_t raw_temp = (int16_t)((temp_data[0] << 8) | temp_data[1]);

	// Convert to temperature in Celsius
	// Temperature is 11-bit in 16-bit register
	// Formula: temp = raw * (0.125 / 32.0) = raw * 0.00390625
	*temperature = (float)raw_temp * TEMP_CONVERSION_FACTOR;

	return PX4_OK;
}
