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

#include "tmp102.h"

TMP102::TMP102(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
}

TMP102::~TMP102()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);
}

void TMP102::RunImpl()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	float temperature = read_temperature();

	if (std::isnan(temperature)) {
		perf_count(_comms_errors);

	} else {
		sensor_temp_s _sensor_temp{};
		_sensor_temp.timestamp = hrt_absolute_time();
		_sensor_temp.temperature = temperature;
		_sensor_temp.device_id = get_device_id();
		_sensor_temp_pub.publish(_sensor_temp);
	}

	perf_end(_cycle_perf);
}

int TMP102::probe()
{
	uint16_t conf_reg;

	for(int i=0; i<3; i++) {
		if (read_reg(TMP102_CONFIG_REG, conf_reg) == PX4_OK && conf_reg == DEFAULT_CONFIG_REG) {
			return PX4_OK;
		}
		px4_sleep(1);
	}

	return PX4_ERROR;
}

int TMP102::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("TMP102, I2C init failed");
		return ret;
	}

	_sensor_temp_pub.advertise();
	ScheduleOnInterval(250_ms); // DEFAULT SAMPLE RATE IS 4HZ => 250ms INTERVAL
	return PX4_OK;
}

float TMP102::read_temperature()
{
	uint16_t tmp_data;

	if (read_reg(TMP102_TEMP_REG, tmp_data) != PX4_OK) {
		return NAN;
	}

	float temperature = (tmp_data >> 4) * 0.0625f;
	return temperature;
}

int TMP102::read_reg(uint8_t reg, uint16_t &data)
{
	int ret = PX4_OK;
	uint8_t tmp_data[2];

	if (reg != _curr_pr) {
		tmp_data[0] = reg;

		if (transfer(tmp_data, 1, nullptr, 0) == PX4_OK) {
			_curr_pr = reg;
		}
	}

	ret |= transfer(nullptr, 0, tmp_data, 2); // Now read the data from the desired register
	data = (tmp_data[0] << 8) | tmp_data[1];
	return ret;
}
