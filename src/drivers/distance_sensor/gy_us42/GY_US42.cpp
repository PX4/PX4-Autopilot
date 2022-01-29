/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "GY_US42.hpp"

GY_US42::GY_US42(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_rangefinder(get_device_id(), config.rotation)
{
	_px4_rangefinder.set_max_distance(GY_US42_MAX_DISTANCE);
	_px4_rangefinder.set_min_distance(GY_US42_MIN_DISTANCE);
}

GY_US42::~GY_US42()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int GY_US42::init()
{
	// I2C init (and probe) first.
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	_state = STATE::POWERON_WAIT;
	ScheduleDelayed(2000);
	return OK;
}

int GY_US42::collect()
{
	// Read from the sensor.
	uint8_t val[2] = {};
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = transfer(nullptr, 0, val, 2);

	if (ret < 0) {
		PX4_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t distance_cm = val[0] << 8 | val[1];
	float distance_m = float(distance_cm) * 1e-2f;

	_px4_rangefinder.update(timestamp_sample, distance_m);

	perf_end(_sample_perf);
	return PX4_OK;
}

int GY_US42::measure()
{
	uint8_t cmd[1] = {GY_US42_TAKE_RANGE_REG};

	// Send the command to begin a measurement.
	int ret = transfer(cmd, 1, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
}

void GY_US42::RunImpl()
{
	switch (_state) {
	case STATE::INIT:
		// do nothing
		break;

	case STATE::POWERON_WAIT:
		measure();
		_state = STATE::MEASURE_WAIT;
		ScheduleOnInterval(GY_US42_CONVERSION_INTERVAL, GY_US42_CONVERSION_INTERVAL);
		break;

	case STATE::MEASURE_WAIT:
		collect();
		measure();
		// forever loop
		break;

	case STATE::MODIFYADDR_WAIT:
		break;
	}
}

void GY_US42::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
