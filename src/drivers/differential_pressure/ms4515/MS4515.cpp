/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
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

#include "MS4515.hpp"
#include <parameters/param.h>

MS4515::MS4515(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

MS4515::~MS4515()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int MS4515::probe()
{
	uint8_t cmd = 0;
	int ret = transfer(&cmd, 1, nullptr, 0);

	return ret;
}

int MS4515::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	if (ret == PX4_OK) {
		ScheduleNow();
	}

	return ret;
}

int MS4515::measure()
{
	// Send the command to begin a measurement.
	uint8_t cmd = 0;
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int MS4515::collect()
{
	/* read from the sensor */
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	uint8_t val[4] {};
	int ret = transfer(nullptr, 0, &val[0], 4);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint8_t status = (val[0] & 0xC0) >> 6;

	switch (status) {
	case 0:
		// Normal Operation. Good Data Packet
		break;

	case 1:
		// Reserved
		return -EAGAIN;

	case 2:
		// Stale Data. Data has been fetched since last measurement cycle.
		return -EAGAIN;

	case 3:
		// Fault Detected
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return -EAGAIN;
	}

	/* mask the used bits */
	int16_t dp_raw = (0x3FFF & ((val[0] << 8) + val[1]));
	int16_t dT_raw = (0xFFE0 & ((val[2] << 8) + val[3])) >> 5;

	// dT max is almost certainly an invalid reading
	if (dT_raw == 2047) {
		perf_count(_comms_errors);
		return -EAGAIN;
	}

	// only publish changes
	if ((dp_raw != _dp_raw_prev) || (dT_raw != _dT_raw_prev)) {

		_dp_raw_prev = dp_raw;
		_dT_raw_prev = dT_raw;

		float temperature_c = ((200.0f * dT_raw) / 2047) - 50;

		// Calculate differential pressure. As its centered around 8000
		// and can go positive or negative
		const float P_min = -1.0f;
		const float P_max = 1.0f;
		const float IN_H20_to_Pa = 248.84f;
		/*
		  this equation is an inversion of the equation in the
		  pressure transfer function figure on page 4 of the datasheet

		  We negate the result so that positive differential pressures
		  are generated when the bottom port is used as the static
		  port on the pitot and top port is used as the dynamic port
		 */
		float diff_press_PSI = -((dp_raw - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
		float diff_press_pa = diff_press_PSI * IN_H20_to_Pa;

		/*
		  With the above calculation the MS4515 sensor will produce a
		  positive number when the top port is used as a dynamic port
		  and bottom port is used as the static port
		 */
		differential_pressure_s differential_pressure{};
		differential_pressure.timestamp_sample = timestamp_sample;
		differential_pressure.device_id = get_device_id();
		differential_pressure.differential_pressure_pa = diff_press_pa;
		int32_t differential_press_rev = 0;
		param_get(param_find("SENS_DPRES_REV"), &differential_press_rev);

		//If differential pressure reverse param set, swap positive and negative
		if (differential_press_rev == 1) {
			differential_pressure.differential_pressure_pa = -1.0f * diff_press_pa;
		}

		differential_pressure.temperature = temperature_c;
		differential_pressure.error_count = perf_event_count(_comms_errors);
		differential_pressure.timestamp = hrt_absolute_time();
		_differential_pressure_pub.publish(differential_pressure);
	}

	perf_end(_sample_perf);

	return PX4_OK;
}

void MS4515::RunImpl()
{
	int ret = PX4_ERROR;

	// collection phase
	if (_collect_phase) {
		// perform collection
		ret = collect();

		if (OK != ret) {
			perf_count(_comms_errors);
			/* restart the measurement state machine */
			_collect_phase = false;
			_sensor_ok = false;
			ScheduleNow();
			return;
		}

		// next phase is measurement
		_collect_phase = false;

		// is there a collect->measure gap?
		if (_measure_interval > CONVERSION_INTERVAL) {

			// schedule a fresh cycle call when we are ready to measure again
			ScheduleDelayed(_measure_interval - CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	ret = measure();

	if (OK != ret) {
		DEVICE_DEBUG("measure error");
	}

	_sensor_ok = (ret == OK);

	// next phase is collection
	_collect_phase = true;

	// schedule a fresh cycle call when the measurement is done
	ScheduleDelayed(CONVERSION_INTERVAL);
}
