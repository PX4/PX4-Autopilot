/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "EagleTreeAirspeed.hpp"

#include <float.h>

/* Register address */
static constexpr uint8_t READ_CMD{0x07}; // Read the data

/* Measurement rate is 100Hz */
static constexpr uint32_t CONVERSION_INTERVAL{1000000 / 100};	/* microseconds */

EagleTreeAirspeed::EagleTreeAirspeed(int bus, int address) :
	I2C(MODULE_NAME, nullptr, bus, address, 100000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_diff_press(get_device_id())
{
	_px4_diff_press.set_device_type(DRV_DIFF_PRESS_DEVTYPE_ETS3);
}

void
EagleTreeAirspeed::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
EagleTreeAirspeed::stop()
{
	ScheduleClear();
}

int
EagleTreeAirspeed::measure()
{
	// Send the command to begin a measurement.
	uint8_t cmd = READ_CMD;
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
EagleTreeAirspeed::collect()
{
	/* read from the sensor */
	uint8_t val[2] {};

	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	float diff_pres_pa_raw = (float)(val[1] << 8 | val[0]);

	if (diff_pres_pa_raw < FLT_EPSILON) {
		// a zero value indicates no measurement
		// since the noise floor has been arbitrarily killed
		// it defeats our stuck sensor detection - the best we
		// can do is to output some numerical noise to show
		// that we are still correctly sampling.
		diff_pres_pa_raw = 0.001f * (timestamp_sample & 0x01);
	}

	_px4_diff_press.set_error_count(perf_event_count(_comms_errors));
	_px4_diff_press.update(timestamp_sample, diff_pres_pa_raw);

	perf_end(_sample_perf);

	return ret;
}

void
EagleTreeAirspeed::Run()
{
	int ret;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (OK != ret) {
			perf_count(_comms_errors);
			/* restart the measurement state machine */
			start();
			_sensor_ok = false;
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;
	}

	/* measurement phase */
	ret = measure();

	if (OK != ret) {
		DEVICE_DEBUG("measure error");
	}

	_sensor_ok = (ret == OK);

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(CONVERSION_INTERVAL);
}
