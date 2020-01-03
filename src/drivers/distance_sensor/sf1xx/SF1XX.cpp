/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "SF1XX.hpp"

#include <lib/parameters/param.h>

using namespace time_literals;

/* Configuration Constants */
#define SF1XX_TAKE_RANGE_REG		0

SF1XX::SF1XX(int bus, int address, uint8_t rotation) :
	I2C("SF1XX", nullptr, bus, address, 400000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_rangefinder(0 /* device id not yet used */, ORB_PRIO_DEFAULT, rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
}

SF1XX::~SF1XX()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int SF1XX::init()
{
	int32_t hw_model = 0;
	param_get(param_find("SENS_EN_SF1XX"), &hw_model);

	switch (hw_model) {
	case 1:
		/* SF10/a (25m 32Hz) */
		_px4_rangefinder.set_min_distance(0.01f);
		_px4_rangefinder.set_max_distance(25.0f);
		_interval = 31250;
		break;

	case 2:
		/* SF10/b (50m 32Hz) */
		_px4_rangefinder.set_min_distance(0.01f);
		_px4_rangefinder.set_max_distance(50.0f);
		_interval = 31250;
		break;

	case 3:
		/* SF10/c (100m 16Hz) */
		_px4_rangefinder.set_min_distance(0.01f);
		_px4_rangefinder.set_max_distance(100.0f);
		_interval = 62500;
		break;

	case 4:
		/* SF11/c (120m 20Hz) */
		_px4_rangefinder.set_min_distance(0.01f);
		_px4_rangefinder.set_max_distance(120.0f);
		_interval = 50000;
		break;

	case 5:
		/* SF/LW20/b (50m 48-388Hz) */
		_px4_rangefinder.set_min_distance(0.001f);
		_px4_rangefinder.set_max_distance(50.0f);
		_interval = 20834;
		break;

	case 6:
		/* SF/LW20/c (100m 48-388Hz) */
		_px4_rangefinder.set_min_distance(0.001f);
		_px4_rangefinder.set_max_distance(100.0f);
		_interval = 20834;
		break;

	default:
		PX4_ERR("invalid HW model %d.", hw_model);
		return -1;
	}

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	return PX4_OK;
}

int SF1XX::measure()
{
	// Send the command to begin a measurement.
	uint8_t cmd = SF1XX_TAKE_RANGE_REG;
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
}

int SF1XX::collect()
{
	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* read from the sensor (i2c) */
	uint8_t val[2] {};
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		PX4_DEBUG("read err: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

		/* only throw an error if we time out */
		if (read_elapsed > (_interval * 2)) {
			return ret;

		} else {
			return -EAGAIN;
		}

	} else if (ret == 0) {
		return -EAGAIN;
	}

	_last_read = hrt_absolute_time();

	uint16_t distance_cm = val[0] << 8 | val[1];
	float distance_m = float(distance_cm) * 1e-2f;

	_px4_rangefinder.update(timestamp_sample, distance_m);

	perf_end(_sample_perf);

	return PX4_OK;
}

void SF1XX::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void SF1XX::stop()
{
	ScheduleClear();
}

void SF1XX::Run()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		int collect_ret = collect();

		if (collect_ret == -EAGAIN) {
			/* reschedule to grab the missing bits, time to transmit 8 bytes @ 9600 bps */
			ScheduleDelayed(1042 * 8);

			return;
		}

		if (OK != collect_ret) {

			/* we know the sensor needs about four seconds to initialize */
			if (hrt_absolute_time() > 5_s && _consecutive_fail_count < 5) {
				PX4_ERR("collection error #%u", _consecutive_fail_count);
			}

			_consecutive_fail_count++;

			/* restart the measurement state machine */
			start();
			return;

		} else {
			/* apparently success */
			_consecutive_fail_count = 0;
		}

		/* next phase is measurement */
		_collect_phase = false;
	}

	/* measurement phase */
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(_interval);
}

void SF1XX::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	_px4_rangefinder.print_status();
}
