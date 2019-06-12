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

#include "MS4525.hpp"

/* Measurement rate is 100Hz */
static constexpr int MEAS_RATE{100};
static constexpr uint32_t CONVERSION_INTERVAL{1000000 / MEAS_RATE};

MS4525::MS4525(uint8_t bus, uint8_t address) :
	I2C(MODULE_NAME, nullptr, bus, address, 400000),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_diff_press(get_device_id())
{
	_px4_diff_press.set_device_type(DRV_DIFF_PRESS_DEVTYPE_MS4525);
}

void
MS4525::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void
MS4525::stop()
{
	ScheduleClear();
}

int
MS4525::measure()
{
	// Send the command to begin a measurement.
	uint8_t cmd = 0;
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return ret;
}

int
MS4525::collect()
{
	perf_begin(_sample_perf);

	// read from the sensor
	uint8_t val[4] {};

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int ret = transfer(nullptr, 0, &val[0], 4);

	if (ret != PX4_OK) {
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

	int16_t dp_raw = 0, dT_raw = 0;
	dp_raw = (val[0] << 8) + val[1];
	/* mask the used bits */
	dp_raw = 0x3FFF & dp_raw;
	dT_raw = (val[2] << 8) + val[3];
	dT_raw = (0xFFE0 & dT_raw) >> 5;

	// dT max is almost certainly an invalid reading
	if (dT_raw == 2047) {
		perf_count(_comms_errors);
		return -EAGAIN;
	}

	float temperature = ((200.0f * dT_raw) / 2047) - 50;

	// Calculate differential pressure. As its centered around 8000
	// and can go positive or negative
	const float P_min = -1.0f;
	const float P_max = 1.0f;
	const float PSI_to_Pa = 6894.757f;
	/*
	  this equation is an inversion of the equation in the
	  pressure transfer function figure on page 4 of the datasheet

	  We negate the result so that positive differential pressures
	  are generated when the bottom port is used as the static
	  port on the pitot and top port is used as the dynamic port
	 */
	float diff_press_PSI = -((dp_raw - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
	float diff_press_pa_raw = diff_press_PSI * PSI_to_Pa;

	/*
	  With the above calculation the MS4525 sensor will produce a
	  positive number when the top port is used as a dynamic port
	  and bottom port is used as the static port
	 */
	_px4_diff_press.set_error_count(perf_event_count(_comms_errors));
	_px4_diff_press.set_temperature(temperature);
	_px4_diff_press.update(timestamp_sample, diff_press_pa_raw);

	perf_end(_sample_perf);

	return ret;
}

void
MS4525::Run()
{
	int ret = PX4_ERROR;

	// collection phase
	if (_collect_phase) {
		// perform collection
		ret = collect();

		if (OK != ret) {
			/* restart the measurement state machine */
			start();
			_sensor_ok = false;
			return;
		}

		// next phase is measurement
		_collect_phase = false;
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
