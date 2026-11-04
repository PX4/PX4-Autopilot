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


#include "OpticalFlowAccumulator.hpp"

namespace sensors
{

using namespace matrix;

OpticalFlowAccumulator::OpticalFlowAccumulator()
{
	_gyro_integrator.set_reset_samples(1);
}

void OpticalFlowAccumulator::addGyroSample(uint64_t timestamp_sample, const Vector3f &angular_velocity, float dt)
{
	GyroSample sample;
	sample.time_us = timestamp_sample;
	sample.data = angular_velocity;
	sample.dt = dt;

	_gyro_buffer.push(sample);
}

bool OpticalFlowAccumulator::addFrame(const sensor_optical_flow_s &flow)
{
	bool discarded = false;

	// clear data accumulation if there's a gap in data
	const uint64_t integration_gap_threshold_us = flow.integration_timespan_us * 2;

	if ((flow.timestamp_sample >= _timestamp_sample_last + integration_gap_threshold_us)
	    || (_frame_count > 0 && (flow.quality > 0) && _quality_sum == 0)) {

		reset();
		discarded = true;
	}

	// quality 0 marks a frame the sensor rejected: it consumed its window but carries no flow
	const bool rejected = (flow.quality == 0);

	const uint64_t timestamp_oldest = flow.timestamp_sample - flow.integration_timespan_us;
	// never integrate past the end of the flow window; the rest belongs to the next frame
	const uint64_t timestamp_newest = flow.timestamp_sample;

	// delta angle from the sensor if available, otherwise from the buffered gyro over the same window
	if (flow.delta_angle_available && Vector2f(flow.delta_angle).isAllFinite()) {
		Vector3f delta_angle(flow.delta_angle);

		if (!PX4_ISFINITE(delta_angle(2))) {
			// Some sensors only provide X and Y angular rates, rotate them but place back the NAN on the Z axis
			delta_angle(2) = 0.f;

			if (!rejected) {
				_delta_angle += _sensor_rotation * delta_angle;
			}

			_delta_angle(2) = NAN;

		} else if (!rejected) {
			_delta_angle += _sensor_rotation * delta_angle;
		}

		_delta_angle_from_sensor = true;

	} else {
		_delta_angle_from_sensor = false;

		GyroSample gyro_sample;

		while (_gyro_buffer.pop_oldest(timestamp_oldest, timestamp_newest, &gyro_sample)) {

			_gyro_integrator.put(gyro_sample.data, gyro_sample.dt);

			const float min_interval_s = (flow.integration_timespan_us * 1e-6f) * 0.99f;

			if (_gyro_integrator.integral_dt() > min_interval_s) {
				break;
			}
		}

		Vector3f delta_angle{NAN, NAN, NAN};
		uint32_t delta_angle_dt;

		if (_gyro_integrator.reset(delta_angle, delta_angle_dt)) {
			if (!rejected) {
				_delta_angle += delta_angle;
			}

		} else {
			// force integrator reset
			_gyro_integrator.reset();
		}
	}

	_timestamp_sample_last = flow.timestamp_sample;

	if (rejected) {
		// keep the time so the publication cadence holds, but never let a blind
		// frame's zeros dilute the flow of the good frames around it
		_rejected_timespan_us += flow.integration_timespan_us;

	} else {
		_flow_integral(0) += flow.pixel_flow[0];
		_flow_integral(1) += flow.pixel_flow[1];

		_integration_timespan_us += flow.integration_timespan_us;

		_quality_sum += flow.quality;
		_frame_count++;
	}

	return discarded;
}

OpticalFlowAccumulator::Window OpticalFlowAccumulator::window() const
{
	Window window{};
	window.pixel_flow = _flow_integral;
	window.delta_angle = _delta_angle;
	window.frame_count = _frame_count;

	if (_frame_count > 0) {
		window.integration_timespan_us = _integration_timespan_us;

		// blind frames already left the flow and the timespan; the quality of what
		// remains is the mean quality of the frames that produced it
		window.quality = static_cast<uint8_t>(_quality_sum / _frame_count);

	} else {
		// every frame in the window was rejected: report the window blind
		window.integration_timespan_us = _rejected_timespan_us;
		window.quality = 0;
	}

	return window;
}

void OpticalFlowAccumulator::reset()
{
	_flow_integral.zero();
	_integration_timespan_us = 0;
	_rejected_timespan_us = 0;

	_delta_angle.zero();

	_quality_sum = 0;
	_frame_count = 0;

	_gyro_integrator.reset();
}

} // namespace sensors
