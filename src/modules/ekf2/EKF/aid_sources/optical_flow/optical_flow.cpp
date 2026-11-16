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

#include "ekf.h"

#if defined(CONFIG_EKF2_OPTICAL_FLOW) && defined(MODULE_NAME)

#include <float.h>

using namespace time_literals;
using matrix::Vector2f;
using matrix::Vector3f;

void OpticalFlowSource::initParams()
{
	char param_name[20] {};
	snprintf(param_name, sizeof(param_name), "EKF2_OF%d_CTRL", _slot);
	_param_handles.ctrl = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "SENS_FLOW%d_DELAY", _slot);
	_param_handles.delay = param_find(param_name);

	updateParams();
}

void OpticalFlowSource::updateParams()
{
	if (_param_handles.ctrl == PARAM_INVALID) {
		return;
	}

	param_get(_param_handles.ctrl, &params.ctrl);

	// the tuning of a disabled slot stays hidden from the GCS until the slot is enabled
	if (params.ctrl == 0) {
		return;
	}

	if (_param_handles.gyr_src == PARAM_INVALID) {
		char param_name[20] {};

		snprintf(param_name, sizeof(param_name), "EKF2_OF%d_GYR_SRC", _slot);
		_param_handles.gyr_src = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OF%d_N_MIN", _slot);
		_param_handles.n_min = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OF%d_N_MAX", _slot);
		_param_handles.n_max = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OF%d_QMIN", _slot);
		_param_handles.qmin = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OF%d_QMINGND", _slot);
		_param_handles.qmin_gnd = param_find(param_name);

		snprintf(param_name, sizeof(param_name), "EKF2_OF%d_GATE", _slot);
		_param_handles.gate = param_find(param_name);
	}

	param_get(_param_handles.gyr_src, &params.gyr_src);
	param_get(_param_handles.n_min, &params.n_min);
	param_get(_param_handles.n_max, &params.n_max);
	param_get(_param_handles.qmin, &params.qmin);
	param_get(_param_handles.qmin_gnd, &params.qmin_gnd);
	param_get(_param_handles.gate, &params.gate);
}

float OpticalFlowSource::delayMs() const
{
	// read live, EKF2 checks the delays before the EKF parameters are updated
	int32_t ctrl = 0;
	float delay_ms = 0.f;

	if ((param_get(_param_handles.ctrl, &ctrl) == PX4_OK) && (ctrl != 0)) {
		param_get(_param_handles.delay, &delay_ms);
	}

	return delay_ms;
}

bool OpticalFlowSource::updateSample(Ekf &ekf, vehicle_optical_flow_s &optical_flow)
{
	if (!_sub.update(&optical_flow)) {
		return false;
	}

	const float dt = 1e-6f * (float)optical_flow.integration_timespan_us;
	Vector2f flow_rate;
	Vector3f gyro_rate;

	if (dt > FLT_EPSILON) {
		// NOTE: the EKF uses the reverse sign convention to the flow sensor. EKF assumes positive LOS rate
		// is produced by a RH rotation of the image about the sensor axis.
		flow_rate = Vector2f(-optical_flow.pixel_flow[0], -optical_flow.pixel_flow[1]) / dt;
		gyro_rate = Vector3f(-optical_flow.delta_angle[0], -optical_flow.delta_angle[1], -optical_flow.delta_angle[2]) / dt;

	} else if (optical_flow.quality == 0) {
		// handle special case of SITL and PX4Flow where dt is forced to zero when the quaity is 0
		flow_rate.zero();
		gyro_rate.zero();
	}

	estimator::flowSample flow {
		.time_us = optical_flow.timestamp_sample - optical_flow.integration_timespan_us / 2, // correct timestamp to midpoint of integration interval as the data is converted to rates
		.flow_rate = flow_rate,
		.gyro_rate = gyro_rate,
		.quality = optical_flow.quality,
		.device_id = optical_flow.device_id
	};

	if (Vector2f(optical_flow.pixel_flow).isAllFinite() && optical_flow.integration_timespan_us < 1e6) {
		setLimits(optical_flow.max_flow_rate, optical_flow.min_ground_distance, optical_flow.max_ground_distance);
		setPositionBody(Vector3f(optical_flow.position_offset));
		setData(flow, ekf._imu_buffer_length, ekf._min_obs_interval_us, ekf._dt_ekf_avg);
	}

	return true;
}

void OpticalFlowAiding::updateParams()
{
	for (auto &source : _sources) {
		source.updateParams();
	}
}

void OpticalFlowAiding::updateSamples(Ekf &ekf)
{
	for (uint8_t slot = 0; slot < estimator::MAX_OF_INSTANCES; slot++) {
		vehicle_optical_flow_s optical_flow;

		if (!_sources[slot].updateSample(ekf, optical_flow)) {
			continue;
		}

#if defined(CONFIG_EKF2_RANGE_FINDER)

		if ((_range_instance < 0) && PX4_ISFINITE(optical_flow.distance_m) && (optical_flow.distance_m > 0.f)) {
			_range_instance = slot;
		}

		// the distance measured by the flow sensor substitutes a missing range finder
		if ((slot == _range_instance) && PX4_ISFINITE(optical_flow.distance_m)
		    && !ekf.isNewestSampleRecent(ekf._time_last_range_sensor_data, 1_s)) {

			int8_t quality = static_cast<float>(optical_flow.quality) / static_cast<float>(UINT8_MAX) * 100.f;

			estimator::sensor::rangeSample range_sample {
				.time_us = optical_flow.timestamp_sample,
				.rng = optical_flow.distance_m,
				.quality = quality,
			};
			ekf.pushRangeData(range_sample);
			ekf.set_rangefinder_limits(optical_flow.min_ground_distance, optical_flow.max_ground_distance);
		}

#endif // CONFIG_EKF2_RANGE_FINDER
	}
}

#endif // CONFIG_EKF2_OPTICAL_FLOW && MODULE_NAME
