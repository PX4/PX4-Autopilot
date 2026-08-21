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
#include <aid_sources/optical_flow/optical_flow.hpp>

#if defined(CONFIG_EKF2_OPTICAL_FLOW) && defined(MODULE_NAME)

#include <float.h>

using namespace time_literals;
using matrix::Vector2f;
using matrix::Vector3f;

void OpticalFlow::initParameters(Ekf &ekf)
{
	float delay_max_ms = 110.f;
	int32_t predict_us = 10000;
	param_get(param_find("EKF2_DELAY_MAX"), &delay_max_ms);
	param_get(param_find("EKF2_PREDICT_US"), &predict_us);
	const uint8_t buffer_length = math::constrain((int)ceilf(delay_max_ms / (predict_us * 1e-3f)), 2, UINT8_MAX);

	for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
		char param_name[20] {};

		snprintf(param_name, sizeof(param_name), "EKF2_OF%d_CTRL", i);
		_param_handles[i].ctrl = param_find(param_name);

		int32_t ctrl = 0;

		if (_param_handles[i].ctrl != PARAM_INVALID) {
			param_get(_param_handles[i].ctrl, &ctrl);
		}

		// only resolve the remaining handles of enabled slots so that a disabled slot's
		// parameters stay hidden from the GCS
		if (ctrl != 0) {
			resolveTuningHandles(i);

			if (!ekf.flowSource(i).allocate(buffer_length)) {
				PX4_ERR("optical flow %d buffer allocation failed", i);
			}
		}
	}

	updateParameters(ekf);
}

float OpticalFlow::maxEnabledDelayMs(Ekf &ekf) const
{
	float delay_max_ms = 0.f;

	for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
		float delay_ms = 0.f;

		if ((ekf.flowSource(i).params.ctrl != 0) && (_param_handles[i].delay != PARAM_INVALID)
		    && (param_get(_param_handles[i].delay, &delay_ms) == PX4_OK)) {
			delay_max_ms = math::max(delay_max_ms, delay_ms);
		}
	}

	return delay_max_ms;
}

void OpticalFlow::resolveTuningHandles(const uint8_t i)
{
	char param_name[20] {};

	snprintf(param_name, sizeof(param_name), "SENS_FLOW%d_DELAY", i);
	_param_handles[i].delay = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "EKF2_OF%d_GYR_SRC", i);
	_param_handles[i].gyr_src = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "EKF2_OF%d_N_MIN", i);
	_param_handles[i].n_min = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "EKF2_OF%d_N_MAX", i);
	_param_handles[i].n_max = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "EKF2_OF%d_QMIN", i);
	_param_handles[i].qmin = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "EKF2_OF%d_QMINGND", i);
	_param_handles[i].qmin_gnd = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "EKF2_OF%d_GATE", i);
	_param_handles[i].gate = param_find(param_name);
}

void OpticalFlow::updateParameters(Ekf &ekf)
{
	for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
		if (_param_handles[i].ctrl == PARAM_INVALID) {
			continue;
		}

		OpticalFlowSource::Params &params = ekf.flowSource(i).params;

		param_get(_param_handles[i].ctrl, &params.ctrl);

		if (_param_handles[i].gyr_src == PARAM_INVALID) {
			continue;
		}

		param_get(_param_handles[i].gyr_src, &params.gyr_src);
		param_get(_param_handles[i].n_min, &params.n_min);
		param_get(_param_handles[i].n_max, &params.n_max);
		param_get(_param_handles[i].qmin, &params.qmin);
		param_get(_param_handles[i].qmin_gnd, &params.qmin_gnd);
		param_get(_param_handles[i].gate, &params.gate);
	}
}

void OpticalFlow::advertiseEnabledPublications(const Ekf &ekf)
{
	for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
		if (ekf.flowSource(i).params.ctrl) {
			_estimator_optical_flow_vel_pub[i].advertise();
			_estimator_aid_src_optical_flow_pub[i].advertise();
		}
	}
}

int OpticalFlow::lowestConfiguredSlot(const Ekf &ekf) const
{
	for (uint8_t slot = 0; slot < MAX_OF_INSTANCES; slot++) {
		if (ekf.flowSource(slot).params.ctrl != 0) {
			return slot;
		}
	}

	return -1;
}

void OpticalFlow::updateSamples(Ekf &ekf, ekf2_timestamps_s &ekf2_timestamps,
				const hrt_abstime &last_range_sensor_update)
{
	for (uint8_t instance = 0; instance < MAX_OF_INSTANCES; instance++) {
		vehicle_optical_flow_s optical_flow;

		if (!_vehicle_optical_flow_subs[instance].update(&optical_flow)) {
			continue;
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

		flowSample flow {
			.time_us = optical_flow.timestamp_sample - optical_flow.integration_timespan_us / 2, // correct timestamp to midpoint of integration interval as the data is converted to rates
			.flow_rate = flow_rate,
			.gyro_rate = gyro_rate,
			.quality = optical_flow.quality,
			.device_id = optical_flow.device_id
		};

		if (Vector2f(optical_flow.pixel_flow).isAllFinite() && optical_flow.integration_timespan_us < 1e6) {

			ekf.set_optical_flow_limits(optical_flow.max_flow_rate, optical_flow.min_ground_distance,
						    optical_flow.max_ground_distance, instance);
			ekf.flowSource(instance).setPositionBody(Vector3f(optical_flow.position_offset));
			ekf.setOpticalFlowData(flow, instance);
		}

#if defined(CONFIG_EKF2_RANGE_FINDER)

		if ((instance == lowestConfiguredSlot(ekf))
		    && PX4_ISFINITE(optical_flow.distance_m) && (ekf2_timestamps.timestamp > last_range_sensor_update + 1_s)) {

			int8_t quality = static_cast<float>(optical_flow.quality) / static_cast<float>(UINT8_MAX) * 100.f;

			estimator::sensor::rangeSample range_sample {
				.time_us = optical_flow.timestamp_sample,
				.rng = optical_flow.distance_m,
				.quality = quality,
			};
			ekf.setRangeData(range_sample);
			ekf.set_rangefinder_limits(optical_flow.min_ground_distance, optical_flow.max_ground_distance);
		}

#endif // CONFIG_EKF2_RANGE_FINDER

		ekf2_timestamps.optical_flow_timestamp_rel = (int16_t)((int64_t)optical_flow.timestamp / 100 -
				(int64_t)ekf2_timestamps.timestamp / 100);
	}
}

void OpticalFlow::publishAidSourceStatus(const Ekf &ekf, const hrt_abstime &timestamp,
		const uint8_t estimator_instance, const bool replay_mode)
{
	for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
		const estimator_aid_source2d_s &status = ekf.aid_src_optical_flow(i);

		if (status.timestamp_sample > _status_pub_last[i]) {
			estimator_aid_source2d_s status_out{status};
			status_out.estimator_instance = estimator_instance;
			status_out.timestamp = replay_mode ? timestamp : hrt_absolute_time();
			_estimator_aid_src_optical_flow_pub[i].publish(status_out);
			_status_pub_last[i] = status.timestamp_sample;
		}
	}
}

void OpticalFlow::publishFlowVel(const Ekf &ekf, const hrt_abstime &timestamp, const bool replay_mode)
{
	for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
		const hrt_abstime timestamp_sample = ekf.aid_src_optical_flow(i).timestamp_sample;

		if ((timestamp_sample != 0) && (timestamp_sample > _flow_vel_pub_last[i])) {

			vehicle_optical_flow_vel_s flow_vel{};
			flow_vel.timestamp_sample = ekf.aid_src_optical_flow(i).timestamp_sample;

			ekf.getFlowVelBody(i).copyTo(flow_vel.vel_body);
			ekf.getFlowVelNE(i).copyTo(flow_vel.vel_ne);

			ekf.getFilteredFlowVelBody(i).copyTo(flow_vel.vel_body_filtered);
			ekf.getFilteredFlowVelNE(i).copyTo(flow_vel.vel_ne_filtered);

			ekf.getFlowUncompensated(i).copyTo(flow_vel.flow_rate_uncompensated);
			ekf.getFlowCompensated(i).copyTo(flow_vel.flow_rate_compensated);

			ekf.getFlowGyro(i).copyTo(flow_vel.gyro_rate);

			ekf.getFlowGyroBias(i).copyTo(flow_vel.gyro_bias);
			ekf.getFlowRefBodyRate().copyTo(flow_vel.ref_gyro);

			flow_vel.timestamp = replay_mode ? timestamp : hrt_absolute_time();

			_estimator_optical_flow_vel_pub[i].publish(flow_vel);

			_flow_vel_pub_last[i] = timestamp_sample;
		}
	}
}

#endif // CONFIG_EKF2_OPTICAL_FLOW && MODULE_NAME
