/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

/**
 * @file optical_flow_control.cpp
 * Control functions for optical flow fusion
 */

#include "ekf.h"

#include <ekf_derivation/generated/compute_flow_xy_innov_var_and_hx.h>

void OpticalFlowAiding::update(Ekf &ekf, const imuSample &imu_delayed)
{
	bool any_ctrl_enabled = false;

	for (uint8_t slot = 0; slot < MAX_OF_INSTANCES; slot++) {
		any_ctrl_enabled |= (_sources[slot].params.ctrl != 0);
	}

	ekf._fc.of.available = any_ctrl_enabled;

	for (uint8_t slot = 0; slot < MAX_OF_INSTANCES; slot++) {
		bool other_slot_fusing = false;

		for (uint8_t other = 0; other < MAX_OF_INSTANCES; other++) {
			other_slot_fusing |= (other != slot) && _sources[other].isFusing(ekf);
		}

		_sources[slot].update(ekf, imu_delayed, other_slot_fusing);
	}

	bool any_active = false;
	bool any_terrain = false;

	for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
		any_active |= _sources[i]._active;
		any_terrain |= _sources[i]._terrain;
	}

	ekf._control_status.flags.opt_flow = any_active;
	ekf._control_status.flags.opt_flow_terrain = any_terrain;

	if (!any_active) {
		ekf._fault_status.flags.bad_optflow_X = false;
		ekf._fault_status.flags.bad_optflow_Y = false;
	}
}

uint8_t OpticalFlowAiding::primarySlot() const
{
	for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
		if (_sources[i]._active) {
			return i;
		}
	}

	for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
		if (_sources[i]._aid_src.timestamp_sample != 0) {
			return i;
		}
	}

	return 0;
}

uint64_t OpticalFlowAiding::timeLastFuse() const
{
	uint64_t latest = 0;

	for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
		latest = math::max(latest, _sources[i]._aid_src.time_last_fuse);
	}

	return latest;
}

uint64_t OpticalFlowAiding::latestSampleTimestamp() const
{
	uint64_t latest = 0;

	for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
		latest = math::max(latest, _sources[i]._aid_src.timestamp_sample);
	}

	return latest;
}

float OpticalFlowAiding::maxActiveInnovNorm() const
{
	float innov_norm = 0.f;

	for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
		if (_sources[i]._active) {
			innov_norm = math::max(innov_norm, Vector2f(_sources[i]._aid_src.innovation).norm());
		}
	}

	return innov_norm;
}

float OpticalFlowAiding::maxActiveTestRatioFiltered() const
{
	float test_ratio = 0.f;

	for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
		if (_sources[i]._active) {
			for (const auto &test_ratio_filtered : _sources[i]._aid_src.test_ratio_filtered) {
				test_ratio = math::max(test_ratio, fabsf(test_ratio_filtered));
			}
		}
	}

	return test_ratio;
}

void OpticalFlowAiding::getLimits(const Ekf &ekf, float &hagl_min, float &hagl_max, float &max_rate) const
{
	// combined envelope of all sensors currently delivering data
	// (sensors with different ranges hand over as the height changes)
	hagl_min = INFINITY;
	hagl_max = 0.f;
	max_rate = 0.f;
	bool any_flow_source = false;

	for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
		if (ekf._fc.of.intended() && (_sources[i].params.ctrl != 0) && (_sources[i]._buffer != nullptr)
		    && ekf.isRecent(_sources[i]._buffer->get_newest().time_us, (uint64_t)1e6)) {
			hagl_min = math::min(hagl_min, _sources[i]._min_distance);
			hagl_max = math::max(hagl_max, _sources[i]._max_distance);
			max_rate = math::max(max_rate, _sources[i]._max_rate);
			any_flow_source = true;
		}
	}

	if (!any_flow_source) {
		// no flow source delivering data, fall back to the primary slot's limits
		const uint8_t slot = primarySlot();
		hagl_min = _sources[slot]._min_distance;
		hagl_max = _sources[slot]._max_distance;
		max_rate = _sources[slot]._max_rate;
	}
}

void OpticalFlowSource::setData(const flowSample &flow, const uint8_t buffer_length, const uint64_t min_obs_interval_us,
				const float dt_ekf_avg)
{
	if (params.ctrl == 0) {
		return;
	}

	// allocated on the first sample of an enabled slot, so that a slot can be enabled at runtime
	if (_buffer == nullptr) {
		_buffer = new TimestampedRingBuffer<flowSample>(buffer_length);

		if (_buffer == nullptr || !_buffer->valid()) {
			delete _buffer;
			_buffer = nullptr;
			ECL_ERR("optical flow %d buffer allocation failed", _slot);
			return;
		}
	}

	// the measurement delay is already compensated in the sample timestamp (SENS_FLOW<i>_DELAY)
	const int64_t time_us = flow.time_us
				- static_cast<int64_t>(dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

	// limit data rate to prevent data being lost
	if (time_us >= static_cast<int64_t>(_buffer->get_newest().time_us + min_obs_interval_us)) {

		flowSample optflow_sample_new{flow};
		optflow_sample_new.time_us = time_us;

		_buffer->push(optflow_sample_new);

	} else {
		ECL_WARN("optical flow %d data too fast %" PRIi64 " < %" PRIu64 " + %" PRIu64, _slot, time_us,
			 _buffer->get_newest().time_us, min_obs_interval_us);
	}
}

bool OpticalFlowSource::isFusing(const Ekf &ekf) const
{
	return _active && !ekf.isTimedOut(_aid_src.time_last_fuse, ekf._params.no_aid_timeout_max);
}

void OpticalFlowSource::update(Ekf &ekf, const imuSample &imu_delayed, const bool other_slot_fusing)
{
	if (!_buffer || !ekf._fc.of.intended() || (params.ctrl == 0)) {
		stop();
		return;
	}

	Ekf::VectorState H;

	// New optical flow data is available and is ready to be fused when the midpoint of the sample falls behind the fusion time horizon
	if (_buffer->pop_first_older_than(imu_delayed.time_us, &_sample_delayed)) {

		// flow gyro has opposite sign convention
		_ref_body_rate = -(imu_delayed.delta_ang / imu_delayed.delta_ang_dt - ekf.getGyroBias());

		// ensure valid flow sample gyro rate before proceeding
		switch (static_cast<FlowGyroSource>(params.gyr_src)) {
		default:

		/* FALLTHROUGH */
		case FlowGyroSource::Auto:
			if (!PX4_ISFINITE(_sample_delayed.gyro_rate(0)) || !PX4_ISFINITE(_sample_delayed.gyro_rate(1))) {
				_sample_delayed.gyro_rate = _ref_body_rate;
			}

			if (!PX4_ISFINITE(_sample_delayed.gyro_rate(2))) {
				// Some flow modules only provide X ind Y angular rates. If this is the case, complete the vector with our own Z gyro
				_sample_delayed.gyro_rate(2) = _ref_body_rate(2);
			}

			break;

		case FlowGyroSource::Internal:
			_sample_delayed.gyro_rate = _ref_body_rate;
			break;
		}

		const flowSample &flow_sample = _sample_delayed;

		const int32_t min_quality = ekf._control_status.flags.in_air
					    ? params.qmin
					    : params.qmin_gnd;

		const bool is_quality_good = (flow_sample.quality >= min_quality);

		bool is_tilt_good = true;

#if defined(CONFIG_EKF2_RANGE_FINDER)
		is_tilt_good = (ekf._R_to_earth(2, 2) > ekf._params.range_cos_max_tilt);
#endif // CONFIG_EKF2_RANGE_FINDER

		calcBodyRateComp();

		// calculate optical LOS rates using optical flow rates that have had the body angular rate contribution removed
		// correct for gyro bias errors in the data used to do the motion compensation
		// Note the sign convention used: A positive LOS rate is a RH rotation of the scene about that axis.
		const Vector3f flow_gyro_corrected = flow_sample.gyro_rate - _gyro_bias;
		const Vector2f flow_compensated = flow_sample.flow_rate - flow_gyro_corrected.xy();

		// calculate the optical flow observation variance
		const float R_LOS = calcOptFlowMeasVar(flow_sample);

		const float epsilon = 1e-3f;
		Vector2f innov_var;
		sym::ComputeFlowXyInnovVarAndHx(ekf._state.vector(), ekf.P, R_LOS, epsilon, &innov_var, &H);

		_aid_src.device_id = flow_sample.device_id;

		// run the innovation consistency check and record result
		ekf.updateAidSourceStatus(_aid_src,
					  flow_sample.time_us,                                          // sample timestamp
					  flow_compensated,                                             // observation
					  Vector2f{R_LOS, R_LOS},                                       // observation variance
					  ekf.predictFlow(_pos_body, flow_gyro_corrected) - flow_compensated, // innovation
					  innov_var,                                                    // innovation variance
					  math::max(params.gate, 1.f));                                 // innovation gate

		// logging
		_rate_compensated = flow_compensated;

		// compute the velocities in body and local frames from corrected optical flow measurement for logging only
		const float range = ekf.predictFlowRange(_pos_body);
		_vel_body(0) = -flow_compensated(1) * range;
		_vel_body(1) =  flow_compensated(0) * range;

		if (_counter == 0) {
			_vel_body_lpf.setParameters(static_cast<uint64_t>(ekf._dt_ekf_avg * 1e6f), ekf._kSensorLpfTimeConstant);
			_rate_compensated_lpf.setParameters(static_cast<uint64_t>(ekf._dt_ekf_avg * 1e6f), ekf._kSensorLpfTimeConstant);

			_vel_body_lpf.reset(_vel_body);
			_rate_compensated_lpf.reset(_rate_compensated);
			_counter = 1;

		} else {

			_vel_body_lpf.update(_vel_body);
			_rate_compensated_lpf.update(_rate_compensated);
			_counter++;
		}

		// Check if we are in-air and require optical flow to control position drift
		bool is_flow_required = ekf._control_status.flags.in_air
					&& (ekf._control_status.flags.inertial_dead_reckoning // is doing inertial dead-reckoning so must constrain drift urgently
					    || ekf.isOnlyActiveSourceOfHorizontalAiding(ekf._control_status.flags.opt_flow));

		const bool is_within_sensor_dist = (ekf.getHagl() >= _min_distance) && (ekf.getHagl() <= _max_distance);

		const bool is_magnitude_good = flow_sample.flow_rate.isAllFinite()
					       && !flow_sample.flow_rate.longerThan(_max_rate)
					       && !flow_compensated.longerThan(_max_rate);

		const bool continuing_conditions_passing = ekf._fc.of.intended() && (params.ctrl != 0)
				&& ekf._control_status.flags.tilt_align
				&& is_within_sensor_dist;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& is_quality_good
				&& is_magnitude_good
				&& is_tilt_good
				&& (_counter > 10)
				&& (ekf.isHeightAboveGroundEstimateValid() || ekf.isHorizontalAidingActive())
				&& ekf.isTimedOut(_aid_src.time_last_fuse, (uint64_t)2e6); // Prevent rapid switching

		// If the height is relative to the ground, terrain height cannot be observed.
		_terrain = _active && !(ekf._height_sensor_ref == HeightSensor::RANGE);

		if (_active) {
			if (continuing_conditions_passing) {

				if (is_quality_good && is_magnitude_good && is_tilt_good) {
					fuse(ekf, H, _terrain);
				}

				// handle the case when we have optical flow, are reliant on it, but have not been using it for an extended period
				// (if another flow sensor is still being fused, this one is the inconsistent one and must not reset the state)
				if (ekf.isTimedOut(_aid_src.time_last_fuse, ekf._params.no_aid_timeout_max)) {
					if (is_flow_required && is_quality_good && is_magnitude_good && !other_slot_fusing) {
						reset(ekf);

						if (_terrain && !ekf.isTerrainEstimateValid()) {
							resetTerrain(ekf);
						}

					} else {
						stop();
					}
				}

			} else {
				stop();
			}

		} else {
			if (starting_conditions_passing) {
				// If the height is relative to the ground, terrain height cannot be observed.
				const bool terrain_observable = (ekf._height_sensor_ref != HeightSensor::RANGE);

				// another slot may have started during this update, before the aiding flags are updated
				if (ekf.isHorizontalAidingActive() || other_slot_fusing) {
					if (fuse(ekf, H, terrain_observable)) {
						ECL_INFO("starting optical flow %d", _slot);
						_active = true;

					} else if (terrain_observable && !ekf._control_status.flags.rng_terrain && !other_slot_fusing) {
						// only when no other flow sensor observes the terrain, otherwise retry on the next sample
						ECL_INFO("starting optical flow %d, resetting terrain", _slot);
						resetTerrain(ekf);
						_active = true;
					}

				} else {
					if (ekf.isHeightAboveGroundEstimateValid()) {
						ECL_INFO("starting optical flow %d, resetting", _slot);
						reset(ekf);
						_active = true;

					} else if (terrain_observable) {
						ECL_INFO("starting optical flow %d, resetting terrain", _slot);
						resetTerrain(ekf);
						_active = true;
					}
				}

				_terrain = _active && terrain_observable;
			}
		}


	} else if (_active && ekf.isTimedOut(_sample_delayed.time_us, ekf._params.reset_timeout_max)) {
		stop();
	}
}

bool OpticalFlowSource::fuse(Ekf &ekf, Ekf::VectorState &H, const bool update_terrain)
{
	return ekf.fuseOptFlow(_aid_src, _pos_body, _sample_delayed.gyro_rate - _gyro_bias, params.gate, H, update_terrain);
}

void OpticalFlowSource::reset(Ekf &ekf)
{
	ECL_INFO("reset velocity to flow %d", _slot);
	ekf._information_events.flags.reset_vel_to_flow = true;

	const float flow_vel_var = sq(ekf.predictFlowRange(_pos_body)) * calcOptFlowMeasVar(_sample_delayed);
	ekf.resetHorizontalVelocityTo(ekf.getFilteredFlowVelNE(_slot), flow_vel_var);

	ekf.resetAidSourceStatusZeroInnovation(_aid_src);
}

void OpticalFlowSource::resetTerrain(Ekf &ekf)
{
	ECL_INFO("reset hagl to flow %d", _slot);

	float new_terrain = -ekf._gpos.altitude() + ekf._params.ekf2_min_rng;

	if (ekf.isOtherSourceOfHorizontalAidingThan(ekf._control_status.flags.opt_flow)) {
		// ||vel_NE|| = ||( R * flow_body * range).xy()||
		// range = ||vel_NE|| / ||P * R * flow_body||
		constexpr float kProjXY[2][3] = {{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}};
		const matrix::Matrix<float, 2, 3> proj(kProjXY);

		const Vector3f flow_body(-_rate_compensated_lpf.getState()(1), _rate_compensated_lpf.getState()(0), 0.f);
		const float denom = Vector2f(proj * ekf._R_to_earth * flow_body).norm();

		if (denom > 1e-6f) {
			const float range = ekf._state.vel.xy().norm() / denom;
			new_terrain = -ekf._gpos.altitude() + math::max(range, ekf._params.ekf2_min_rng);
		}
	}

	ekf.resetTerrainTo(new_terrain, 100.f);
	ekf.resetAidSourceStatusZeroInnovation(_aid_src);
}

void OpticalFlowSource::stop()
{
	if (_active) {
		ECL_INFO("stopping optical flow fusion %d", _slot);
		_active = false;
		_terrain = false;

		_counter = 0;
	}
}

void OpticalFlowSource::calcBodyRateComp()
{
	// calculate the bias estimate using a combined LPF and spike filter
	_gyro_bias = 0.99f * _gyro_bias
		     + 0.01f * matrix::constrain(_sample_delayed.gyro_rate - _ref_body_rate, -0.1f, 0.1f);
}
