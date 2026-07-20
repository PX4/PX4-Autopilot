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

void Ekf::controlOpticalFlowFusion(const imuSample &imu_delayed)
{
	bool any_ctrl_enabled = false;

	for (uint8_t slot = 0; slot < MAX_OF_INSTANCES; slot++) {
		any_ctrl_enabled |= (_params.of[slot].ctrl != 0);
	}

	_fc.of.available = any_ctrl_enabled;

	for (uint8_t slot = 0; slot < MAX_OF_INSTANCES; slot++) {
		controlOpticalFlowFusionSlot(slot, imu_delayed);

		bool any_active = false;
		bool any_terrain = false;

		for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
			any_active |= _flow_src[i].active;
			any_terrain |= _flow_src[i].terrain;
		}

		_control_status.flags.opt_flow = any_active;
		_control_status.flags.opt_flow_terrain = any_terrain;
	}

	if (!_control_status.flags.opt_flow) {
		_fault_status.flags.bad_optflow_X = false;
		_fault_status.flags.bad_optflow_Y = false;
	}
}

void Ekf::controlOpticalFlowFusionSlot(const uint8_t slot, const imuSample &imu_delayed)
{
	OpticalFlowSource &src = _flow_src[slot];

	if (!src.buffer || !isFlowSlotIntended(slot)) {
		stopFlowFusion(slot);
		return;
	}

	VectorState H;

	// New optical flow data is available and is ready to be fused when the midpoint of the sample falls behind the fusion time horizon
	if (src.buffer->pop_first_older_than(imu_delayed.time_us, &src.sample_delayed)) {

		// flow gyro has opposite sign convention
		_ref_body_rate = -(imu_delayed.delta_ang / imu_delayed.delta_ang_dt - getGyroBias());

		// ensure valid flow sample gyro rate before proceeding
		switch (static_cast<FlowGyroSource>(_params.of[slot].gyr_src)) {
		default:

		/* FALLTHROUGH */
		case FlowGyroSource::Auto:
			if (!PX4_ISFINITE(src.sample_delayed.gyro_rate(0)) || !PX4_ISFINITE(src.sample_delayed.gyro_rate(1))) {
				src.sample_delayed.gyro_rate = _ref_body_rate;
			}

			if (!PX4_ISFINITE(src.sample_delayed.gyro_rate(2))) {
				// Some flow modules only provide X ind Y angular rates. If this is the case, complete the vector with our own Z gyro
				src.sample_delayed.gyro_rate(2) = _ref_body_rate(2);
			}

			break;

		case FlowGyroSource::Internal:
			src.sample_delayed.gyro_rate = _ref_body_rate;
			break;
		}

		const flowSample &flow_sample = src.sample_delayed;

		const int32_t min_quality = _control_status.flags.in_air
					    ? _params.of[slot].qmin
					    : _params.of[slot].qmin_gnd;

		const bool is_quality_good = (flow_sample.quality >= min_quality);

		bool is_tilt_good = true;

#if defined(CONFIG_EKF2_RANGE_FINDER)
		is_tilt_good = (_R_to_earth(2, 2) > _params.range_cos_max_tilt);
#endif // CONFIG_EKF2_RANGE_FINDER

		calcOptFlowBodyRateComp(slot);

		// calculate optical LOS rates using optical flow rates that have had the body angular rate contribution removed
		// correct for gyro bias errors in the data used to do the motion compensation
		// Note the sign convention used: A positive LOS rate is a RH rotation of the scene about that axis.
		const Vector3f flow_gyro_corrected = flow_sample.gyro_rate - src.gyro_bias;
		const Vector2f flow_compensated = flow_sample.flow_rate - flow_gyro_corrected.xy();

		// calculate the optical flow observation variance
		const float R_LOS = calcOptFlowMeasVar(slot, flow_sample);

		const float epsilon = 1e-3f;
		Vector2f innov_var;
		sym::ComputeFlowXyInnovVarAndHx(_state.vector(), P, R_LOS, epsilon, &innov_var, &H);

		// run the innovation consistency check and record result
		updateAidSourceStatus(src.aid_src,
				      flow_sample.time_us,                                       // sample timestamp
				      flow_compensated,                                          // observation
				      Vector2f{R_LOS, R_LOS},                                    // observation variance
				      predictFlow(slot, flow_gyro_corrected) - flow_compensated, // innovation
				      innov_var,                                                 // innovation variance
				      math::max(_params.of[slot].gate, 1.f));                    // innovation gate

		// logging
		src.rate_compensated = flow_compensated;

		// compute the velocities in body and local frames from corrected optical flow measurement for logging only
		const float range = predictFlowRange(slot);
		src.vel_body(0) = -flow_compensated(1) * range;
		src.vel_body(1) =  flow_compensated(0) * range;

		if (src.counter == 0) {
			src.vel_body_lpf.setParameters(_dt_ekf_avg, _kSensorLpfTimeConstant);
			src.rate_compensated_lpf.setParameters(_dt_ekf_avg, _kSensorLpfTimeConstant);

			src.vel_body_lpf.reset(src.vel_body);
			src.rate_compensated_lpf.reset(src.rate_compensated);
			src.counter = 1;

		} else {

			src.vel_body_lpf.update(src.vel_body);
			src.rate_compensated_lpf.update(src.rate_compensated);
			src.counter++;
		}

		// Check if we are in-air and require optical flow to control position drift
		bool is_flow_required = _control_status.flags.in_air
					&& (_control_status.flags.inertial_dead_reckoning // is doing inertial dead-reckoning so must constrain drift urgently
					    || isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow));

		const bool is_within_sensor_dist = (getHagl() >= src.min_distance) && (getHagl() <= src.max_distance);

		const bool is_magnitude_good = flow_sample.flow_rate.isAllFinite()
					       && !flow_sample.flow_rate.longerThan(src.max_rate)
					       && !flow_compensated.longerThan(src.max_rate);

		const bool continuing_conditions_passing = isFlowSlotIntended(slot)
				&& _control_status.flags.tilt_align
				&& is_within_sensor_dist;

		const bool starting_conditions_passing = continuing_conditions_passing
				&& is_quality_good
				&& is_magnitude_good
				&& is_tilt_good
				&& (src.counter > 10)
				&& (isHeightAboveGroundEstimateValid() || isHorizontalAidingActive())
				&& isTimedOut(src.aid_src.time_last_fuse, (uint64_t)2e6); // Prevent rapid switching

		// If the height is relative to the ground, terrain height cannot be observed.
		src.terrain = src.active && !(_height_sensor_ref == HeightSensor::RANGE);

		if (src.active) {
			if (continuing_conditions_passing) {

				if (is_quality_good && is_magnitude_good && is_tilt_good) {
					fuseOptFlow(slot, H, src.terrain);
				}

				// handle the case when we have optical flow, are reliant on it, but have not been using it for an extended period
				if (isTimedOut(src.aid_src.time_last_fuse, _params.no_aid_timeout_max)) {
					if (is_flow_required && is_quality_good && is_magnitude_good) {
						resetFlowFusion(slot);

						if (src.terrain && !isTerrainEstimateValid()) {
							resetTerrainToFlow(slot);
						}

					} else {
						stopFlowFusion(slot);
					}
				}

			} else {
				stopFlowFusion(slot);
			}

		} else {
			if (starting_conditions_passing) {
				// If the height is relative to the ground, terrain height cannot be observed.
				const bool terrain_observable = (_height_sensor_ref != HeightSensor::RANGE);

				if (isHorizontalAidingActive()) {
					if (fuseOptFlow(slot, H, terrain_observable)) {
						ECL_INFO("starting optical flow %d", slot);
						src.active = true;

					} else if (terrain_observable && !_control_status.flags.rng_terrain) {
						ECL_INFO("starting optical flow %d, resetting terrain", slot);
						resetTerrainToFlow(slot);
						src.active = true;
					}

				} else {
					if (isHeightAboveGroundEstimateValid()) {
						ECL_INFO("starting optical flow %d, resetting", slot);
						resetFlowFusion(slot);
						src.active = true;

					} else if (terrain_observable) {
						ECL_INFO("starting optical flow %d, resetting terrain", slot);
						resetTerrainToFlow(slot);
						src.active = true;
					}
				}

				src.terrain = src.active && terrain_observable;
			}
		}

	} else if (src.active && isTimedOut(src.sample_delayed.time_us, _params.reset_timeout_max)) {
		stopFlowFusion(slot);
	}
}

void Ekf::resetFlowFusion(const uint8_t slot)
{
	OpticalFlowSource &src = _flow_src[slot];

	ECL_INFO("reset velocity to flow %d", slot);
	_information_events.flags.reset_vel_to_flow = true;

	const float flow_vel_var = sq(predictFlowRange(slot)) * calcOptFlowMeasVar(slot, src.sample_delayed);
	resetHorizontalVelocityTo(getFilteredFlowVelNE(slot), flow_vel_var);

	resetAidSourceStatusZeroInnovation(src.aid_src);
}

void Ekf::resetTerrainToFlow(const uint8_t slot)
{
	OpticalFlowSource &src = _flow_src[slot];

	ECL_INFO("reset hagl to flow %d", slot);

	float new_terrain = -_gpos.altitude() + _params.ekf2_min_rng;

	if (isOtherSourceOfHorizontalAidingThan(_control_status.flags.opt_flow)) {
		// ||vel_NE|| = ||( R * flow_body * range).xy()||
		// range = ||vel_NE|| / ||P * R * flow_body||
		constexpr float kProjXY[2][3] = {{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}};
		const matrix::Matrix<float, 2, 3> proj(kProjXY);

		const Vector3f flow_body(-src.rate_compensated_lpf.getState()(1), src.rate_compensated_lpf.getState()(0), 0.f);
		const float denom = Vector2f(proj * _R_to_earth * flow_body).norm();

		if (denom > 1e-6f) {
			const float range = _state.vel.xy().norm() / denom;
			new_terrain = -_gpos.altitude() + max(range, _params.ekf2_min_rng);
		}
	}

	const float delta_terrain = new_terrain - _state.terrain;
	_state.terrain = new_terrain;
	P.uncorrelateCovarianceSetVariance<State::terrain.dof>(State::terrain.idx, 100.f);

	resetAidSourceStatusZeroInnovation(src.aid_src);

	// record the state change
	if (_state_reset_status.reset_count.hagl == _state_reset_count_prev.hagl) {
		_state_reset_status.hagl_change = delta_terrain;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.hagl_change += delta_terrain;
	}

	_state_reset_status.reset_count.hagl++;
}

void Ekf::stopFlowFusion(const uint8_t slot)
{
	OpticalFlowSource &src = _flow_src[slot];

	if (src.active) {
		ECL_INFO("stopping optical flow fusion %d", slot);
		src.active = false;
		src.terrain = false;

		src.counter = 0;
	}
}

void Ekf::calcOptFlowBodyRateComp(const uint8_t slot)
{
	OpticalFlowSource &src = _flow_src[slot];

	// calculate the bias estimate using a combined LPF and spike filter
	src.gyro_bias = 0.99f * src.gyro_bias
			+ 0.01f * matrix::constrain(src.sample_delayed.gyro_rate - _ref_body_rate, -0.1f, 0.1f);
}
