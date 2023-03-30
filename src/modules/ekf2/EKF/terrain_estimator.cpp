/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file terrain_estimator.cpp
 * Function for fusing rangefinder and optical flow measurements
 * to estimate terrain vertical position
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/terr_est_compute_flow_xy_innov_var_and_hx.h"
#include "python/ekf_derivation/generated/terr_est_compute_flow_y_innov_var_and_h.h"

#include <mathlib/mathlib.h>

void Ekf::initHagl()
{
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	stopHaglFlowFusion();
#endif // CONFIG_EKF2_OPTICAL_FLOW

	stopHaglRngFusion();

	// assume a ground clearance
	_terrain_vpos = _state.pos(2) + _params.rng_gnd_clearance;

	// use the ground clearance value as our uncertainty
	_terrain_var = sq(_params.rng_gnd_clearance);
}

void Ekf::runTerrainEstimator(const imuSample &imu_delayed)
{
	// If we are on ground, store the local position and time to use as a reference
	if (!_control_status.flags.in_air) {
		_last_on_ground_posD = _state.pos(2);
		_control_status.flags.rng_fault = false;
	}

	predictHagl(imu_delayed);

	controlHaglRngFusion();
#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	controlHaglFlowFusion();
#endif // CONFIG_EKF2_OPTICAL_FLOW
	controlHaglFakeFusion();

	// constrain _terrain_vpos to be a minimum of _params.rng_gnd_clearance larger than _state.pos(2)
	if (_terrain_vpos - _state.pos(2) < _params.rng_gnd_clearance) {
		_terrain_vpos = _params.rng_gnd_clearance + _state.pos(2);
	}
}

void Ekf::predictHagl(const imuSample &imu_delayed)
{
	// predict the state variance growth where the state is the vertical position of the terrain underneath the vehicle

	// process noise due to errors in vehicle height estimate
	_terrain_var += sq(imu_delayed.delta_vel_dt * _params.terrain_p_noise);

	// process noise due to terrain gradient
	_terrain_var += sq(imu_delayed.delta_vel_dt * _params.terrain_gradient)
			* (sq(_state.vel(0)) + sq(_state.vel(1)));

	// limit the variance to prevent it becoming badly conditioned
	_terrain_var = math::constrain(_terrain_var, 0.0f, 1e4f);
}

void Ekf::controlHaglRngFusion()
{
	if (!(_params.terrain_fusion_mode & TerrainFusionMask::TerrainFuseRangeFinder)
	    || _control_status.flags.rng_fault) {

		stopHaglRngFusion();
		return;
	}

	if (_range_sensor.isDataHealthy()) {
		const bool continuing_conditions_passing = _control_status.flags.in_air && _rng_consistency_check.isKinematicallyConsistent();
		//const bool continuing_conditions_passing = _control_status.flags.in_air && !_control_status.flags.rng_hgt; // TODO: should not be fused when using range height
		const bool starting_conditions_passing = continuing_conditions_passing && _range_sensor.isRegularlySendingData() && (_rng_consistency_check.getTestRatio() < 1.f);

		_time_last_healthy_rng_data = _time_delayed_us;

		if (_hagl_sensor_status.flags.range_finder) {
			if (continuing_conditions_passing) {
				fuseHaglRng();

				// We have been rejecting range data for too long
				const uint64_t timeout = static_cast<uint64_t>(_params.terrain_timeout * 1e6f);
				const bool is_fusion_failing = isTimedOut(_time_last_hagl_fuse, timeout);

				if (is_fusion_failing) {
					if (_range_sensor.getDistBottom() > 2.f * _params.rng_gnd_clearance) {
						// Data seems good, attempt a reset
						resetHaglRng();

					} else if (starting_conditions_passing) {
						// The sensor can probably not detect the ground properly
						// declare the sensor faulty and stop the fusion
						_control_status.flags.rng_fault = true;
						_range_sensor.setFaulty(true);
						stopHaglRngFusion();

					} else {
						// This could be a temporary issue, stop the fusion without declaring the sensor faulty
						stopHaglRngFusion();
					}
				}

			} else {
				stopHaglRngFusion();
			}

		} else {
			if (starting_conditions_passing) {
				startHaglRngFusion();
			}
		}

	} else if (_hagl_sensor_status.flags.range_finder && isTimedOut(_time_last_healthy_rng_data, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopHaglRngFusion();
	}
}

void Ekf::startHaglRngFusion()
{
	_hagl_sensor_status.flags.range_finder = true;
	resetHaglRngIfNeeded();
}

void Ekf::resetHaglRngIfNeeded()
{
	if (_hagl_sensor_status.flags.flow) {
		const float meas_hagl = _range_sensor.getDistBottom();
		const float pred_hagl = _terrain_vpos - _state.pos(2);
		const float hagl_innov = pred_hagl - meas_hagl;
		const float obs_variance = getRngVar();

		const float hagl_innov_var = fmaxf(_terrain_var + obs_variance, obs_variance);

		const float gate_size = fmaxf(_params.range_innov_gate, 1.0f);
		const float hagl_test_ratio = sq(hagl_innov) / (sq(gate_size) * hagl_innov_var);

		// Reset the state to the measurement only if the test ratio is large,
		// otherwise let it converge through the fusion
		if (hagl_test_ratio > 0.2f) {
			resetHaglRng();

		} else {
			fuseHaglRng();
		}

	} else {
		resetHaglRng();
	}
}

float Ekf::getRngVar()
{
	return fmaxf(P(9, 9) * _params.vehicle_variance_scaler, 0.0f)
	       + sq(_params.range_noise)
	       + sq(_params.range_noise_scaler * _range_sensor.getRange());
}

void Ekf::resetHaglRng()
{
	_terrain_vpos = _state.pos(2) + _range_sensor.getDistBottom();
	_terrain_var = getRngVar();
	_terrain_vpos_reset_counter++;
	_time_last_hagl_fuse = _time_delayed_us;
	_time_last_healthy_rng_data = 0;
}

void Ekf::stopHaglRngFusion()
{
	if (_hagl_sensor_status.flags.range_finder) {

		_hagl_innov = 0.f;
		_hagl_innov_var = 0.f;
		_hagl_test_ratio = 0.f;
		_innov_check_fail_status.flags.reject_hagl = false;

		_hagl_sensor_status.flags.range_finder = false;
	}

	_time_last_healthy_rng_data = 0;
}

void Ekf::fuseHaglRng()
{
	// get a height above ground measurement from the range finder assuming a flat earth
	const float meas_hagl = _range_sensor.getDistBottom();

	// predict the hagl from the vehicle position and terrain height
	const float pred_hagl = _terrain_vpos - _state.pos(2);

	// calculate the innovation
	_hagl_innov = pred_hagl - meas_hagl;

	// calculate the observation variance adding the variance of the vehicles own height uncertainty
	const float obs_variance = getRngVar();

	// calculate the innovation variance - limiting it to prevent a badly conditioned fusion
	_hagl_innov_var = fmaxf(_terrain_var + obs_variance, obs_variance);

	// perform an innovation consistency check and only fuse data if it passes
	const float gate_size = fmaxf(_params.range_innov_gate, 1.0f);
	_hagl_test_ratio = sq(_hagl_innov) / (sq(gate_size) * _hagl_innov_var);

	if (_hagl_test_ratio <= 1.0f) {
		// calculate the Kalman gain
		const float gain = _terrain_var / _hagl_innov_var;
		// correct the state
		_terrain_vpos -= gain * _hagl_innov;
		// correct the variance
		_terrain_var = fmaxf(_terrain_var * (1.0f - gain), 0.0f);
		// record last successful fusion event
		_time_last_hagl_fuse = _time_delayed_us;
		_innov_check_fail_status.flags.reject_hagl = false;

	} else {
		_innov_check_fail_status.flags.reject_hagl = true;
	}
}

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
void Ekf::controlHaglFlowFusion()
{
	if (!(_params.terrain_fusion_mode & TerrainFusionMask::TerrainFuseOpticalFlow)) {
		stopHaglFlowFusion();
		return;
	}

	if (_flow_data_ready) {
		updateOptFlow(_aid_src_terrain_optical_flow);

		const bool continuing_conditions_passing = _control_status.flags.in_air
		                                           && !_control_status.flags.opt_flow
							   && _control_status.flags.gps
							   && isTimedOut(_time_last_hagl_fuse, 5e6f); // TODO: check for range_finder hagl aiding instead?
							   /* && !_hagl_sensor_status.flags.range_finder; */
		const bool starting_conditions_passing = continuing_conditions_passing;

		if (_hagl_sensor_status.flags.flow) {
			if (continuing_conditions_passing) {

				// TODO: wait until the midpoint of the flow sample has fallen behind the fusion time horizon
				fuseFlowForTerrain(_aid_src_terrain_optical_flow);
				_flow_data_ready = false;

				// TODO: do something when failing continuously the innovation check
				/* const bool is_fusion_failing = isTimedOut(_time_last_flow_terrain_fuse, _params.reset_timeout_max); */

				/* if (is_fusion_failing) { */
				/* 	resetHaglFlow(); */
				/* } */

			} else {
				stopHaglFlowFusion();
			}

		} else {
			if (starting_conditions_passing) {
				startHaglFlowFusion();
			}
		}

	} else if (_hagl_sensor_status.flags.flow
		   && (_time_delayed_us > _flow_sample_delayed.time_us + (uint64_t)5e6)) {
		// No data anymore. Stop until it comes back.
		stopHaglFlowFusion();
	}
}

void Ekf::startHaglFlowFusion()
{
	_hagl_sensor_status.flags.flow = true;
	// TODO: do a reset instead of trying to fuse the data?
	fuseFlowForTerrain(_aid_src_terrain_optical_flow);
	_flow_data_ready = false;
}

void Ekf::stopHaglFlowFusion()
{
	if (_hagl_sensor_status.flags.flow) {
		_hagl_sensor_status.flags.flow = false;
		resetEstimatorAidStatus(_aid_src_terrain_optical_flow);
	}
}

void Ekf::resetHaglFlow()
{
	// TODO: use the flow data
	_terrain_vpos = fmaxf(0.0f, _state.pos(2));
	_terrain_var = 100.0f;
	_terrain_vpos_reset_counter++;
}

void Ekf::fuseFlowForTerrain(estimator_aid_source2d_s &flow)
{
	flow.fusion_enabled = true;

	const float R_LOS = flow.observation_variance[0];

	// calculate the height above the ground of the optical flow camera. Since earth frame is NED
	// a positive offset in earth frame leads to a smaller height above the ground.
	float range = predictFlowRange();

	const float state = _terrain_vpos; // linearize both axes using the same state value
	Vector2f innov_var;
	float H;
	sym::TerrEstComputeFlowXyInnovVarAndHx(state, _terrain_var, _state.quat_nominal, _state.vel, _state.pos(2), R_LOS, FLT_EPSILON, &innov_var, &H);
	innov_var.copyTo(flow.innovation_variance);

	if ((flow.innovation_variance[0] < R_LOS)
	    || (flow.innovation_variance[1] < R_LOS)) {
		// we need to reinitialise the covariance matrix and abort this fusion step
		ECL_ERR("Opt flow error - covariance reset");
		_terrain_var = 100.0f;
		return;
	}

	// run the innovation consistency check and record result
	setEstimatorAidStatusTestRatio(flow, math::max(_params.flow_innov_gate, 1.f));

	_innov_check_fail_status.flags.reject_optflow_X = (flow.test_ratio[0] > 1.f);
	_innov_check_fail_status.flags.reject_optflow_Y = (flow.test_ratio[1] > 1.f);

	// if either axis fails we abort the fusion
	if (flow.innovation_rejected) {
		return;
	}

	// fuse observation axes sequentially
	for (uint8_t index = 0; index <= 1; index++) {
		if (index == 0) {
			// everything was already computed above

		} else if (index == 1) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::TerrEstComputeFlowYInnovVarAndH(state, _terrain_var, _state.quat_nominal, _state.vel, _state.pos(2), R_LOS, FLT_EPSILON, &flow.innovation_variance[1], &H);

			// recalculate the innovation using the updated state
			const Vector2f vel_body = predictFlowVelBody();
			range = predictFlowRange();
			flow.innovation[1] = (-vel_body(0) / range) - flow.observation[1];

			if (flow.innovation_variance[1] < R_LOS) {
				// we need to reinitialise the covariance matrix and abort this fusion step
				ECL_ERR("Opt flow error - covariance reset");
				_terrain_var = 100.0f;
				return;
			}
		}

		float Kfusion = _terrain_var * H / flow.innovation_variance[index];

		_terrain_vpos += Kfusion * flow.innovation[0];
		// constrain terrain to minimum allowed value and predict height above ground
		_terrain_vpos = fmaxf(_terrain_vpos, _params.rng_gnd_clearance + _state.pos(2));

		// guard against negative variance
		_terrain_var = fmaxf(_terrain_var - Kfusion * H * _terrain_var, sq(0.01f));
	}

	_fault_status.flags.bad_optflow_X = false;
	_fault_status.flags.bad_optflow_Y = false;

	_time_last_flow_terrain_fuse = _time_delayed_us;
	//_aid_src_optical_flow.time_last_fuse = _time_delayed_us; // TODO: separate aid source status for OF terrain?
	_aid_src_optical_flow.fused = true;
}
#endif // CONFIG_EKF2_OPTICAL_FLOW

void Ekf::controlHaglFakeFusion()
{
	if (!_control_status.flags.in_air
	    && !_hagl_sensor_status.flags.range_finder
	    && !_hagl_sensor_status.flags.flow) {

		initHagl();
	}
}

bool Ekf::isTerrainEstimateValid() const
{
	// we have been fusing range finder measurements in the last 5 seconds
	if (_hagl_sensor_status.flags.range_finder && isRecent(_time_last_hagl_fuse, (uint64_t)5e6)) {
		return true;
	}

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	// we have been fusing optical flow measurements for terrain estimation within the last 5 seconds
	// this can only be the case if the main filter does not fuse optical flow
	if (_hagl_sensor_status.flags.flow && isRecent(_time_last_flow_terrain_fuse, (uint64_t)5e6)) {
		return true;
	}
#endif // CONFIG_EKF2_OPTICAL_FLOW

	return false;
}
