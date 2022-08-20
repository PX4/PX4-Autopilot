/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * Function for fusing rangefinder measurements to estimate terrain vertical position/
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

void Ekf::initHagl()
{
	resetHaglFake();
}

void Ekf::runTerrainEstimator()
{
	// If we are on ground, store the local position and time to use as a reference
	if (!_control_status.flags.in_air) {
		_last_on_ground_posD = _state.pos(2);
		_control_status.flags.rng_fault = false;
	}

	predictHagl();

	controlHaglRngFusion();
	controlHaglFlowFusion();
	controlHaglFakeFusion();

	// constrain _terrain_vpos to be a minimum of _params.rng_gnd_clearance larger than _state.pos(2)
	if (_terrain_vpos - _state.pos(2) < _params.rng_gnd_clearance) {
		_terrain_vpos = _params.rng_gnd_clearance + _state.pos(2);
	}

	updateTerrainValidity();
}

void Ekf::predictHagl()
{
	// predict the state variance growth where the state is the vertical position of the terrain underneath the vehicle

	// process noise due to errors in vehicle height estimate
	_terrain_var += sq(_imu_sample_delayed.delta_vel_dt * _params.terrain_p_noise);

	// process noise due to terrain gradient
	_terrain_var += sq(_imu_sample_delayed.delta_vel_dt * _params.terrain_gradient)
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

		_time_last_healthy_rng_data = _time_last_imu;

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
						_range_sensor.setFaulty();
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
	_time_last_hagl_fuse = _time_last_imu;
}

void Ekf::stopHaglRngFusion()
{
	_hagl_sensor_status.flags.range_finder = false;
	_hagl_innov = 0.f;
	_hagl_innov_var = 0.f;
	_hagl_test_ratio = 0.f;
	_innov_check_fail_status.flags.reject_hagl = false;
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
		_time_last_hagl_fuse = _time_last_imu;
		_innov_check_fail_status.flags.reject_hagl = false;

	} else {
		_innov_check_fail_status.flags.reject_hagl = true;
	}
}

void Ekf::controlHaglFlowFusion()
{
	if (!(_params.terrain_fusion_mode & TerrainFusionMask::TerrainFuseOpticalFlow)) {
		stopHaglFlowFusion();
		return;
	}

	if (_flow_data_ready) {
		const bool continuing_conditions_passing = _control_status.flags.in_air
		                                           && !_control_status.flags.opt_flow
							   && _control_status.flags.gps
							   && isTimedOut(_time_last_hagl_fuse, 5e6f); // TODO: check for range_finder hagl aiding instead?
							   /* && !_hagl_sensor_status.flags.range_finder; */
		const bool starting_conditions_passing = continuing_conditions_passing;

		if (_hagl_sensor_status.flags.flow) {
			if (continuing_conditions_passing) {

				// TODO: wait until the midpoint of the flow sample has fallen behind the fusion time horizon
				fuseFlowForTerrain();
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
		   && (_imu_sample_delayed.time_us >  _flow_sample_delayed.time_us + (uint64_t)5e6)) {
		// No data anymore. Stop until it comes back.
		stopHaglFlowFusion();
	}
}

void Ekf::startHaglFlowFusion()
{
	_hagl_sensor_status.flags.flow = true;
	// TODO: do a reset instead of trying to fuse the data?
	fuseFlowForTerrain();
	_flow_data_ready = false;
}

void Ekf::stopHaglFlowFusion()
{
	_hagl_sensor_status.flags.flow = false;
	_hagl_innov = 0.f;
	_hagl_innov_var = 0.f;
	_hagl_test_ratio = 0.f;
	_innov_check_fail_status.flags.reject_hagl = false;
}

void Ekf::resetHaglFlow()
{
	// TODO: use the flow data
	_terrain_vpos = fmaxf(0.0f,  _state.pos(2));
	_terrain_var = 100.0f;
	_terrain_vpos_reset_counter++;
}

void Ekf::fuseFlowForTerrain()
{
	// calculate optical LOS rates using optical flow rates that have had the body angular rate contribution removed
	// correct for gyro bias errors in the data used to do the motion compensation
	// Note the sign convention used: A positive LOS rate is a RH rotation of the scene about that axis.
	const Vector2f opt_flow_rate = _flow_compensated_XY_rad / _flow_sample_delayed.dt + Vector2f(_flow_gyro_bias);

	// get latest estimated orientation
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	// calculate the optical flow observation variance
	const float R_LOS = calcOptFlowMeasVar();

	// get rotation matrix from earth to body
	const Dcmf earth_to_body = quatToInverseRotMat(_state.quat_nominal);

	// calculate the sensor position relative to the IMU
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the velocity of the sensor relative to the imu in body frame
	// Note: _flow_sample_delayed.gyro_xyz is the negative of the body angular velocity, thus use minus sign
	const Vector3f vel_rel_imu_body = Vector3f(-_flow_sample_delayed.gyro_xyz / _flow_sample_delayed.dt) % pos_offset_body;

	// calculate the velocity of the sensor in the earth frame
	const Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

	// rotate into body frame
	const Vector3f vel_body = earth_to_body * vel_rel_earth;

	const float t0 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

	// constrain terrain to minimum allowed value and predict height above ground
	_terrain_vpos = fmaxf(_terrain_vpos, _params.rng_gnd_clearance + _state.pos(2));
	const float pred_hagl_inv = 1.f / (_terrain_vpos - _state.pos(2));

	// Calculate observation matrix for flow around the vehicle x axis
	const float Hx = vel_body(1) * t0 * pred_hagl_inv * pred_hagl_inv;

	// Constrain terrain variance to be non-negative
	_terrain_var = fmaxf(_terrain_var, 0.0f);

	// Cacluate innovation variance
	_flow_innov_var(0) = Hx * Hx * _terrain_var + R_LOS;

	// calculate the kalman gain for the flow x measurement
	const float Kx = _terrain_var * Hx / _flow_innov_var(0);

	// calculate prediced optical flow about x axis
	const float pred_flow_x = vel_body(1) * earth_to_body(2, 2) * pred_hagl_inv;

	// calculate flow innovation (x axis)
	_flow_innov(0) = pred_flow_x - opt_flow_rate(0);

	// calculate correction term for terrain variance
	const float KxHxP =  Kx * Hx * _terrain_var;

	// innovation consistency check
	const float gate_size = fmaxf(_params.flow_innov_gate, 1.0f);
	float flow_test_ratio = sq(_flow_innov(0)) / (sq(gate_size) * _flow_innov_var(0));

	// do not perform measurement update if badly conditioned
	if (flow_test_ratio <= 1.0f) {
		_terrain_vpos += Kx * _flow_innov(0);
		// guard against negative variance
		_terrain_var = fmaxf(_terrain_var - KxHxP, 0.0f);
		_time_last_flow_terrain_fuse = _time_last_imu;
	}

	// Calculate observation matrix for flow around the vehicle y axis
	const float Hy = -vel_body(0) * t0 * pred_hagl_inv * pred_hagl_inv;

	// Calculuate innovation variance
	_flow_innov_var(1) = Hy * Hy * _terrain_var + R_LOS;

	// calculate the kalman gain for the flow y measurement
	const float Ky = _terrain_var * Hy / _flow_innov_var(1);

	// calculate prediced optical flow about y axis
	const float pred_flow_y = -vel_body(0) * earth_to_body(2, 2) * pred_hagl_inv;

	// calculate flow innovation (y axis)
	_flow_innov(1) = pred_flow_y - opt_flow_rate(1);

	// calculate correction term for terrain variance
	const float KyHyP =  Ky * Hy * _terrain_var;

	// innovation consistency check
	flow_test_ratio = sq(_flow_innov(1)) / (sq(gate_size) * _flow_innov_var(1));

	if (flow_test_ratio <= 1.0f) {
		_terrain_vpos += Ky * _flow_innov(1);
		// guard against negative variance
		_terrain_var = fmaxf(_terrain_var - KyHyP, 0.0f);
		_time_last_flow_terrain_fuse = _time_last_imu;
	}
}

void Ekf::controlHaglFakeFusion()
{
	if (!_control_status.flags.in_air
	    && !_hagl_sensor_status.flags.range_finder
	    && !_hagl_sensor_status.flags.flow) {
		resetHaglFake();
	}
}

void Ekf::resetHaglFake()
{
	// assume a ground clearance
	_terrain_vpos = _state.pos(2) + _params.rng_gnd_clearance;
	// use the ground clearance value as our uncertainty
	_terrain_var = sq(_params.rng_gnd_clearance);
	_time_last_hagl_fuse = _time_last_imu;
}

void Ekf::updateTerrainValidity()
{
	// we have been fusing range finder measurements in the last 5 seconds
	const bool recent_range_fusion = isRecent(_time_last_hagl_fuse, (uint64_t)5e6);

	// we have been fusing optical flow measurements for terrain estimation within the last 5 seconds
	// this can only be the case if the main filter does not fuse optical flow
	const bool recent_flow_for_terrain_fusion = isRecent(_time_last_flow_terrain_fuse, (uint64_t)5e6);

	_hagl_valid = (recent_range_fusion || recent_flow_for_terrain_fusion);
}
