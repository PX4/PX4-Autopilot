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
 * @file vel_pos_fusion.cpp
 * Function for fusing gps and baro measurements/
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <float.h>
#include "python/ekf_derivation/generated/compute_flow_xy_innov_var_and_hx.h"
#include "python/ekf_derivation/generated/compute_flow_y_innov_var_and_h.h"
#include "utils.hpp"

void Ekf::updateOptFlow(estimator_aid_source2d_s &aid_src)
{
	resetEstimatorAidStatus(aid_src);
	aid_src.timestamp_sample = _flow_sample_delayed.time_us;

	const Vector2f vel_body = predictFlowVelBody();
	const float range = predictFlowRange();

	// calculate optical LOS rates using optical flow rates that have had the body angular rate contribution removed
	// correct for gyro bias errors in the data used to do the motion compensation
	// Note the sign convention used: A positive LOS rate is a RH rotation of the scene about that axis.
	const Vector2f opt_flow_rate = _flow_compensated_XY_rad / _flow_sample_delayed.dt;

	// compute the velocities in body and local frames from corrected optical flow measurement for logging only
	_flow_vel_body(0) = -opt_flow_rate(1) * range;
	_flow_vel_body(1) =  opt_flow_rate(0) * range;
	_flow_vel_ne = Vector2f(_R_to_earth * Vector3f(_flow_vel_body(0), _flow_vel_body(1), 0.f));

	aid_src.observation[0] = opt_flow_rate(0); // flow around the X axis
	aid_src.observation[1] = opt_flow_rate(1); // flow around the Y axis

	aid_src.innovation[0] =  (vel_body(1) / range) - aid_src.observation[0];
	aid_src.innovation[1] = (-vel_body(0) / range) - aid_src.observation[1];

	// calculate the optical flow observation variance
	const float R_LOS = calcOptFlowMeasVar(_flow_sample_delayed);
	aid_src.observation_variance[0] = R_LOS;
	aid_src.observation_variance[1] = R_LOS;
}

void Ekf::fuseOptFlow()
{
	_aid_src_optical_flow.fusion_enabled = true;

	const float R_LOS = _aid_src_optical_flow.observation_variance[0];

	// calculate the height above the ground of the optical flow camera. Since earth frame is NED
	// a positive offset in earth frame leads to a smaller height above the ground.
	float range = predictFlowRange();

	const Vector24f state_vector = getStateAtFusionHorizonAsVector();

	Vector2f innov_var;
	Vector24f H;
	sym::ComputeFlowXyInnovVarAndHx(state_vector, P, range, R_LOS, FLT_EPSILON, &innov_var, &H);
	innov_var.copyTo(_aid_src_optical_flow.innovation_variance);

	if ((_aid_src_optical_flow.innovation_variance[0] < R_LOS)
	    || (_aid_src_optical_flow.innovation_variance[1] < R_LOS)) {
		// we need to reinitialise the covariance matrix and abort this fusion step
		ECL_ERR("Opt flow error - covariance reset");
		initialiseCovariance();
		return;
	}

	// run the innovation consistency check and record result
	setEstimatorAidStatusTestRatio(_aid_src_optical_flow, math::max(_params.flow_innov_gate, 1.f));

	_innov_check_fail_status.flags.reject_optflow_X = (_aid_src_optical_flow.test_ratio[0] > 1.f);
	_innov_check_fail_status.flags.reject_optflow_Y = (_aid_src_optical_flow.test_ratio[1] > 1.f);

	// if either axis fails we abort the fusion
	if (_aid_src_optical_flow.innovation_rejected) {
		return;
	}

	bool fused[2] {false, false};

	// fuse observation axes sequentially
	for (uint8_t index = 0; index <= 1; index++) {
		if (index == 0) {
			// everything was already computed above

		} else if (index == 1) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeFlowYInnovVarAndH(state_vector, P, range, R_LOS, FLT_EPSILON, &_aid_src_optical_flow.innovation_variance[1], &H);

			// recalculate the innovation using the updated state
			const Vector2f vel_body = predictFlowVelBody();
			range = predictFlowRange();
			_aid_src_optical_flow.innovation[1] = (-vel_body(0) / range) - _aid_src_optical_flow.observation[1];

			if (_aid_src_optical_flow.innovation_variance[1] < R_LOS) {
				// we need to reinitialise the covariance matrix and abort this fusion step
				ECL_ERR("Opt flow error - covariance reset");
				initialiseCovariance();
				return;
			}
		}

		SparseVector24f<0,1,2,3,4,5,6> Hfusion(H);
		Vector24f Kfusion = P * Hfusion / _aid_src_optical_flow.innovation_variance[index];

		if (measurementUpdate(Kfusion, _aid_src_optical_flow.innovation_variance[index], _aid_src_optical_flow.innovation[index])) {
			fused[index] = true;
		}
	}

	_fault_status.flags.bad_optflow_X = !fused[0];
	_fault_status.flags.bad_optflow_Y = !fused[1];

	if (fused[0] && fused[1]) {
		_aid_src_optical_flow.time_last_fuse = _time_delayed_us;
		_aid_src_optical_flow.fused = true;
	}
}

float Ekf::predictFlowRange()
{
	// calculate the sensor position relative to the IMU
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the sensor position relative to the IMU in earth frame
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	// calculate the height above the ground of the optical flow camera. Since earth frame is NED
	// a positive offset in earth frame leads to a smaller height above the ground.
	const float height_above_gnd_est = math::max(_terrain_vpos - _state.pos(2) - pos_offset_earth(2), fmaxf(_params.rng_gnd_clearance, 0.01f));

	// calculate range from focal point to centre of image
	return height_above_gnd_est / _R_to_earth(2, 2); // absolute distance to the frame region in view
}

Vector2f Ekf::predictFlowVelBody()
{
	// calculate the sensor position relative to the IMU
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the velocity of the sensor relative to the imu in body frame
	// Note: _flow_sample_delayed.gyro_xyz is the negative of the body angular velocity, thus use minus sign
	const Vector3f vel_rel_imu_body = Vector3f(-_flow_sample_delayed.gyro_xyz / _flow_sample_delayed.dt) % pos_offset_body;

	// calculate the velocity of the sensor in the earth frame
	const Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

	// rotate into body frame
	return _state.quat_nominal.rotateVectorInverse(vel_rel_earth).xy();
}


// calculate optical flow body angular rate compensation
// returns false if bias corrected body rate data is unavailable
bool Ekf::calcOptFlowBodyRateComp()
{
	bool is_body_rate_comp_available = false;
	const bool use_flow_sensor_gyro = _flow_sample_delayed.gyro_xyz.isAllFinite();

	if (use_flow_sensor_gyro) {

		// if accumulation time differences are not excessive and accumulation time is adequate
		// compare the optical flow and and navigation rate data and calculate a bias error
		if ((_delta_time_of > FLT_EPSILON)
		    && (_flow_sample_delayed.dt > FLT_EPSILON)
		    && (fabsf(_delta_time_of - _flow_sample_delayed.dt) < 0.1f)) {

			const Vector3f reference_body_rate(_imu_del_ang_of * (1.0f / _delta_time_of));

			const Vector3f measured_body_rate(_flow_sample_delayed.gyro_xyz * (1.0f / _flow_sample_delayed.dt));

			// calculate the bias estimate using  a combined LPF and spike filter
			_flow_gyro_bias = _flow_gyro_bias * 0.99f + matrix::constrain(measured_body_rate - reference_body_rate, -0.1f, 0.1f) * 0.01f;

			// apply gyro bias
			_flow_sample_delayed.gyro_xyz -= (_flow_gyro_bias * _flow_sample_delayed.dt);

			is_body_rate_comp_available = true;
		}

	} else {
		// Use the EKF gyro data if optical flow sensor gyro data is not available
		// for clarification of the sign see definition of flowSample and imuSample in common.h
		if ((_delta_time_of > FLT_EPSILON)
		    && (_flow_sample_delayed.dt > FLT_EPSILON)) {

			_flow_sample_delayed.gyro_xyz = -_imu_del_ang_of / _delta_time_of * _flow_sample_delayed.dt;
			_flow_gyro_bias.zero();

			is_body_rate_comp_available = true;
		}
	}

	// reset the accumulators
	_imu_del_ang_of.setZero();
	_delta_time_of = 0.0f;
	return is_body_rate_comp_available;
}

// calculate the measurement variance for the optical flow sensor (rad/sec)^2
float Ekf::calcOptFlowMeasVar(const flowSample &flow_sample)
{
	// calculate the observation noise variance - scaling noise linearly across flow quality range
	const float R_LOS_best = fmaxf(_params.flow_noise, 0.05f);
	const float R_LOS_worst = fmaxf(_params.flow_noise_qual_min, 0.05f);

	// calculate a weighting that varies between 1 when flow quality is best and 0 when flow quality is worst
	float weighting = (255.f - (float)_params.flow_qual_min);

	if (weighting >= 1.f) {
		weighting = math::constrain((float)(flow_sample.quality - _params.flow_qual_min) / weighting, 0.f, 1.f);

	} else {
		weighting = 0.0f;
	}

	// take the weighted average of the observation noise for the best and wort flow quality
	const float R_LOS = sq(R_LOS_best * weighting + R_LOS_worst * (1.f - weighting));

	return R_LOS;
}
