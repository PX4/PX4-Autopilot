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
 * @file optflow_fusion.cpp
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
#include <ekf_derivation/generated/compute_flow_xy_innov_var_and_hx.h>
#include <ekf_derivation/generated/compute_flow_y_innov_var_and_h.h>

void Ekf::updateOptFlow(estimator_aid_source2d_s &aid_src)
{
	resetEstimatorAidStatus(aid_src);
	aid_src.timestamp_sample = _flow_sample_delayed.time_us;

	const Vector2f vel_body = predictFlowVelBody();
	const float range = predictFlowRange();

	// calculate optical LOS rates using optical flow rates that have had the body angular rate contribution removed
	// correct for gyro bias errors in the data used to do the motion compensation
	// Note the sign convention used: A positive LOS rate is a RH rotation of the scene about that axis.
	const Vector2f opt_flow_rate = _flow_rate_compensated;

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

	Vector2f innov_var;
	VectorState H;
	sym::ComputeFlowXyInnovVarAndHx(_state.vector(), P, range, R_LOS, FLT_EPSILON, &innov_var, &H);
	innov_var.copyTo(aid_src.innovation_variance);

	// run the innovation consistency check and record result
	setEstimatorAidStatusTestRatio(aid_src, math::max(_params.flow_innov_gate, 1.f));
}

void Ekf::fuseOptFlow()
{
	const float R_LOS = _aid_src_optical_flow.observation_variance[0];

	// calculate the height above the ground of the optical flow camera. Since earth frame is NED
	// a positive offset in earth frame leads to a smaller height above the ground.
	float range = predictFlowRange();

	const auto state_vector = _state.vector();

	Vector2f innov_var;
	VectorState H;
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

		VectorState Kfusion = P * H / _aid_src_optical_flow.innovation_variance[index];

		if (measurementUpdate(Kfusion, H, _aid_src_optical_flow.observation_variance[index], _aid_src_optical_flow.innovation[index])) {
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
	// Note: _flow_sample_delayed.gyro_rate is the negative of the body angular velocity, thus use minus sign
	const Vector3f vel_rel_imu_body = Vector3f(-(_flow_sample_delayed.gyro_rate - _flow_gyro_bias)) % pos_offset_body;

	// calculate the velocity of the sensor in the earth frame
	const Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

	// rotate into body frame
	return _state.quat_nominal.rotateVectorInverse(vel_rel_earth).xy();
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
