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
#include "utils.hpp"

using D = matrix::Dual<float, 7>;
using matrix::Vector3;

void Ekf::fuseOptFlow()
{
	float gndclearance = fmaxf(_params.rng_gnd_clearance, 0.1f);

	// calculate the optical flow observation variance
	const float R_LOS = calcOptFlowMeasVar();

	// get rotation matrix from earth to body
	const Dcmf earth_to_body = quatToInverseRotMat(_state.quat_nominal);

	matrix::Quaternion<D> q(D(_state.quat_nominal(0), 0),
			D(_state.quat_nominal(1), 1),
			D(_state.quat_nominal(2), 2),
			D(_state.quat_nominal(3), 3));

	// calculate the sensor position relative to the IMU
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the velocity of the sensor relative to the imu in body frame
	// Note: _flow_sample_delayed.gyro_xyz is the negative of the body angular velocity, thus use minus sign
	const Vector3f vel_rel_imu_body = Vector3f(-_flow_sample_delayed.gyro_xyz / _flow_sample_delayed.dt) % pos_offset_body;

	// calculate the velocity of the sensor in the earth frame
	Vector3<D> vel_earth(D(_state.vel(0), 4), D(_state.vel(1), 5), D(_state.vel(2), 6));

	// rotate into body frame
	Vector3<D> vel_body = q.rotateVectorInverse(vel_earth) + Vector3<D>(D(vel_rel_imu_body(0)), D(vel_rel_imu_body(1)), D(vel_rel_imu_body(2)));

	// height above ground of the IMU
	float heightAboveGndEst = _terrain_vpos - _state.pos(2);

	// calculate the sensor position relative to the IMU in earth frame
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	// calculate the height above the ground of the optical flow camera. Since earth frame is NED
	// a positive offset in earth frame leads to a smaller height above the ground.
	heightAboveGndEst -= pos_offset_earth(2);

	// constrain minimum height above ground
	heightAboveGndEst = math::max(heightAboveGndEst, gndclearance);

	// calculate range from focal point to centre of image
	const float range = heightAboveGndEst / earth_to_body(2, 2); // absolute distance to the frame region in view

	// calculate optical LOS rates using optical flow rates that have had the body angular rate contribution removed
	// correct for gyro bias errors in the data used to do the motion compensation
	// Note the sign convention used: A positive LOS rate is a RH rotation of the scene about that axis.
	const Vector2f opt_flow_rate = _flow_compensated_XY_rad / _flow_sample_delayed.dt + Vector2f(_flow_gyro_bias);

	// compute the velocities in body and local frames from corrected optical flow measurement
	// for logging only
	_flow_vel_body(0) = -opt_flow_rate(1) * range;
	_flow_vel_body(1) = opt_flow_rate(0) * range;
	_flow_vel_ne = Vector2f(_R_to_earth * Vector3f(_flow_vel_body(0), _flow_vel_body(1), 0.f));

	matrix::Vector2<D> predicted_flow;
	predicted_flow(0) =  vel_body(1) / range;
	predicted_flow(1) = -vel_body(0) / range;
	_flow_innov(0) = predicted_flow(0).value - opt_flow_rate(0);
	_flow_innov(1) = predicted_flow(1).value - opt_flow_rate(1);

	// fuse observation axes sequentially
	SparseVector24f<0,1,2,3,4,5,6> Hfusion; // Optical flow observation Jacobians
	Vector24f Kfusion; // Optical flow Kalman gains

	for (uint8_t obs_index = 0; obs_index <= 1; obs_index++) {
		// Observation Jacobians
		Hfusion.at<0>() = predicted_flow(obs_index).derivative(0);
		Hfusion.at<1>() = predicted_flow(obs_index).derivative(1);
		Hfusion.at<2>() = predicted_flow(obs_index).derivative(2);
		Hfusion.at<3>() = predicted_flow(obs_index).derivative(3);
		Hfusion.at<4>() = predicted_flow(obs_index).derivative(4);
		Hfusion.at<5>() = predicted_flow(obs_index).derivative(5);
		Hfusion.at<6>() = predicted_flow(obs_index).derivative(6);

		auto PHt = P * Hfusion;
		_flow_innov_var(obs_index) = Hfusion.dot(PHt) + R_LOS;

		// Kalman gains
		Kfusion = PHt / _flow_innov_var(obs_index);

		// calculate innovation variance and protect against a badly conditioned calculation
		if (_flow_innov_var(obs_index) < R_LOS) {
			// we need to reinitialise the covariance matrix and abort this fusion step
			initialiseCovariance();
			return;
		}

		// run the innovation consistency check and record result
		float test_ratio = sq(_flow_innov(obs_index)) / (sq(math::max(_params.flow_innov_gate, 1.0f)) * _flow_innov_var(obs_index));
		if (obs_index == 0) {
			_optflow_test_ratio = test_ratio;

		} else {
			_optflow_test_ratio = math::max(_optflow_test_ratio, test_ratio);
		}

		bool flow_fail = false;

		if (test_ratio > 1.0f) {
			flow_fail = true;
			_innov_check_fail_status.value |= (1 << (obs_index + 10));

		} else {
			_innov_check_fail_status.value &= ~(1 << (obs_index + 10));

		}

		if (flow_fail) {
			continue;

		}

		const bool is_fused = measurementUpdate(Kfusion, Hfusion, _flow_innov(obs_index));

		if (obs_index == 0) {
			_fault_status.flags.bad_optflow_X = !is_fused;

		} else if (obs_index == 1) {
			_fault_status.flags.bad_optflow_Y = !is_fused;
		}

		if (is_fused) {
			_time_last_of_fuse = _time_last_imu;
		}
	}
}

// calculate optical flow body angular rate compensation
// returns false if bias corrected body rate data is unavailable
bool Ekf::calcOptFlowBodyRateComp()
{
	// reset the accumulators if the time interval is too large
	if (_delta_time_of > 1.0f) {
		_imu_del_ang_of.setZero();
		_delta_time_of = 0.0f;
		return false;
	}

	bool is_body_rate_comp_available = false;
	const bool use_flow_sensor_gyro = PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(0)) && PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(1)) && PX4_ISFINITE(_flow_sample_delayed.gyro_xyz(2));

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
float Ekf::calcOptFlowMeasVar()
{
	// calculate the observation noise variance - scaling noise linearly across flow quality range
	const float R_LOS_best = fmaxf(_params.flow_noise, 0.05f);
	const float R_LOS_worst = fmaxf(_params.flow_noise_qual_min, 0.05f);

	// calculate a weighting that varies between 1 when flow quality is best and 0 when flow quality is worst
	float weighting = (255.0f - (float)_params.flow_qual_min);

	if (weighting >= 1.0f) {
		weighting = math::constrain(((float)_flow_sample_delayed.quality - (float)_params.flow_qual_min) / weighting, 0.0f,
					    1.0f);

	} else {
		weighting = 0.0f;
	}

	// take the weighted average of the observation noise for the best and wort flow quality
	const float R_LOS = sq(R_LOS_best * weighting + R_LOS_worst * (1.0f - weighting));

	return R_LOS;
}
