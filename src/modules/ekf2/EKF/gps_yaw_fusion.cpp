/****************************************************************************
 *
 *   Copyright (c) 2018-2023 PX4 Development Team. All rights reserved.
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
 * @file gps_yaw_fusion.cpp
 * Definition of functions required to use yaw obtained from GPS dual antenna measurements.
 * Equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include <ekf_derivation/generated/compute_gnss_yaw_pred_innov_var_and_h.h>

#include <mathlib/mathlib.h>
#include <cstdlib>

void Ekf::updateGpsYaw(const gnssSample &gps_sample)
{
	if (PX4_ISFINITE(gps_sample.yaw)) {

		auto &gnss_yaw = _aid_src_gnss_yaw;
		resetEstimatorAidStatus(gnss_yaw);

		// initially populate for estimator_aid_src_gnss_yaw logging

		// calculate the observed yaw angle of antenna array, converting a from body to antenna yaw measurement
		const float measured_hdg = wrap_pi(gps_sample.yaw + gps_sample.yaw_offset);

		const float yaw_acc = PX4_ISFINITE(gps_sample.yaw_acc) ? gps_sample.yaw_acc : 0.f;
		const float R_YAW = sq(fmaxf(yaw_acc, _params.gps_heading_noise));

		float heading_pred;
		float heading_innov_var;

		{
		VectorState H;
		sym::ComputeGnssYawPredInnovVarAndH(_state.vector(), P, gps_sample.yaw_offset, R_YAW, FLT_EPSILON, &heading_pred, &heading_innov_var, &H);
		}

		gnss_yaw.observation = measured_hdg;
		gnss_yaw.observation_variance = R_YAW;
		gnss_yaw.innovation = wrap_pi(heading_pred - measured_hdg);
		gnss_yaw.innovation_variance = heading_innov_var;

		gnss_yaw.timestamp_sample = gps_sample.time_us;

		const float innov_gate = math::max(_params.heading_innov_gate, 1.0f);
		setEstimatorAidStatusTestRatio(gnss_yaw, innov_gate);
	}
}

void Ekf::fuseGpsYaw(float antenna_yaw_offset)
{
	auto &gnss_yaw = _aid_src_gnss_yaw;

	if (gnss_yaw.innovation_rejected) {
		_innov_check_fail_status.flags.reject_yaw = true;
		return;
	}

	if (!PX4_ISFINITE(antenna_yaw_offset)) {
		antenna_yaw_offset = 0.f;
	}

	VectorState H;

	{
	float heading_pred;
	float heading_innov_var;

	// Note: we recompute innov and innov_var because it doesn't cost much more than just computing H
	// making a separate function just for H uses more flash space without reducing CPU load significantly
	sym::ComputeGnssYawPredInnovVarAndH(_state.vector(), P, antenna_yaw_offset, gnss_yaw.observation_variance, FLT_EPSILON, &heading_pred, &heading_innov_var, &H);
	}

	// check if the innovation variance calculation is badly conditioned
	if (gnss_yaw.innovation_variance < gnss_yaw.observation_variance) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR("GPS yaw numerical error - covariance reset");
		return;
	}

	_fault_status.flags.bad_hdg = false;
	_innov_check_fail_status.flags.reject_yaw = false;

	_gnss_yaw_signed_test_ratio_lpf.update(matrix::sign(gnss_yaw.innovation) * gnss_yaw.test_ratio);

	if ((fabsf(_gnss_yaw_signed_test_ratio_lpf.getState()) > 0.2f)
		&& !_control_status.flags.in_air && isTimedOut(gnss_yaw.time_last_fuse, (uint64_t)1e6)) {

		// A constant large signed test ratio is a sign of wrong gyro bias
		// Reset the yaw gyro variance to converge faster and avoid
		// being stuck on a previous bad estimate
		resetGyroBiasZCov();
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	VectorState Kfusion = P * H / gnss_yaw.innovation_variance;

	const bool is_fused = measurementUpdate(Kfusion, H, gnss_yaw.observation_variance, gnss_yaw.innovation);
	_fault_status.flags.bad_hdg = !is_fused;
	gnss_yaw.fused = is_fused;

	if (is_fused) {
		_time_last_heading_fuse = _time_delayed_us;
		gnss_yaw.time_last_fuse = _time_delayed_us;
	}
}

bool Ekf::resetYawToGps(const float gnss_yaw, const float gnss_yaw_offset)
{
	// define the predicted antenna array vector and rotate into earth frame
	const Vector3f ant_vec_bf = {cosf(gnss_yaw_offset), sinf(gnss_yaw_offset), 0.0f};
	const Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf;

	// check if antenna array vector is within 30 degrees of vertical and therefore unable to provide a reliable heading
	if (fabsf(ant_vec_ef(2)) > cosf(math::radians(30.0f)))  {
		return false;
	}

	// GPS yaw measurement is alreday compensated for antenna offset in the driver
	const float measured_yaw = gnss_yaw;

	const float yaw_variance = sq(fmaxf(_params.gps_heading_noise, 1.e-2f));
	resetQuatStateYaw(measured_yaw, yaw_variance);

	_aid_src_gnss_yaw.time_last_fuse = _time_delayed_us;
	_gnss_yaw_signed_test_ratio_lpf.reset(0.f);

	return true;
}
