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
 * @file airspeed_fusion.cpp
 * airspeed fusion methods.
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include "python/ekf_derivation/generated/compute_airspeed_h_and_k.h"
#include "python/ekf_derivation/generated/compute_airspeed_innov_and_innov_var.h"

#include <mathlib/mathlib.h>

void Ekf::updateAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source_1d_s &airspeed) const
{
	// reset flags
	resetEstimatorAidStatusFlags(airspeed);

	// Variance for true airspeed measurement - (m/sec)^2
	const float R = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) *
			   math::constrain(airspeed_sample.eas2tas, 0.9f, 10.0f));

	float innov = 0.f;
	float innov_var = 0.f;
	sym::ComputeAirspeedInnovAndInnovVar(getStateAtFusionHorizonAsVector(), P, airspeed_sample.true_airspeed, R, FLT_EPSILON, &innov, &innov_var);

	airspeed.observation = airspeed_sample.true_airspeed;
	airspeed.observation_variance = R;
	airspeed.innovation = innov;
	airspeed.innovation_variance = innov_var;

	airspeed.fusion_enabled = _control_status.flags.fuse_aspd;

	airspeed.timestamp_sample = airspeed_sample.time_us;

	const float innov_gate = fmaxf(_params.tas_innov_gate, 1.f);
	setEstimatorAidStatusTestRatio(airspeed, innov_gate);
}

void Ekf::fuseAirspeed(estimator_aid_source_1d_s &airspeed)
{
	if (airspeed.innovation_rejected) {
		return;
	}

	// determine if we need the airspeed fusion to correct states other than wind
	const bool update_wind_only = !_control_status.flags.wind_dead_reckoning;

	const float innov_var = airspeed.innovation_variance;

	if (innov_var < airspeed.observation_variance || innov_var < FLT_EPSILON) {
		// Reset the estimator covariance matrix
		// if we are getting aiding from other sources, warn and reset the wind states and covariances only
		const char *action_string = nullptr;

		if (update_wind_only) {
			resetWindUsingAirspeed();
			action_string = "wind";

		} else {
			initialiseCovariance();
			_state.wind_vel.setZero();
			action_string = "full";
		}

		ECL_ERR("airspeed badly conditioned - %s covariance reset", action_string);

		_fault_status.flags.bad_airspeed = true;

		return;
	}

	_fault_status.flags.bad_airspeed = false;

	Vector24f H; // Observation jacobian
	Vector24f K; // Kalman gain vector

	sym::ComputeAirspeedHAndK(getStateAtFusionHorizonAsVector(), P, innov_var, FLT_EPSILON, &H, &K);

	SparseVector24f<4,5,6,22,23> H_sparse(H);

	if (update_wind_only) {
		for (unsigned row = 0; row <= 21; row++) {
			K(row) = 0.f;
		}
	}

	const bool is_fused = measurementUpdate(K, H_sparse, airspeed.innovation);

	airspeed.fused = is_fused;
	_fault_status.flags.bad_airspeed = !is_fused;

	if (is_fused) {
		airspeed.time_last_fuse = _imu_sample_delayed.time_us;
	}
}

float Ekf::getTrueAirspeed() const
{
	return (_state.vel - Vector3f(_state.wind_vel(0), _state.wind_vel(1), 0.f)).norm();
}

void Ekf::resetWind()
{
	if (_control_status.flags.fuse_aspd) {
		resetWindUsingAirspeed();

	} else {
		resetWindToZero();
	}
}

/*
 * Reset the wind states using the current airspeed measurement, ground relative nav velocity, yaw angle and assumption of zero sideslip
*/
void Ekf::resetWindUsingAirspeed()
{
	const float euler_yaw = getEulerYaw(_R_to_earth);

	// estimate wind using zero sideslip assumption and airspeed measurement if airspeed available
	_state.wind_vel(0) = _state.vel(0) - _airspeed_sample_delayed.true_airspeed * cosf(euler_yaw);
	_state.wind_vel(1) = _state.vel(1) - _airspeed_sample_delayed.true_airspeed * sinf(euler_yaw);

	resetWindCovarianceUsingAirspeed();

	_aid_src_airspeed.time_last_fuse = _imu_sample_delayed.time_us;
}

void Ekf::resetWindToZero()
{
	// If we don't have an airspeed measurement, then assume the wind is zero
	_state.wind_vel.setZero();
	// start with a small initial uncertainty to improve the initial estimate
	P.uncorrelateCovarianceSetVariance<2>(22, _params.initial_wind_uncertainty);
}
