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
#include <mathlib/mathlib.h>

void Ekf::updateAirspeed(const airspeedSample &airspeed_sample, estimator_aid_source_1d_s &airspeed) const
{
	// reset flags
	resetEstimatorAidStatusFlags(airspeed);

	const float vn = _state.vel(0); // Velocity in north direction
	const float ve = _state.vel(1); // Velocity in east direction
	const float vd = _state.vel(2); // Velocity in downwards direction
	const float vwn = _state.wind_vel(0); // Wind speed in north direction
	const float vwe = _state.wind_vel(1); // Wind speed in east direction

	// Variance for true airspeed measurement - (m/sec)^2
	const float R_TAS = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) *
			       math::constrain(airspeed_sample.eas2tas, 0.9f, 10.0f));

	// Intermediate variables
	const float IV0 = ve - vwe;
	const float IV1 = vn - vwn;
	const float IV2 = (IV0)*(IV0) + (IV1)*(IV1) + (vd)*(vd);

	const float predicted_airspeed = sqrtf(IV2);

	if (fabsf(predicted_airspeed) < FLT_EPSILON) {
		return;
	}

	const float IV3 = 1.0F/(IV2);
	const float IV4 = IV0*P(5,23);
	const float IV5 = IV0*IV3;
	const float IV6 = IV1*P(4,22);
	const float IV7 = IV1*IV3;

	const float innov_var = IV3*vd*(IV0*P(5,6) - IV0*P(6,23) + IV1*P(4,6) - IV1*P(6,22) + P(6,6)*vd) - IV5*(-IV0*P(23,23) - IV1*P(22,23) + IV1*P(4,23) + IV4 + P(6,23)*vd) + IV5*(IV0*P(5,5) + IV1*P(4,5) - IV1*P(5,22) - IV4 + P(5,6)*vd) - IV7*(-IV0*P(22,23) + IV0*P(5,22) - IV1*P(22,22) + IV6 + P(6,22)*vd) + IV7*(-IV0*P(4,23) + IV0*P(4,5) + IV1*P(4,4) - IV6 + P(4,6)*vd) + R_TAS;

	airspeed.observation = airspeed_sample.true_airspeed;
	airspeed.observation_variance = R_TAS;
	airspeed.innovation = predicted_airspeed - airspeed.observation;
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

	const float vn = _state.vel(0); // Velocity in north direction
	const float ve = _state.vel(1); // Velocity in east direction
	const float vd = _state.vel(2); // Velocity in downwards direction
	const float vwn = _state.wind_vel(0); // Wind speed in north direction
	const float vwe = _state.wind_vel(1); // Wind speed in east direction

	// determine if we need the airspeed fusion to correct states other than wind
	const bool update_wind_only = !_control_status.flags.wind_dead_reckoning;

	// Intermediate variables
	const float HK0 = vn - vwn;
	const float HK1 = ve - vwe;
	const float HK2 = sqrtf((HK0)*(HK0) + (HK1)*(HK1) + (vd)*(vd));

	const float predicted_airspeed = HK2;

	if (predicted_airspeed < 1.0f) {
		// calculation can be badly conditioned for very low airspeed values so don't fuse this time
		return;
	}

	const float HK3 = 1.0F/(HK2);
	const float HK4 = HK0*HK3;
	const float HK5 = HK1*HK3;
	const float HK6 = HK3*vd;
	const float HK7 = -HK0*HK3;
	const float HK8 = -HK1*HK3;

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

	const float HK9 = 1.0F/(innov_var);

	_fault_status.flags.bad_airspeed = false;

	// Observation Jacobians
	SparseVector24f<4,5,6,22,23> Hfusion;
	Hfusion.at<4>() = HK4;
	Hfusion.at<5>() = HK5;
	Hfusion.at<6>() = HK6;
	Hfusion.at<22>() = HK7;
	Hfusion.at<23>() = HK8;

	Vector24f Kfusion; // Kalman gain vector

	if (!update_wind_only) {
		// we have no other source of aiding, so use airspeed measurements to correct states
		for (unsigned row = 0; row <= 21; row++) {
			Kfusion(row) = HK9*(HK4*P(row,4) + HK5*P(row,5) + HK6*P(row,6) + HK7*P(row,22) + HK8*P(row,23));
		}
	}

	Kfusion(22) = HK9*(HK4*P(4,22) + HK5*P(5,22) + HK6*P(6,22) + HK7*P(22,22) + HK8*P(22,23));
	Kfusion(23) = HK9*(HK4*P(4,23) + HK5*P(5,23) + HK6*P(6,23) + HK7*P(22,23) + HK8*P(23,23));

	const bool is_fused = measurementUpdate(Kfusion, Hfusion, airspeed.innovation);

	airspeed.fused = is_fused;
	_fault_status.flags.bad_airspeed = !is_fused;

	if (is_fused) {
		airspeed.time_last_fuse = _time_last_imu;
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

	_aid_src_airspeed.time_last_fuse = _time_last_imu;
}

void Ekf::resetWindToZero()
{
	// If we don't have an airspeed measurement, then assume the wind is zero
	_state.wind_vel.setZero();
	// start with a small initial uncertainty to improve the initial estimate
	P.uncorrelateCovarianceSetVariance<2>(22, _params.initial_wind_uncertainty);
}
