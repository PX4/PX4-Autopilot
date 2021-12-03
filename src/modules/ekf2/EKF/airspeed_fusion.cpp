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

void Ekf::fuseAirspeed()
{
	const float &vn = _state.vel(0); // Velocity in north direction
	const float &ve = _state.vel(1); // Velocity in east direction
	const float &vd = _state.vel(2); // Velocity in downwards direction
	const float &vwn = _state.wind_vel(0); // Wind speed in north direction
	const float &vwe = _state.wind_vel(1); // Wind speed in east direction

	// Variance for true airspeed measurement - (m/sec)^2
	const float R_TAS = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) *
			       math::constrain(_airspeed_sample_delayed.eas2tas, 0.9f, 10.0f));

	// determine if we need the airspeed fusion to correct states other than wind
	const bool update_wind_only = !_control_status.flags.wind_dead_reckoning;

	// Intermediate variables
	const float HK0 = vn - vwn;
	const float HK1 = ve - vwe;
	const float HK2 = ecl::powf(HK0, 2) + ecl::powf(HK1, 2) + ecl::powf(vd, 2);
	const float v_tas_pred = sqrtf(HK2); // predicted airspeed

	//const float HK3 = powf(HK2, -1.0F/2.0F);
	if (v_tas_pred < 1.0f) {
		// calculation can be badly conditioned for very low airspeed values so don't fuse this time
		return;
	}

	const float HK3 = 1.0f / v_tas_pred;
	const float HK4 = HK0*HK3;
	const float HK5 = HK1*HK3;
	const float HK6 = 1.0F/HK2;
	const float HK7 = HK0*P(4,6) - HK0*P(6,22) + HK1*P(5,6) - HK1*P(6,23) + P(6,6)*vd;
	const float HK8 = HK1*P(5,23);
	const float HK9 = HK0*P(4,5) - HK0*P(5,22) + HK1*P(5,5) - HK8 + P(5,6)*vd;
	const float HK10 = HK1*HK6;
	const float HK11 = HK0*P(4,22);
	const float HK12 = HK0*P(4,4) - HK1*P(4,23) + HK1*P(4,5) - HK11 + P(4,6)*vd;
	const float HK13 = HK0*HK6;
	const float HK14 = -HK0*P(22,23) + HK0*P(4,23) - HK1*P(23,23) + HK8 + P(6,23)*vd;
	const float HK15 = -HK0*P(22,22) - HK1*P(22,23) + HK1*P(5,22) + HK11 + P(6,22)*vd;
	//const float HK16 = HK3/(-HK10*HK14 + HK10*HK9 + HK12*HK13 - HK13*HK15 + HK6*HK7*vd + R_TAS);

	// innovation variance - check for badly conditioned calculation
	_airspeed_innov_var = (-HK10 * HK14 + HK10 * HK9 + HK12 * HK13 - HK13 * HK15 + HK6 * HK7 * vd + R_TAS);

	if (_airspeed_innov_var < R_TAS) { //
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

	const float HK16 = HK3 / _airspeed_innov_var;
	_fault_status.flags.bad_airspeed = false;

	// Observation Jacobians
	SparseVector24f<4,5,6,22,23> Hfusion;
	Hfusion.at<4>() = HK4;
	Hfusion.at<5>() = HK5;
	Hfusion.at<6>() = HK3*vd;
	Hfusion.at<22>() = -HK4;
	Hfusion.at<23>() = -HK5;

	Vector24f Kfusion; // Kalman gain vector

	if (!update_wind_only) {
		// we have no other source of aiding, so use airspeed measurements to correct states
		for (unsigned row = 0; row <= 3; row++) {
			Kfusion(row) = HK16*(HK0*P(4,row) - HK0*P(row,22) + HK1*P(5,row) - HK1*P(row,23) + P(6,row)*vd);
		}

		Kfusion(4) = HK12*HK16;
		Kfusion(5) = HK16*HK9;
		Kfusion(6) = HK16*HK7;

		for (unsigned row = 7; row <= 21; row++) {
			Kfusion(row) = HK16*(HK0*P(4,row) - HK0*P(row,22) + HK1*P(5,row) - HK1*P(row,23) + P(6,row)*vd);
		}
	}

	Kfusion(22) = HK15*HK16;
	Kfusion(23) = HK14*HK16;

	// Calculate measurement innovation
	_airspeed_innov = v_tas_pred - _airspeed_sample_delayed.true_airspeed;

	// Compute the ratio of innovation to gate size
	_tas_test_ratio = sq(_airspeed_innov) / (sq(fmaxf(_params.tas_innov_gate, 1.0f)) * _airspeed_innov_var);

	// If the innovation consistency check fails then don't fuse the sample and indicate bad airspeed health
	if (_tas_test_ratio > 1.0f) {
		_innov_check_fail_status.flags.reject_airspeed = true;
		return;

	} else {
		_innov_check_fail_status.flags.reject_airspeed = false;
	}

	const bool is_fused = measurementUpdate(Kfusion, Hfusion, _airspeed_innov);

	_fault_status.flags.bad_airspeed = !is_fused;

	if (is_fused) {
		_time_last_arsp_fuse = _time_last_imu;
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
	const float euler_yaw = shouldUse321RotationSequence(_R_to_earth)
				? getEuler321Yaw(_state.quat_nominal)
				: getEuler312Yaw(_state.quat_nominal);

	// estimate wind using zero sideslip assumption and airspeed measurement if airspeed available
	_state.wind_vel(0) = _state.vel(0) - _airspeed_sample_delayed.true_airspeed * cosf(euler_yaw);
	_state.wind_vel(1) = _state.vel(1) - _airspeed_sample_delayed.true_airspeed * sinf(euler_yaw);

	resetWindCovarianceUsingAirspeed();

	_time_last_arsp_fuse = _time_last_imu;
}

void Ekf::resetWindToZero()
{
	// If we don't have an airspeed measurement, then assume the wind is zero
	_state.wind_vel.setZero();
	// start with a small initial uncertainty to improve the initial estimate
	P.uncorrelateCovarianceSetVariance<2>(22, _params.initial_wind_uncertainty);
}
