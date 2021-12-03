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
 * @file sideslip_fusion.cpp
 * sideslip fusion methods.
 * equations generated using EKF/python/ekf_derivation/main.py
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

void Ekf::fuseSideslip()
{
	// get latest estimated orientation
	const float &q0 = _state.quat_nominal(0);
	const float &q1 = _state.quat_nominal(1);
	const float &q2 = _state.quat_nominal(2);
	const float &q3 = _state.quat_nominal(3);

	// get latest velocity in earth frame
	const float &vn = _state.vel(0);
	const float &ve = _state.vel(1);
	const float &vd = _state.vel(2);

	// get latest wind velocity in earth frame
	const float &vwn = _state.wind_vel(0);
	const float &vwe = _state.wind_vel(1);

	// calculate relative wind velocity in earth frame and rotate into body frame
	const Vector3f rel_wind_earth(vn - vwn, ve - vwe, vd);
	const Dcmf earth_to_body = quatToInverseRotMat(_state.quat_nominal);
	const Vector3f rel_wind_body = earth_to_body * rel_wind_earth;

	// perform fusion of assumed sideslip  = 0
	if (rel_wind_body.norm() > 7.0f) {
		const float R_BETA = sq(_params.beta_noise); // observation noise variance

		// determine if we need the sideslip fusion to correct states other than wind
		bool update_wind_only = !_control_status.flags.wind_dead_reckoning;

		// Intermediate Values
		const float HK0 = vn - vwn;
		const float HK1 = ve - vwe;
		const float HK2 = HK0*q0 + HK1*q3 - q2*vd;
		const float HK3 = q0*q2 - q1*q3;
		const float HK4 = 2*vd;
		const float HK5 = q0*q3;
		const float HK6 = q1*q2;
		const float HK7 = 2*HK5 + 2*HK6;
		const float HK8 = ecl::powf(q0, 2);
		const float HK9 = ecl::powf(q3, 2);
		const float HK10 = HK8 - HK9;
		const float HK11 = ecl::powf(q1, 2);
		const float HK12 = ecl::powf(q2, 2);
		const float HK13 = HK11 - HK12;
		const float HK14 = HK10 + HK13;
		const float HK15 = HK0*HK14 + HK1*HK7 - HK3*HK4;
		const float HK16 = 1.0F/HK15;
		const float HK17 = q0*q1 + q2*q3;
		const float HK18 = HK10 - HK11 + HK12;
		const float HK19 = HK16*(-2*HK0*(HK5 - HK6) + HK1*HK18 + HK17*HK4);
		const float HK20 = -HK0*q3 + HK1*q0 + q1*vd;
		const float HK21 = -HK19*HK2 + HK20;
		const float HK22 = 2*HK16;
		const float HK23 = HK0*q1 + HK1*q2 + q3*vd;
		const float HK24 = HK0*q2 - HK1*q1 + q0*vd;
		const float HK25 = -HK19*HK23 + HK24;
		const float HK26 = HK19*HK24 + HK23;
		const float HK27 = HK19*HK20 + HK2;
		const float HK28 = HK14*HK19 + 2*HK5 - 2*HK6;
		const float HK29 = HK16*HK28;
		const float HK30 = HK19*HK7;
		const float HK31 = HK17 + HK19*HK3;
		const float HK32 = HK13 + HK30 - HK8 + HK9;
		const float HK33 = 2*HK31;
		const float HK34 = 2*HK26;
		const float HK35 = 2*HK25;
		const float HK36 = 2*HK27;
		const float HK37 = 2*HK21;
		const float HK38 = HK28*P(0,22) - HK28*P(0,4) + HK32*P(0,23) - HK32*P(0,5) + HK33*P(0,6) + HK34*P(0,2) + HK35*P(0,1) - HK36*P(0,3) + HK37*P(0,0);
		const float HK39 = ecl::powf(HK15, -2);
		const float HK40 = -HK28*P(4,6) + HK28*P(6,22) - HK32*P(5,6) + HK32*P(6,23) + HK33*P(6,6) + HK34*P(2,6) + HK35*P(1,6) - HK36*P(3,6) + HK37*P(0,6);
		const float HK41 = HK32*P(5,23);
		const float HK42 = HK28*P(22,23) - HK28*P(4,23) + HK32*P(23,23) + HK33*P(6,23) + HK34*P(2,23) + HK35*P(1,23) - HK36*P(3,23) + HK37*P(0,23) - HK41;
		const float HK43 = HK32*HK39;
		const float HK44 = HK28*P(4,22);
		const float HK45 = HK28*P(22,22) + HK32*P(22,23) - HK32*P(5,22) + HK33*P(6,22) + HK34*P(2,22) + HK35*P(1,22) - HK36*P(3,22) + HK37*P(0,22) - HK44;
		const float HK46 = HK28*HK39;
		const float HK47 = -HK28*P(4,5) + HK28*P(5,22) - HK32*P(5,5) + HK33*P(5,6) + HK34*P(2,5) + HK35*P(1,5) - HK36*P(3,5) + HK37*P(0,5) + HK41;
		const float HK48 = -HK28*P(4,4) + HK32*P(4,23) - HK32*P(4,5) + HK33*P(4,6) + HK34*P(2,4) + HK35*P(1,4) - HK36*P(3,4) + HK37*P(0,4) + HK44;
		const float HK49 = HK28*P(2,22) - HK28*P(2,4) + HK32*P(2,23) - HK32*P(2,5) + HK33*P(2,6) + HK34*P(2,2) + HK35*P(1,2) - HK36*P(2,3) + HK37*P(0,2);
		const float HK50 = HK28*P(1,22) - HK28*P(1,4) + HK32*P(1,23) - HK32*P(1,5) + HK33*P(1,6) + HK34*P(1,2) + HK35*P(1,1) - HK36*P(1,3) + HK37*P(0,1);
		const float HK51 = HK28*P(3,22) - HK28*P(3,4) + HK32*P(3,23) - HK32*P(3,5) + HK33*P(3,6) + HK34*P(2,3) + HK35*P(1,3) - HK36*P(3,3) + HK37*P(0,3);
		//const float HK52 = HK16/(HK33*HK39*HK40 + HK34*HK39*HK49 + HK35*HK39*HK50 - HK36*HK39*HK51 + HK37*HK38*HK39 + HK42*HK43 - HK43*HK47 + HK45*HK46 - HK46*HK48 + R_BETA);

		// innovation variance
		_beta_innov_var = (HK33*HK39*HK40 + HK34*HK39*HK49 + HK35*HK39*HK50 - HK36*HK39*HK51 + HK37*HK38*HK39 + HK42*HK43 - HK43*HK47 + HK45*HK46 - HK46*HK48 + R_BETA);

		// Reset covariance and states if the calculation is badly conditioned
		if (_beta_innov_var < R_BETA) {
			_fault_status.flags.bad_sideslip = true;

			// if we are getting aiding from other sources, warn and reset the wind states and covariances only
			const char *action_string = nullptr;

			if (update_wind_only) {
				resetWind();
				action_string = "wind";

			} else {
				initialiseCovariance();
				_state.wind_vel.setZero();
				action_string = "full";
			}

			ECL_ERR("sideslip badly conditioned - %s covariance reset", action_string);

			return;
		}

		_fault_status.flags.bad_sideslip = false;
		const float HK52 = HK16 / _beta_innov_var;

		// Calculate predicted sideslip angle and innovation using small angle approximation
		_beta_innov = rel_wind_body(1) / rel_wind_body(0);

		// Compute the ratio of innovation to gate size
		_beta_test_ratio = sq(_beta_innov) / (sq(fmaxf(_params.beta_innov_gate, 1.0f)) * _beta_innov_var);

		// if the innovation consistency check fails then don't fuse the sample and indicate bad beta health
		if (_beta_test_ratio > 1.0f) {
			_innov_check_fail_status.flags.reject_sideslip = true;
			return;

		} else {
			_innov_check_fail_status.flags.reject_sideslip = false;
		}

		// Observation Jacobians
		SparseVector24f<0,1,2,3,4,5,6,22,23> Hfusion;
		Hfusion.at<0>() = HK21*HK22;
		Hfusion.at<1>() = HK22*HK25;
		Hfusion.at<2>() = HK22*HK26;
		Hfusion.at<3>() = -HK22*HK27;
		Hfusion.at<4>() = -HK29;
		Hfusion.at<5>() = HK16*(HK18 - HK30);
		Hfusion.at<6>() = HK22*HK31;
		Hfusion.at<22>() = HK29;
		Hfusion.at<23>() = HK16*HK32;

		// Calculate Kalman gains
		Vector24f Kfusion;

		if (!update_wind_only) {

			Kfusion(0) = HK38*HK52;
			Kfusion(1) = HK50*HK52;
			Kfusion(2) = HK49*HK52;
			Kfusion(3) = HK51*HK52;
			Kfusion(4) = HK48*HK52;
			Kfusion(5) = HK47*HK52;
			Kfusion(6) = HK40*HK52;

			for (unsigned row = 7; row <= 21; row++) {
				Kfusion(row) = HK52*(HK28*P(row,22) - HK28*P(4,row) + HK32*P(row,23) - HK32*P(5,row) + HK33*P(6,row) + HK34*P(2,row) + HK35*P(1,row) - HK36*P(3,row) + HK37*P(0,row));
			}

		}

		Kfusion(22) = HK45*HK52;
		Kfusion(23) = HK42*HK52;

		const bool is_fused = measurementUpdate(Kfusion, Hfusion, _beta_innov);

		_fault_status.flags.bad_sideslip = !is_fused;

		if (is_fused) {
			_time_last_beta_fuse = _time_last_imu;
		}
	}
}
