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
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include <ecl.h>
#include <mathlib/mathlib.h>

void Ekf::fuseSideslip()
{
	float SH_BETA[13] = {}; // Variable used to optimise calculations of measurement jacobian
	float H_BETA[24] = {}; // Observation Jacobian
	float SK_BETA[8] = {}; // Variable used to optimise calculations of the Kalman gain vector
	float Kfusion[24] = {}; // Kalman gain vector
	float R_BETA = _params.beta_noise;

	// get latest estimated orientation
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	// get latest velocity in earth frame
	const float vn = _state.vel(0);
	const float ve = _state.vel(1);
	const float vd = _state.vel(2);

	// get latest wind velocity in earth frame
	const float vwn = _state.wind_vel(0);
	const float vwe = _state.wind_vel(1);

	// relative wind velocity in earth frame
	Vector3f rel_wind;
	rel_wind(0) = vn - vwn;
	rel_wind(1) = ve - vwe;
	rel_wind(2) = vd;

	const Dcmf earth_to_body = quat_to_invrotmat(_state.quat_nominal);

	// rotate into body axes
	rel_wind = earth_to_body * rel_wind;

	// perform fusion of assumed sideslip  = 0
	if (rel_wind.norm() > 7.0f) {
		// Calculate the observation jacobians

		// intermediate variable from algebraic optimisation
		SH_BETA[0] = (vn - vwn)*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) - vd*(2.0f*q0*q2 - 2.0f*q1*q3) + (ve - vwe)*(2.0f*q0*q3 + 2.0f*q1*q2);

		if (fabsf(SH_BETA[0]) <= 1e-9f) {
			return;
		}

		SH_BETA[1] = (ve - vwe)*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + vd*(2.0f*q0*q1 + 2.0f*q2*q3) - (vn - vwn)*(2.0f*q0*q3 - 2.0f*q1*q2);
		SH_BETA[2] = vn - vwn;
		SH_BETA[3] = ve - vwe;
		SH_BETA[4] = 1.0f/sq(SH_BETA[0]);
		SH_BETA[5] = 1.0f/SH_BETA[0];
		SH_BETA[6] = SH_BETA[5]*(sq(q0) - sq(q1) + sq(q2) - sq(q3));
		SH_BETA[7] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
		SH_BETA[8] = 2.0f*q0*SH_BETA[3] - 2.0f*q3*SH_BETA[2] + 2.0f*q1*vd;
		SH_BETA[9] = 2.0f*q0*SH_BETA[2] + 2.0f*q3*SH_BETA[3] - 2.0f*q2*vd;
		SH_BETA[10] = 2.0f*q2*SH_BETA[2] - 2.0f*q1*SH_BETA[3] + 2.0f*q0*vd;
		SH_BETA[11] = 2.0f*q1*SH_BETA[2] + 2.0f*q2*SH_BETA[3] + 2.0f*q3*vd;
		SH_BETA[12] = 2.0f*q0*q3;

		H_BETA[0] = SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9];
		H_BETA[1] = SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11];
		H_BETA[2] = SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10];
		H_BETA[3] = - SH_BETA[5]*SH_BETA[9] - SH_BETA[1]*SH_BETA[4]*SH_BETA[8];
		H_BETA[4] = - SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) - SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
		H_BETA[5] = SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2);
		H_BETA[6] = SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3);
		H_BETA[22] = SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
		H_BETA[23] = SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2) - SH_BETA[6];

		for (uint8_t i = 7; i <= 21; i++) {
			H_BETA[i] = 0.0f;
		}

		// determine if we need the sideslip fusion to correct states other than wind
		bool update_wind_only = !_is_wind_dead_reckoning;

		// intermediate variables - note SK_BETA[0] is 1/(innovation variance)
		_beta_innov_var = (R_BETA - (SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7])*(P(22,4)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,4)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,4)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,4)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,4)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,4)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,4)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,4)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,4)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) + (SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7])*(P(22,22)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,22)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,22)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,22)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,22)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,22)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,22)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,22)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,22)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) + (SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2))*(P(22,5)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,5)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,5)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,5)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,5)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,5)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,5)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,5)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,5)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) - (SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2))*(P(22,23)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,23)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,23)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,23)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,23)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,23)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,23)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,23)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,23)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) + (SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9])*(P(22,0)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,0)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,0)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,0)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,0)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,0)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,0)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,0)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,0)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) + (SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11])*(P(22,1)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,1)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,1)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,1)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,1)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,1)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,1)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,1)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,1)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) + (SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10])*(P(22,2)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,2)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,2)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,2)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,2)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,2)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,2)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,2)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,2)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) - (SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8])*(P(22,3)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,3)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,3)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,3)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,3)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,3)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,3)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,3)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,3)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))) + (SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))*(P(22,6)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) - P(4,6)*(SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7]) + P(5,6)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) - P(23,6)*(SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2)) + P(0,6)*(SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9]) + P(1,6)*(SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11]) + P(2,6)*(SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10]) - P(3,6)*(SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8]) + P(6,6)*(SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3))));
		if (_beta_innov_var >= R_BETA) {
			SK_BETA[0] = 1.0f / _beta_innov_var;
			_fault_status.flags.bad_sideslip = false;

		} else { // Reset the estimator
			_fault_status.flags.bad_sideslip = true;

			// if we are getting aiding from other sources, warn and reset the wind states and covariances only
			if (update_wind_only) {
				resetWindStates();
				resetWindCovariance();
				ECL_ERR_TIMESTAMPED("synthetic sideslip fusion badly conditioned - wind covariance reset");

			} else {
				initialiseCovariance();
				_state.wind_vel.setZero();
				ECL_ERR_TIMESTAMPED("synthetic sideslip fusion badly conditioned - full covariance reset");
			}

			return;
		}

		SK_BETA[1] = SH_BETA[5]*(SH_BETA[12] - 2.0f*q1*q2) + SH_BETA[1]*SH_BETA[4]*SH_BETA[7];
		SK_BETA[2] = SH_BETA[6] - SH_BETA[1]*SH_BETA[4]*(SH_BETA[12] + 2.0f*q1*q2);
		SK_BETA[3] = SH_BETA[5]*(2.0f*q0*q1 + 2.0f*q2*q3) + SH_BETA[1]*SH_BETA[4]*(2.0f*q0*q2 - 2.0f*q1*q3);
		SK_BETA[4] = SH_BETA[5]*SH_BETA[10] - SH_BETA[1]*SH_BETA[4]*SH_BETA[11];
		SK_BETA[5] = SH_BETA[5]*SH_BETA[8] - SH_BETA[1]*SH_BETA[4]*SH_BETA[9];
		SK_BETA[6] = SH_BETA[5]*SH_BETA[11] + SH_BETA[1]*SH_BETA[4]*SH_BETA[10];
		SK_BETA[7] = SH_BETA[5]*SH_BETA[9] + SH_BETA[1]*SH_BETA[4]*SH_BETA[8];

		// Calculate Kalman gains
		if (update_wind_only) {
			// If we are getting aiding from other sources, then don't allow the sideslip fusion to affect the non-windspeed states
			for (unsigned row = 0; row <= 21; row++) {
				Kfusion[row] = 0.0f;

			}

		} else {
		        Kfusion[0] = SK_BETA[0]*(P(0,0)*SK_BETA[5] + P(0,1)*SK_BETA[4] - P(0,4)*SK_BETA[1] + P(0,5)*SK_BETA[2] + P(0,2)*SK_BETA[6] + P(0,6)*SK_BETA[3] - P(0,3)*SK_BETA[7] + P(0,22)*SK_BETA[1] - P(0,23)*SK_BETA[2]);
		        Kfusion[1] = SK_BETA[0]*(P(1,0)*SK_BETA[5] + P(1,1)*SK_BETA[4] - P(1,4)*SK_BETA[1] + P(1,5)*SK_BETA[2] + P(1,2)*SK_BETA[6] + P(1,6)*SK_BETA[3] - P(1,3)*SK_BETA[7] + P(1,22)*SK_BETA[1] - P(1,23)*SK_BETA[2]);
		        Kfusion[2] = SK_BETA[0]*(P(2,0)*SK_BETA[5] + P(2,1)*SK_BETA[4] - P(2,4)*SK_BETA[1] + P(2,5)*SK_BETA[2] + P(2,2)*SK_BETA[6] + P(2,6)*SK_BETA[3] - P(2,3)*SK_BETA[7] + P(2,22)*SK_BETA[1] - P(2,23)*SK_BETA[2]);
		        Kfusion[3] = SK_BETA[0]*(P(3,0)*SK_BETA[5] + P(3,1)*SK_BETA[4] - P(3,4)*SK_BETA[1] + P(3,5)*SK_BETA[2] + P(3,2)*SK_BETA[6] + P(3,6)*SK_BETA[3] - P(3,3)*SK_BETA[7] + P(3,22)*SK_BETA[1] - P(3,23)*SK_BETA[2]);
		        Kfusion[4] = SK_BETA[0]*(P(4,0)*SK_BETA[5] + P(4,1)*SK_BETA[4] - P(4,4)*SK_BETA[1] + P(4,5)*SK_BETA[2] + P(4,2)*SK_BETA[6] + P(4,6)*SK_BETA[3] - P(4,3)*SK_BETA[7] + P(4,22)*SK_BETA[1] - P(4,23)*SK_BETA[2]);
		        Kfusion[5] = SK_BETA[0]*(P(5,0)*SK_BETA[5] + P(5,1)*SK_BETA[4] - P(5,4)*SK_BETA[1] + P(5,5)*SK_BETA[2] + P(5,2)*SK_BETA[6] + P(5,6)*SK_BETA[3] - P(5,3)*SK_BETA[7] + P(5,22)*SK_BETA[1] - P(5,23)*SK_BETA[2]);
		        Kfusion[6] = SK_BETA[0]*(P(6,0)*SK_BETA[5] + P(6,1)*SK_BETA[4] - P(6,4)*SK_BETA[1] + P(6,5)*SK_BETA[2] + P(6,2)*SK_BETA[6] + P(6,6)*SK_BETA[3] - P(6,3)*SK_BETA[7] + P(6,22)*SK_BETA[1] - P(6,23)*SK_BETA[2]);
		        Kfusion[7] = SK_BETA[0]*(P(7,0)*SK_BETA[5] + P(7,1)*SK_BETA[4] - P(7,4)*SK_BETA[1] + P(7,5)*SK_BETA[2] + P(7,2)*SK_BETA[6] + P(7,6)*SK_BETA[3] - P(7,3)*SK_BETA[7] + P(7,22)*SK_BETA[1] - P(7,23)*SK_BETA[2]);
		        Kfusion[8] = SK_BETA[0]*(P(8,0)*SK_BETA[5] + P(8,1)*SK_BETA[4] - P(8,4)*SK_BETA[1] + P(8,5)*SK_BETA[2] + P(8,2)*SK_BETA[6] + P(8,6)*SK_BETA[3] - P(8,3)*SK_BETA[7] + P(8,22)*SK_BETA[1] - P(8,23)*SK_BETA[2]);
		        Kfusion[9] = SK_BETA[0]*(P(9,0)*SK_BETA[5] + P(9,1)*SK_BETA[4] - P(9,4)*SK_BETA[1] + P(9,5)*SK_BETA[2] + P(9,2)*SK_BETA[6] + P(9,6)*SK_BETA[3] - P(9,3)*SK_BETA[7] + P(9,22)*SK_BETA[1] - P(9,23)*SK_BETA[2]);
		        Kfusion[10] = SK_BETA[0]*(P(10,0)*SK_BETA[5] + P(10,1)*SK_BETA[4] - P(10,4)*SK_BETA[1] + P(10,5)*SK_BETA[2] + P(10,2)*SK_BETA[6] + P(10,6)*SK_BETA[3] - P(10,3)*SK_BETA[7] + P(10,22)*SK_BETA[1] - P(10,23)*SK_BETA[2]);
		        Kfusion[11] = SK_BETA[0]*(P(11,0)*SK_BETA[5] + P(11,1)*SK_BETA[4] - P(11,4)*SK_BETA[1] + P(11,5)*SK_BETA[2] + P(11,2)*SK_BETA[6] + P(11,6)*SK_BETA[3] - P(11,3)*SK_BETA[7] + P(11,22)*SK_BETA[1] - P(11,23)*SK_BETA[2]);
		        Kfusion[12] = SK_BETA[0]*(P(12,0)*SK_BETA[5] + P(12,1)*SK_BETA[4] - P(12,4)*SK_BETA[1] + P(12,5)*SK_BETA[2] + P(12,2)*SK_BETA[6] + P(12,6)*SK_BETA[3] - P(12,3)*SK_BETA[7] + P(12,22)*SK_BETA[1] - P(12,23)*SK_BETA[2]);
		        Kfusion[13] = SK_BETA[0]*(P(13,0)*SK_BETA[5] + P(13,1)*SK_BETA[4] - P(13,4)*SK_BETA[1] + P(13,5)*SK_BETA[2] + P(13,2)*SK_BETA[6] + P(13,6)*SK_BETA[3] - P(13,3)*SK_BETA[7] + P(13,22)*SK_BETA[1] - P(13,23)*SK_BETA[2]);
		        Kfusion[14] = SK_BETA[0]*(P(14,0)*SK_BETA[5] + P(14,1)*SK_BETA[4] - P(14,4)*SK_BETA[1] + P(14,5)*SK_BETA[2] + P(14,2)*SK_BETA[6] + P(14,6)*SK_BETA[3] - P(14,3)*SK_BETA[7] + P(14,22)*SK_BETA[1] - P(14,23)*SK_BETA[2]);
		        Kfusion[15] = SK_BETA[0]*(P(15,0)*SK_BETA[5] + P(15,1)*SK_BETA[4] - P(15,4)*SK_BETA[1] + P(15,5)*SK_BETA[2] + P(15,2)*SK_BETA[6] + P(15,6)*SK_BETA[3] - P(15,3)*SK_BETA[7] + P(15,22)*SK_BETA[1] - P(15,23)*SK_BETA[2]);

			// Only update the magnetometer states if we are airborne and using 3D mag fusion
			if (_control_status.flags.mag_3D && _control_status.flags.in_air) {
				Kfusion[16] = SK_BETA[0]*(P(16,0)*SK_BETA[5] + P(16,1)*SK_BETA[4] - P(16,4)*SK_BETA[1] + P(16,5)*SK_BETA[2] + P(16,2)*SK_BETA[6] + P(16,6)*SK_BETA[3] - P(16,3)*SK_BETA[7] + P(16,22)*SK_BETA[1] - P(16,23)*SK_BETA[2]);
				Kfusion[17] = SK_BETA[0]*(P(17,0)*SK_BETA[5] + P(17,1)*SK_BETA[4] - P(17,4)*SK_BETA[1] + P(17,5)*SK_BETA[2] + P(17,2)*SK_BETA[6] + P(17,6)*SK_BETA[3] - P(17,3)*SK_BETA[7] + P(17,22)*SK_BETA[1] - P(17,23)*SK_BETA[2]);
				Kfusion[18] = SK_BETA[0]*(P(18,0)*SK_BETA[5] + P(18,1)*SK_BETA[4] - P(18,4)*SK_BETA[1] + P(18,5)*SK_BETA[2] + P(18,2)*SK_BETA[6] + P(18,6)*SK_BETA[3] - P(18,3)*SK_BETA[7] + P(18,22)*SK_BETA[1] - P(18,23)*SK_BETA[2]);
				Kfusion[19] = SK_BETA[0]*(P(19,0)*SK_BETA[5] + P(19,1)*SK_BETA[4] - P(19,4)*SK_BETA[1] + P(19,5)*SK_BETA[2] + P(19,2)*SK_BETA[6] + P(19,6)*SK_BETA[3] - P(19,3)*SK_BETA[7] + P(19,22)*SK_BETA[1] - P(19,23)*SK_BETA[2]);
				Kfusion[20] = SK_BETA[0]*(P(20,0)*SK_BETA[5] + P(20,1)*SK_BETA[4] - P(20,4)*SK_BETA[1] + P(20,5)*SK_BETA[2] + P(20,2)*SK_BETA[6] + P(20,6)*SK_BETA[3] - P(20,3)*SK_BETA[7] + P(20,22)*SK_BETA[1] - P(20,23)*SK_BETA[2]);
				Kfusion[21] = SK_BETA[0]*(P(21,0)*SK_BETA[5] + P(21,1)*SK_BETA[4] - P(21,4)*SK_BETA[1] + P(21,5)*SK_BETA[2] + P(21,2)*SK_BETA[6] + P(21,6)*SK_BETA[3] - P(21,3)*SK_BETA[7] + P(21,22)*SK_BETA[1] - P(21,23)*SK_BETA[2]);

			} else {
				for (int i = 16; i <= 21; i++) {
					Kfusion[i] = 0.0f;

				}
			}
		}

		Kfusion[22] = SK_BETA[0]*(P(22,0)*SK_BETA[5] + P(22,1)*SK_BETA[4] - P(22,4)*SK_BETA[1] + P(22,5)*SK_BETA[2] + P(22,2)*SK_BETA[6] + P(22,6)*SK_BETA[3] - P(22,3)*SK_BETA[7] + P(22,22)*SK_BETA[1] - P(22,23)*SK_BETA[2]);
		Kfusion[23] = SK_BETA[0]*(P(23,0)*SK_BETA[5] + P(23,1)*SK_BETA[4] - P(23,4)*SK_BETA[1] + P(23,5)*SK_BETA[2] + P(23,2)*SK_BETA[6] + P(23,6)*SK_BETA[3] - P(23,3)*SK_BETA[7] + P(23,22)*SK_BETA[1] - P(23,23)*SK_BETA[2]);

		// Calculate predicted sideslip angle and innovation using small angle approximation
		_beta_innov = rel_wind(1) / rel_wind(0);

		// Compute the ratio of innovation to gate size
		_beta_test_ratio = sq(_beta_innov) / (sq(fmaxf(_params.beta_innov_gate, 1.0f)) * _beta_innov_var);

		// if the innovation consistency check fails then don't fuse the sample and indicate bad beta health
		if (_beta_test_ratio > 1.0f) {
			_innov_check_fail_status.flags.reject_sideslip = true;
			return;

		} else {
			_innov_check_fail_status.flags.reject_sideslip = false;
		}

		// synthetic sideslip measurement sample has passed check so record it
		_time_last_beta_fuse = _time_last_imu;

		// apply covariance correction via P_new = (I -K*H)*P
		// first calculate expression for KHP
		// then calculate P - KHP
		matrix::SquareMatrix<float, _k_num_states> KHP;
		float KH[9];

		for (unsigned row = 0; row < _k_num_states; row++) {
			KH[0] = Kfusion[row] * H_BETA[0];
			KH[1] = Kfusion[row] * H_BETA[1];
			KH[2] = Kfusion[row] * H_BETA[2];
			KH[3] = Kfusion[row] * H_BETA[3];
			KH[4] = Kfusion[row] * H_BETA[4];
			KH[5] = Kfusion[row] * H_BETA[5];
			KH[6] = Kfusion[row] * H_BETA[6];
			KH[7] = Kfusion[row] * H_BETA[22];
			KH[8] = Kfusion[row] * H_BETA[23];

			for (unsigned column = 0; column < _k_num_states; column++) {
				float tmp = KH[0] * P(0,column);
				tmp += KH[1] * P(1,column);
				tmp += KH[2] * P(2,column);
				tmp += KH[3] * P(3,column);
				tmp += KH[4] * P(4,column);
				tmp += KH[5] * P(5,column);
				tmp += KH[6] * P(6,column);
				tmp += KH[7] * P(22,column);
				tmp += KH[8] * P(23,column);
				KHP(row,column) = tmp;
			}
		}

		// if the covariance correction will result in a negative variance, then
		// the covariance matrix is unhealthy and must be corrected
		bool healthy = true;
		_fault_status.flags.bad_sideslip = false;

		for (int i = 0; i < _k_num_states; i++) {
			if (P(i,i) < KHP(i,i)) {
				// zero rows and columns
				P.uncorrelateCovarianceSetVariance<1>(i, 0.0f);

				//flag as unhealthy
				healthy = false;

				// update individual measurement health status
				_fault_status.flags.bad_sideslip = true;
			}
		}

		// only apply covariance and state corrections if healthy
		if (healthy) {
			// apply the covariance corrections
			for (unsigned row = 0; row < _k_num_states; row++) {
				for (unsigned column = 0; column < _k_num_states; column++) {
					P(row,column) = P(row,column) - KHP(row,column);
				}
			}

			// correct the covariance matrix for gross errors
			fixCovarianceErrors(true);

			// apply the state corrections
			fuse(Kfusion, _beta_innov);
		}
	}
}
