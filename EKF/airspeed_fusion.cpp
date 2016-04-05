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
 *
 * @author Carl Olsson <carlolsson.co@gmail.com>
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */
#include "ekf.h"
#include "mathlib.h"

void Ekf::fuseAirspeed()
{
	// Initialize variables
	float vn; // Velocity in north direction
	float ve; // Velocity in east direction
	float vd; // Velocity in downwards direction
	float vwn; // Wind speed in north direction
	float vwe; // Wind speed in east direction
	float v_tas_pred; // Predicted measurement
	float EAS2TAS = 1.0f; // Where can I get this from?
	float R_TAS = sq(math::constrain(_params.eas_noise, 0.5f, 5.0f) * math::constrain(EAS2TAS, 0.9f, 10.0f)); // Variance for true airspeed measurement - (m/sec)^2
	float SH_TAS[3] = {}; // Varialbe used to optimise calculations of measurement jacobian
	float H_TAS[24] = {}; // Observation Jacobian
	float SK_TAS[2] = {}; // Varialbe used to optimise calculations of the Kalman gain vector
	float Kfusion[24] = {}; // Kalman gain vector

	// Copy required states to local variable names
	vn = _state.vel(0);
	ve = _state.vel(1);
	vd = _state.vel(2);
	vwn = _state.wind_vel(0);
	vwe = _state.wind_vel(1);

	// Calculate the predicted airspeed
	v_tas_pred = sqrtf((ve - vwe) * (ve - vwe) + (vn - vwn) * (vn - vwn) + vd * vd);

	// Perform fusion of True Airspeed measurement
	if (v_tas_pred > 1.0f) {
		// Calculate the observation jacobian
		// intermediate variable from algebraic optimisation
		SH_TAS[0] = 1 / v_tas_pred;
		SH_TAS[1] = (SH_TAS[0] * (2 * ve - 2 * vwe)) / 2.0f;
		SH_TAS[2] = (SH_TAS[0] * (2 * vn - 2 * vwn)) / 2.0f;

		for (uint8_t i = 0; i < _k_num_states; i++) { H_TAS[i] = 0.0f; }

		H_TAS[3] = SH_TAS[2];
		H_TAS[4] = SH_TAS[1];
		H_TAS[5] = vd * SH_TAS[0];
		H_TAS[22] = -SH_TAS[2];
		H_TAS[23] = -SH_TAS[1];
	
		// We don't want to update the innovation variance if the calculation is ill conditioned
		float _airspeed_innov_var_temp = (R_TAS + SH_TAS[2] * (P[3][3] * SH_TAS[2] + P[4][3] * SH_TAS[1] - P[22][3] * SH_TAS[2] - P[23][3] * SH_TAS[1] + P[5][3] * vd *SH_TAS[0]) + SH_TAS[1] * (P[3][4] * SH_TAS[2] + P[4][4] * SH_TAS[1] - P[22][4] * SH_TAS[2] - P[23][4] * SH_TAS[1] + P[5][4] * vd *SH_TAS[0]) - SH_TAS[2] * (P[3][22] * SH_TAS[2] + P[4][22] * SH_TAS[1] - P[22][22] * SH_TAS[2] - P[23][22] * SH_TAS[1] + P[5][22] * vd *SH_TAS[0]) - SH_TAS[1] * (P[3][23] * SH_TAS[2] + P[4][23] * SH_TAS[1] - P[22][23] * SH_TAS[2] - P[23][23] * SH_TAS[1] + P[5][23] * vd *SH_TAS[0]) + vd *SH_TAS[0] * (P[3][5] * SH_TAS[2] + P[4][5] * SH_TAS[1] - P[22][5] * SH_TAS[2] - P[23][5] * SH_TAS[1] + P[5][5] * vd *SH_TAS[0]));
		if (_airspeed_innov_var_temp >= R_TAS) { // Check for badly conditioned calculation
            SK_TAS[0] = 1.0f / _airspeed_innov_var_temp;
            _fault_status.bad_airspeed = false;
        } else { // Reset the estimator
        	_fault_status.bad_airspeed = true;
            initialiseCovariance();
            return;
        }
		SK_TAS[1] = SH_TAS[1];

		Kfusion[0] = SK_TAS[0]*(P[0][3]*SH_TAS[2] - P[0][22]*SH_TAS[2] + P[0][4]*SK_TAS[1] - P[0][23]*SK_TAS[1] + P[0][5]*vd*SH_TAS[0]);
		Kfusion[1] = SK_TAS[0]*(P[1][3]*SH_TAS[2] - P[1][22]*SH_TAS[2] + P[1][4]*SK_TAS[1] - P[1][23]*SK_TAS[1] + P[1][5]*vd*SH_TAS[0]);
		Kfusion[2] = SK_TAS[0]*(P[2][3]*SH_TAS[2] - P[2][22]*SH_TAS[2] + P[2][4]*SK_TAS[1] - P[2][23]*SK_TAS[1] + P[2][5]*vd*SH_TAS[0]);
		Kfusion[3] = SK_TAS[0]*(P[3][3]*SH_TAS[2] - P[3][22]*SH_TAS[2] + P[3][4]*SK_TAS[1] - P[3][23]*SK_TAS[1] + P[3][5]*vd*SH_TAS[0]);
		Kfusion[4] = SK_TAS[0]*(P[4][3]*SH_TAS[2] - P[4][22]*SH_TAS[2] + P[4][4]*SK_TAS[1] - P[4][23]*SK_TAS[1] + P[4][5]*vd*SH_TAS[0]);
		Kfusion[5] = SK_TAS[0]*(P[5][3]*SH_TAS[2] - P[5][22]*SH_TAS[2] + P[5][4]*SK_TAS[1] - P[5][23]*SK_TAS[1] + P[5][5]*vd*SH_TAS[0]);
		Kfusion[6] = SK_TAS[0]*(P[6][3]*SH_TAS[2] - P[6][22]*SH_TAS[2] + P[6][4]*SK_TAS[1] - P[6][23]*SK_TAS[1] + P[6][5]*vd*SH_TAS[0]);
		Kfusion[7] = SK_TAS[0]*(P[7][3]*SH_TAS[2] - P[7][22]*SH_TAS[2] + P[7][4]*SK_TAS[1] - P[7][23]*SK_TAS[1] + P[7][5]*vd*SH_TAS[0]);
		Kfusion[8] = SK_TAS[0]*(P[8][3]*SH_TAS[2] - P[8][22]*SH_TAS[2] + P[8][4]*SK_TAS[1] - P[8][23]*SK_TAS[1] + P[8][5]*vd*SH_TAS[0]);
		Kfusion[9] = SK_TAS[0]*(P[9][3]*SH_TAS[2] - P[9][22]*SH_TAS[2] + P[9][4]*SK_TAS[1] - P[9][23]*SK_TAS[1] + P[9][5]*vd*SH_TAS[0]);
		Kfusion[10] = SK_TAS[0]*(P[10][3]*SH_TAS[2] - P[10][22]*SH_TAS[2] + P[10][4]*SK_TAS[1] - P[10][23]*SK_TAS[1] + P[10][5]*vd*SH_TAS[0]);
		Kfusion[11] = SK_TAS[0]*(P[11][3]*SH_TAS[2] - P[11][22]*SH_TAS[2] + P[11][4]*SK_TAS[1] - P[11][23]*SK_TAS[1] + P[11][5]*vd*SH_TAS[0]);
		Kfusion[12] = SK_TAS[0]*(P[12][3]*SH_TAS[2] - P[12][22]*SH_TAS[2] + P[12][4]*SK_TAS[1] - P[12][23]*SK_TAS[1] + P[12][5]*vd*SH_TAS[0]);
		Kfusion[13] = SK_TAS[0]*(P[13][3]*SH_TAS[2] - P[13][22]*SH_TAS[2] + P[13][4]*SK_TAS[1] - P[13][23]*SK_TAS[1] + P[13][5]*vd*SH_TAS[0]);
		Kfusion[14] = SK_TAS[0]*(P[14][3]*SH_TAS[2] - P[14][22]*SH_TAS[2] + P[14][4]*SK_TAS[1] - P[14][23]*SK_TAS[1] + P[14][5]*vd*SH_TAS[0]);
		Kfusion[15] = SK_TAS[0]*(P[15][3]*SH_TAS[2] - P[15][22]*SH_TAS[2] + P[15][4]*SK_TAS[1] - P[15][23]*SK_TAS[1] + P[15][5]*vd*SH_TAS[0]);
		Kfusion[22] = SK_TAS[0]*(P[22][3]*SH_TAS[2] - P[22][22]*SH_TAS[2] + P[22][4]*SK_TAS[1] - P[22][23]*SK_TAS[1] + P[22][5]*vd*SH_TAS[0]);
		Kfusion[23] = SK_TAS[0]*(P[23][3]*SH_TAS[2] - P[23][22]*SH_TAS[2] + P[23][4]*SK_TAS[1] - P[23][23]*SK_TAS[1] + P[23][5]*vd*SH_TAS[0]);

		// Only update the magnetometer states if we are airborne and using 3D mag fusion
		if(_control_status.flags.mag_3D && _control_status.flags.in_air){
			Kfusion[16] = SK_TAS[0]*(P[16][3]*SH_TAS[2] - P[16][22]*SH_TAS[2] + P[16][4]*SK_TAS[1] - P[16][23]*SK_TAS[1] + P[16][5]*vd*SH_TAS[0]);
			Kfusion[17] = SK_TAS[0]*(P[17][3]*SH_TAS[2] - P[17][22]*SH_TAS[2] + P[17][4]*SK_TAS[1] - P[17][23]*SK_TAS[1] + P[17][5]*vd*SH_TAS[0]);
			Kfusion[18] = SK_TAS[0]*(P[18][3]*SH_TAS[2] - P[18][22]*SH_TAS[2] + P[18][4]*SK_TAS[1] - P[18][23]*SK_TAS[1] + P[18][5]*vd*SH_TAS[0]);
			Kfusion[19] = SK_TAS[0]*(P[19][3]*SH_TAS[2] - P[19][22]*SH_TAS[2] + P[19][4]*SK_TAS[1] - P[19][23]*SK_TAS[1] + P[19][5]*vd*SH_TAS[0]);
			Kfusion[20] = SK_TAS[0]*(P[20][3]*SH_TAS[2] - P[20][22]*SH_TAS[2] + P[20][4]*SK_TAS[1] - P[20][23]*SK_TAS[1] + P[20][5]*vd*SH_TAS[0]);
			Kfusion[21] = SK_TAS[0]*(P[21][3]*SH_TAS[2] - P[21][22]*SH_TAS[2] + P[21][4]*SK_TAS[1] - P[21][23]*SK_TAS[1] + P[21][5]*vd*SH_TAS[0]);
		}
		else{
			for (int i = 16; i <= 21; i++)
			{
				Kfusion[i] = 0.0f;
			}
		}


		// calculate measurement innovation
		_airspeed_innov = v_tas_pred - _airspeed_sample_delayed.true_airspeed;

		// Calculate the innovation variance
		_airspeed_innov_var = 1.0f / SK_TAS[0];

		// Compute the ratio of innovation to gate size
		_tas_test_ratio = sq(_airspeed_innov) / (sq(fmaxf(_params.tas_innov_gate, 1.0f)) * _airspeed_innov_var);

		if (_tas_test_ratio < 1.0f)
		{
			// by definition the angle error state is zero at the fusion time
			_state.ang_error.setZero();

			// Fuse airspeed measurement
			fuse(Kfusion, _airspeed_innov);

			// correct the nominal quaternion
			Quaternion dq;
			dq.from_axis_angle(_state.ang_error);
			_state.quat_nominal = dq * _state.quat_nominal;
			_state.quat_nominal.normalize();

			for (unsigned row = 0; row < _k_num_states; row++) {
				for (unsigned column = 0; column < _k_num_states; column++) { // Here it will be a lot of zeros, should optimize that...
					KH[row][column] = Kfusion[row] * H_TAS[column];
				}
			}

			for (unsigned row = 0; row < _k_num_states; row++) {
				for (unsigned column = 0; column < _k_num_states; column++) {
					for (unsigned i = 0; i < _k_num_states; i++) { // Check if this is correct matrix multiplication!
						KHP[row][column] += KH[row][i] * P[i][column];
					}
				}
			}

			for (unsigned row = 0; row < _k_num_states; row++) {
				for (unsigned column = 0; column < _k_num_states; column++) {
					P[row][column] = P[row][column] - KHP[row][column];
				}
			}
		}

		makeSymmetrical(); 
		limitCov();
	}
}