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
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include <ecl.h>
#include <mathlib/mathlib.h>

/**
 * Update the EKF state with velocity and position measurements sequentially. [(vx vy) (vz) (x y) (z)]
 *
 * @param innov  	Input		[vx vy vz x y z]
 * @param innov_gate	Input		[vxy vz xy z]
 * @param obs_var	Input		[vx vy vz x y z]
 * @param fuse_mask	Input/Output	[vxy vz xy z]
 * 					Specify which innovation components should be fused,
 * 					components that do not pass innovations checks will be set to zero
 * @param innov_var	Ouput		[vx vy vz x y z]
 * @param test_ratio	Output		[vxy vz xy z]
 */
void Ekf::fuseVelPosHeightSeq(const float (&innov)[6], const float (&innov_gate)[4],
				 const float (obs_var)[6], bool (&fuse_mask)[4],
				 float (&innov_var)[6], float (&test_ratio)[4])
{
	// check position, velocity and height innovations sequentially and if checks are passed fuse it
	// treat 2D horizintal velocity, vertical velocity, 2D horizontal position and vertical height as separate sensors
	// At the moment we still fuse velocity as 3D measurement, but this should be split in the future

	// horizontal and vertical velocity
	if(fuse_mask[HVEL] && fuse_mask[VVEL]){
		innov_var[0] = P[4][4] + obs_var[0];
		innov_var[1] = P[5][5] + obs_var[1];
		test_ratio[HVEL] = fmaxf( sq(innov[0]) / (sq(innov_gate[HVEL]) * innov_var[0]),
					sq(innov[1]) / (sq(innov_gate[HVEL]) * innov_var[1]));

		innov_var[2] = P[6][6] + obs_var[2];
		test_ratio[VVEL] = sq(innov[2]) / (sq(innov_gate[VVEL]) * innov_var[2]);

		bool innov_check_pass = (test_ratio[HVEL] <= 1.0f) && (test_ratio[VVEL] <= 1.0f);
		if (innov_check_pass) {
			_time_last_vel_fuse = _time_last_imu;
			_innov_check_fail_status.flags.reject_vel_NED = false;

			// fuse the horizontal and vertical velocity measurements
			fuseVelPosHeight(innov[0],innov_var[0],0);
			fuseVelPosHeight(innov[1],innov_var[1],1);
			fuseVelPosHeight(innov[2],innov_var[2],2);

		}else{
			fuse_mask[HVEL] = fuse_mask[VVEL] = false;
			_innov_check_fail_status.flags.reject_vel_NED = true;
		}
	}

	// horizontal position
	if(fuse_mask[HPOS]){
		innov_var[3] = P[7][7] + obs_var[3];
		innov_var[4] = P[8][8] + obs_var[4];
		test_ratio[HPOS] = fmaxf( sq(innov[3]) / (sq(innov_gate[HPOS]) * innov_var[3]),
					sq(innov[4]) / (sq(innov_gate[HPOS]) * innov_var[4]));

		bool innov_check_pass = (test_ratio[HPOS] <= 1.0f) || !_control_status.flags.tilt_align;
		if (innov_check_pass) {
			if (!_fuse_hpos_as_odom) {
				_time_last_pos_fuse = _time_last_imu;

			} else {
				_time_last_delpos_fuse = _time_last_imu;
			}
			_innov_check_fail_status.flags.reject_pos_NE = false;

			// fuse the horizontal position measurements
			fuseVelPosHeight(innov[3],innov_var[3],3);
			fuseVelPosHeight(innov[4],innov_var[4],4);

		}else{
			fuse_mask[HPOS] = false;
			_innov_check_fail_status.flags.reject_pos_NE = true;
		}
	}

	// vertical position
	if(fuse_mask[VPOS]){
		innov_var[5] = P[9][9] + obs_var[5];
		test_ratio[VPOS] = sq(innov[5]) / (sq(innov_gate[VPOS]) * innov_var[5]);

		bool innov_check_pass = (test_ratio[VPOS] <= 1.0f) || !_control_status.flags.tilt_align;
		if (innov_check_pass) {
			_time_last_hgt_fuse = _time_last_imu;
			_innov_check_fail_status.flags.reject_pos_D = false;

			// fuse the horizontal position measurements
			fuseVelPosHeight(innov[5],innov_var[5],5);

		}else{
			fuse_mask[VPOS] = false;
			_innov_check_fail_status.flags.reject_pos_D = true;
		}
	}

}

// Helper function that fuses a single velocity or position measurement
void Ekf::fuseVelPosHeight(const float innov, const float innov_var, const int obs_index)
{
	float Kfusion[24] = {}; // Kalman gain vector for any single observation - sequential fusion is used.
	unsigned state_index = obs_index + 4;	// we start with vx and this is the 4. state

	// calculate kalman gain K = PHS, where S = 1/innovation variance
	for (int row = 0; row < _k_num_states; row++) {
		Kfusion[row] = P[row][state_index] / innov_var;
	}

	float KHP[_k_num_states][_k_num_states];

	for (unsigned row = 0; row < _k_num_states; row++) {
		for (unsigned column = 0; column < _k_num_states; column++) {
			KHP[row][column] = Kfusion[row] * P[state_index][column];
		}
	}

	// if the covariance correction will result in a negative variance, then
	// the covariance matrix is unhealthy and must be corrected
	bool healthy = true;

	for (int i = 0; i < _k_num_states; i++) {
		if (P[i][i] < KHP[i][i]) {
			// zero rows and columns
			zeroRows(P, i, i);
			zeroCols(P, i, i);

			healthy = false;

			setVelPosFaultStatus(obs_index,true);

		} else {
			setVelPosFaultStatus(obs_index,false);
		}
	}


	// only apply covariance and state corrections if healthy
	if (healthy) {
		// apply the covariance corrections
		for (unsigned row = 0; row < _k_num_states; row++) {
			for (unsigned column = 0; column < _k_num_states; column++) {
				P[row][column] = P[row][column] - KHP[row][column];
			}
		}

		// correct the covariance matrix for gross errors
		fixCovarianceErrors();

		// apply the state corrections
		fuse(Kfusion, innov);

	}
}

void Ekf::setVelPosFaultStatus(const int index, const bool status)
{
	if (index == 0) {
		_fault_status.flags.bad_vel_N = status;

	} else if (index == 1) {
		_fault_status.flags.bad_vel_E = status;

	} else if (index == 2) {
		_fault_status.flags.bad_vel_D = status;

	} else if (index == 3) {
		_fault_status.flags.bad_pos_N = status;

	} else if (index == 4) {
		_fault_status.flags.bad_pos_E = status;

	} else if (index == 5) {
		_fault_status.flags.bad_pos_D = status;
	}
}
