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
#include "mathlib.h"

void Ekf::fuseVelPosHeight()
{
	bool fuse_map[6] = {}; // map of booleans true when [VN,VE,VD,PN,PE,PD] observations are available
	bool innov_check_pass_map[6] = {}; // true when innovations consistency checks pass for [VN,VE,VD,PN,PE,PD] observations
	float R[6] = {}; // observation variances for [VN,VE,VD,PN,PE,PD]
	float gate_size[6] = {}; // innovation consistency check gate sizes for [VN,VE,VD,PN,PE,PD] observations
	float Kfusion[24] = {}; // Kalman gain vector for any single observation - sequential fusion is used

	// calculate innovations, innovations gate sizes and observation variances
	if (_fuse_hor_vel) {
		fuse_map[0] = fuse_map[1] = true;
		// horizontal velocity innovations
		_vel_pos_innov[0] = _state.vel(0) - _gps_sample_delayed.vel(0);
		_vel_pos_innov[1] = _state.vel(1) - _gps_sample_delayed.vel(1);
		// observation variance - use receiver reported accuracy with parameter setting the minimum value
		R[0] = fmaxf(_params.gps_vel_noise, 0.01f);
		R[0] = fmaxf(R[0], _gps_sample_delayed.sacc);
		R[0] = R[0] * R[0];
		R[1] = R[0];
		// innovation gate sizes
		gate_size[0] = fmaxf(_params.vel_innov_gate, 1.0f);
		gate_size[1] = gate_size[0];
	}

	if (_fuse_vert_vel) {
		fuse_map[2] = true;
		// vertical velocity innovation
		_vel_pos_innov[2] = _state.vel(2) - _gps_sample_delayed.vel(2);
		// observation variance - use receiver reported accuracy with parameter setting the minimum value
		R[2] = fmaxf(_params.gps_vel_noise, 0.01f);
		// use scaled horizontal speed accuracy assuming typical ratio of VDOP/HDOP
		R[2] = 1.5f * fmaxf(R[2], _gps_sample_delayed.sacc);
		R[2] = R[2] * R[2];
		// innovation gate size
		gate_size[2] = fmaxf(_params.vel_innov_gate, 1.0f);
	}

	if (_fuse_pos) {
		fuse_map[3] = fuse_map[4] = true;

		// Calculate innovations and observation variance depending on type of observations
		// being used
		if (_control_status.flags.gps) {
			// we are using GPS measurements
			float lower_limit = fmaxf(_params.gps_pos_noise, 0.01f);
			float upper_limit = fmaxf(_params.pos_noaid_noise, lower_limit);
			R[3] = math::constrain(_gps_sample_delayed.hacc, lower_limit, upper_limit);
			_vel_pos_innov[3] = _state.pos(0) - _gps_sample_delayed.pos(0);
			_vel_pos_innov[4] = _state.pos(1) - _gps_sample_delayed.pos(1);

			// innovation gate size
			gate_size[3] = fmaxf(_params.posNE_innov_gate, 1.0f);


		} else if (_control_status.flags.ev_pos) {
			// we are using external vision measurements
			R[3] = fmaxf(_ev_sample_delayed.posErr, 0.01f);
			_vel_pos_innov[3] = _state.pos(0) - _ev_sample_delayed.posNED(0);
			_vel_pos_innov[4] = _state.pos(1) - _ev_sample_delayed.posNED(1);

			// innovation gate size
			gate_size[3] = fmaxf(_params.ev_innov_gate, 1.0f);

		} else {
			// No observations - use a static position to constrain drift
			if (_control_status.flags.in_air && _control_status.flags.tilt_align) {
				R[3] = fmaxf(_params.pos_noaid_noise, _params.gps_pos_noise);
			} else {
				R[3] = 0.5f;
			}
			_vel_pos_innov[3] = _state.pos(0) - _last_known_posNE(0);
			_vel_pos_innov[4] = _state.pos(1) - _last_known_posNE(1);

			// glitch protection is not required so set gate to a large value
			gate_size[3] = 100.0f;

		}

		// convert North position noise to variance
		R[3] = R[3] * R[3];

		// copy North axis values to East axis
		R[4] = R[3];
		gate_size[4] = gate_size[3];

	}

	if (_fuse_height) {
		if (_control_status.flags.baro_hgt) {
			fuse_map[5] = true;
			// vertical position innovation - baro measurement has opposite sign to earth z axis
			_vel_pos_innov[5] = _state.pos(2) + _baro_sample_delayed.hgt - _baro_hgt_offset - _hgt_sensor_offset;
			// observation variance - user parameter defined
			R[5] = fmaxf(_params.baro_noise, 0.01f);
			R[5] = R[5] * R[5];
			// innovation gate size
			gate_size[5] = fmaxf(_params.baro_innov_gate, 1.0f);

		} else if (_control_status.flags.gps_hgt) {
			fuse_map[5] = true;
			// vertical position innovation - gps measurement has opposite sign to earth z axis
			_vel_pos_innov[5] = _state.pos(2) + _gps_sample_delayed.hgt - _gps_alt_ref - _hgt_sensor_offset;
			// observation variance - receiver defined and parameter limited
			// use scaled horizontal position accuracy assuming typical ratio of VDOP/HDOP
			float lower_limit = fmaxf(_params.gps_pos_noise, 0.01f);
			float upper_limit = fmaxf(_params.pos_noaid_noise, lower_limit);
			R[5] = 1.5f * math::constrain(_gps_sample_delayed.vacc, lower_limit, upper_limit);
			R[5] = R[5] * R[5];
			// innovation gate size
			gate_size[5] = fmaxf(_params.baro_innov_gate, 1.0f);

		} else if (_control_status.flags.rng_hgt && (_R_rng_to_earth_2_2 > 0.7071f)) {
			fuse_map[5] = true;
			// use range finder with tilt correction
			_vel_pos_innov[5] = _state.pos(2) - (-math::max(_range_sample_delayed.rng * _R_rng_to_earth_2_2,
							     _params.rng_gnd_clearance)) - _hgt_sensor_offset;
			// observation variance - user parameter defined
			R[5] = fmaxf((sq(_params.range_noise) + sq(_params.range_noise_scaler * _range_sample_delayed.rng)) * sq(_R_rng_to_earth_2_2), 0.01f);
			// innovation gate size
			gate_size[5] = fmaxf(_params.range_innov_gate, 1.0f);
		} else if (_control_status.flags.ev_hgt) {
			fuse_map[5] = true;
			// calculate the innovation assuming the external vision observaton is in local NED frame
			_vel_pos_innov[5] = _state.pos(2) - _ev_sample_delayed.posNED(2);
			// observation variance - defined externally
			R[5] = fmaxf(_ev_sample_delayed.posErr, 0.01f);
			R[5] = R[5] * R[5];
			// innovation gate size
			gate_size[5] = fmaxf(_params.ev_innov_gate, 1.0f);
		}

	}

	// calculate innovation test ratios
	for (unsigned obs_index = 0; obs_index < 6; obs_index++) {
		if (fuse_map[obs_index]) {
			// compute the innovation variance SK = HPH + R
			unsigned state_index = obs_index + 4;	// we start with vx and this is the 4. state
			_vel_pos_innov_var[obs_index] = P[state_index][state_index] + R[obs_index];
			// Compute the ratio of innovation to gate size
			_vel_pos_test_ratio[obs_index] = sq(_vel_pos_innov[obs_index]) / (sq(gate_size[obs_index]) *
							 _vel_pos_innov_var[obs_index]);
		}
	}

	// check position, velocity and height innovations
	// treat 3D velocity, 2D position and height as separate sensors
	// always pass position checks if using synthetic position measurements or yet to complete tilt alignment
	// always pass height checks if yet to complete tilt alignment
	bool vel_check_pass = (_vel_pos_test_ratio[0] <= 1.0f) && (_vel_pos_test_ratio[1] <= 1.0f)
			      && (_vel_pos_test_ratio[2] <= 1.0f);
	innov_check_pass_map[2] = innov_check_pass_map[1] = innov_check_pass_map[0] = vel_check_pass;
	bool pos_check_pass = ((_vel_pos_test_ratio[3] <= 1.0f) && (_vel_pos_test_ratio[4] <= 1.0f)) || !_control_status.flags.tilt_align;
	innov_check_pass_map[4] = innov_check_pass_map[3] = pos_check_pass;
	innov_check_pass_map[5] = (_vel_pos_test_ratio[5] <= 1.0f) || !_control_status.flags.tilt_align;

	// record the successful velocity fusion event
	if (vel_check_pass && _fuse_hor_vel) {
		_time_last_vel_fuse = _time_last_imu;
		_innov_check_fail_status.flags.reject_vel_NED = false;
	} else if (!vel_check_pass) {
		_innov_check_fail_status.flags.reject_vel_NED = true;
	}

	// record the successful position fusion event
	if (pos_check_pass && _fuse_pos) {
		_time_last_pos_fuse = _time_last_imu;
		_innov_check_fail_status.flags.reject_pos_NE = false;
	} else if (!pos_check_pass) {
		_innov_check_fail_status.flags.reject_pos_NE = true;
	}

	// record the successful height fusion event
	if (innov_check_pass_map[5] && _fuse_height) {
		_time_last_hgt_fuse = _time_last_imu;
		_innov_check_fail_status.flags.reject_pos_D = false;
	} else if (!innov_check_pass_map[5]) {
		_innov_check_fail_status.flags.reject_pos_D = true;
	}

	for (unsigned obs_index = 0; obs_index < 6; obs_index++) {
		// skip fusion if not requested or checks have failed
		if (!fuse_map[obs_index] || !innov_check_pass_map[obs_index]) {
			continue;
		}

		unsigned state_index = obs_index + 4;	// we start with vx and this is the 4. state

		// calculate kalman gain K = PHS, where S = 1/innovation variance
		for (int row = 0; row < _k_num_states; row++) {
			Kfusion[row] = P[row][state_index] / _vel_pos_innov_var[obs_index];
		}

		// update covarinace matrix via Pnew = (I - KH)P
		float KHP[_k_num_states][_k_num_states];
		for (unsigned row = 0; row < _k_num_states; row++) {
			for (unsigned column = 0; column < _k_num_states; column++) {
				KHP[row][column] = Kfusion[row] * P[state_index][column];
			}
		}

		// if the covariance correction will result in a negative variance, then
		// the covariance marix is unhealthy and must be corrected
		bool healthy = true;
		for (int i = 0; i < _k_num_states; i++) {
			if (P[i][i] < KHP[i][i]) {
				// zero rows and columns
				zeroRows(P,i,i);
				zeroCols(P,i,i);

				//flag as unhealthy
				healthy = false;

				// update individual measurement health status
				if (obs_index == 0) {
					_fault_status.flags.bad_vel_N = true;
				} else if (obs_index == 1) {
					_fault_status.flags.bad_vel_E = true;
				} else if (obs_index == 2) {
					_fault_status.flags.bad_vel_D = true;
				} else if (obs_index == 3) {
					_fault_status.flags.bad_pos_N = true;
				} else if (obs_index == 4) {
					_fault_status.flags.bad_pos_E = true;
				} else if (obs_index == 5) {
					_fault_status.flags.bad_pos_D = true;
				}
			} else {
				// update individual measurement health status
				if (obs_index == 0) {
					_fault_status.flags.bad_vel_N = false;
				} else if (obs_index == 1) {
					_fault_status.flags.bad_vel_E = false;
				} else if (obs_index == 2) {
					_fault_status.flags.bad_vel_D = false;
				} else if (obs_index == 3) {
					_fault_status.flags.bad_pos_N = false;
				} else if (obs_index == 4) {
					_fault_status.flags.bad_pos_E = false;
				} else if (obs_index == 5) {
					_fault_status.flags.bad_pos_D = false;
				}
			}
		}

		// only apply covariance and state corrrections if healthy
		if (healthy) {
			// apply the covariance corrections
			for (unsigned row = 0; row < _k_num_states; row++) {
				for (unsigned column = 0; column < _k_num_states; column++) {
					P[row][column] = P[row][column] - KHP[row][column];
				}
			}

			// correct the covariance marix for gross errors
			fixCovarianceErrors();

			// apply the state corrections
			fuse(Kfusion, _vel_pos_innov[obs_index]);
		}
	}
}
