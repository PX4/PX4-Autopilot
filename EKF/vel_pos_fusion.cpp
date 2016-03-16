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
	bool fuse_map[6] = {}; // map of booelans true when [VN,VE,VD,PN,PE,PD] observations are available
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
		// horizontal position innovations
		_vel_pos_innov[3] = _state.pos(0) - _gps_sample_delayed.pos(0);
		_vel_pos_innov[4] = _state.pos(1) - _gps_sample_delayed.pos(1);

		// observation variance - user parameter defined
		// if we are in flight and not using GPS, then use a specific parameter
		if (!_control_status.flags.gps) {
			if (_control_status.flags.in_air) {
				R[3] = fmaxf(_params.pos_noaid_noise, _params.gps_pos_noise);

			} else {
				R[3] = _params.gps_pos_noise;
			}

		} else {
			float lower_limit = fmaxf(_params.gps_pos_noise, 0.01f);
			float upper_limit = fmaxf(_params.pos_noaid_noise, lower_limit);
			R[3] = math::constrain(_gps_sample_delayed.hacc, lower_limit, upper_limit);

		}

		R[3] = R[3] * R[3];
		R[4] = R[3];
		// innovation gate sizes
		gate_size[3] = fmaxf(_params.posNE_innov_gate, 1.0f);
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

		} else if (_control_status.flags.rng_hgt && (_R_prev(2, 2) > 0.7071f)) {
			fuse_map[5] = true;
			// use range finder with tilt correction
			_vel_pos_innov[5] = _state.pos(2) - (-math::max(_range_sample_delayed.rng * _R_prev(2, 2),
							     _params.rng_gnd_clearance));
			// observation variance - user parameter defined
			R[5] = fmaxf(_params.range_noise, 0.01f);
			R[5] = R[5] * R[5];
			// innovation gate size
			gate_size[5] = fmaxf(_params.range_innov_gate, 1.0f);
		}

	}

	// calculate innovation test ratios
	for (unsigned obs_index = 0; obs_index < 6; obs_index++) {
		if (fuse_map[obs_index]) {
			// compute the innovation variance SK = HPH + R
			unsigned state_index = obs_index + 3;	// we start with vx and this is the 4. state
			_vel_pos_innov_var[obs_index] = P[state_index][state_index] + R[obs_index];
			// Compute the ratio of innovation to gate size
			_vel_pos_test_ratio[obs_index] = sq(_vel_pos_innov[obs_index]) / (sq(gate_size[obs_index]) *
							 _vel_pos_innov_var[obs_index]);
		}
	}

	// check position, velocity and height innovations
	// treat 3D velocity, 2D position and height as separate sensors
	// always pass position checks if using synthetic position measurements
	bool vel_check_pass = (_vel_pos_test_ratio[0] <= 1.0f) && (_vel_pos_test_ratio[1] <= 1.0f)
			      && (_vel_pos_test_ratio[2] <= 1.0f);
	innov_check_pass_map[2] = innov_check_pass_map[1] = innov_check_pass_map[0] = vel_check_pass;
	bool using_synthetic_measurements = !_control_status.flags.gps && !_control_status.flags.opt_flow;
	bool pos_check_pass = ((_vel_pos_test_ratio[3] <= 1.0f) && (_vel_pos_test_ratio[4] <= 1.0f))
			      || using_synthetic_measurements;
	innov_check_pass_map[4] = innov_check_pass_map[3] = pos_check_pass;
	innov_check_pass_map[5] = (_vel_pos_test_ratio[5] <= 1.0f);

	// record the successful velocity fusion time
	if (vel_check_pass && _fuse_hor_vel) {
		_time_last_vel_fuse = _time_last_imu;
		_tilt_err_vec.setZero();
	}

	// record the successful position fusion time
	if (pos_check_pass && _fuse_pos) {
		_time_last_pos_fuse = _time_last_imu;
		_tilt_err_vec.setZero();
	}

	// record the successful height fusion time
	if (innov_check_pass_map[5] && _fuse_height) {
		_time_last_hgt_fuse = _time_last_imu;
	}

	for (unsigned obs_index = 0; obs_index < 6; obs_index++) {
		// skip fusion if not requested or checks have failed
		if (!fuse_map[obs_index] || !innov_check_pass_map[obs_index]) {
			continue;
		}

		unsigned state_index = obs_index + 3;	// we start with vx and this is the 4. state

		// calculate kalman gain K = PHS, where S = 1/innovation variance
		for (int row = 0; row <= 15; row++) {
			Kfusion[row] = P[row][state_index] / _vel_pos_innov_var[obs_index];
		}

		// only update magnetic field states if we are fusing 3-axis observations
		if (_control_status.flags.mag_3D) {
			for (int row = 16; row <= 21; row++) {
				Kfusion[row] = P[row][state_index] / _vel_pos_innov_var[obs_index];
			}

		} else {
			for (int row = 16; row <= 21; row++) {
				Kfusion[row] = 0.0f;
			}
		}

		// only update wind states if we are doing wind estimation
		if (_control_status.flags.wind) {
			for (int row = 22; row <= 23; row++) {
				Kfusion[row] = P[row][state_index] / _vel_pos_innov_var[obs_index];
			}

		} else {
			for (int row = 22; row <= 23; row++) {
				Kfusion[row] = 0.0f;
			}
		}

		// sum the attitude error from velocity and position fusion only
		// used as a metric for convergence monitoring
		if (obs_index != 5) {
			_tilt_err_vec += _state.ang_error;
		}

		// by definition the angle error state is zero at the fusion time
		_state.ang_error.setZero();

		// fuse the observation
		fuse(Kfusion, _vel_pos_innov[obs_index]);

		// correct the nominal quaternion
		Quaternion dq;
		dq.from_axis_angle(_state.ang_error);
		_state.quat_nominal = dq * _state.quat_nominal;
		_state.quat_nominal.normalize();

		// update covarinace matrix via Pnew = (I - KH)P
		float KHP[_k_num_states][_k_num_states] = {};

		for (unsigned row = 0; row < _k_num_states; row++) {
			for (unsigned column = 0; column < _k_num_states; column++) {
				KHP[row][column] = Kfusion[row] * P[state_index][column];
			}
		}

		for (unsigned row = 0; row < _k_num_states; row++) {
			for (unsigned column = 0; column < _k_num_states; column++) {
				P[row][column] = P[row][column] - KHP[row][column];
			}
		}

		makeSymmetrical();
		limitCov();
	}
}
