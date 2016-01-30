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
 *
 */

#include "ekf.h"

void Ekf::fuseVelPosHeight()
{
	bool fuse_map[6] = {};
    bool innov_check_pass_map[6] = {};
    float R[6] = {};
    float gate_size[6] = {};
	float Kfusion[24] = {};

    // calculate innovations and gate sizes
	if (_fuse_hor_vel) {
		fuse_map[0] = fuse_map[1] = true;
		_vel_pos_innov[0] = _state.vel(0) - _gps_sample_delayed.vel(0);
		_vel_pos_innov[1] = _state.vel(1) - _gps_sample_delayed.vel(1);
		R[0] = _params.gps_vel_noise;
		R[1] = _params.gps_vel_noise;
        gate_size[0] = fmaxf(_params.vel_innov_gate, 1.0f);
        gate_size[1] = gate_size[0];
    }

	if (_fuse_vert_vel) {
		fuse_map[2] = true;
		_vel_pos_innov[2] = _state.vel(2) - _gps_sample_delayed.vel(2);
		R[2] = _params.gps_vel_noise;
        gate_size[2] = fmaxf(_params.vel_innov_gate, 1.0f);
    }

	if (_fuse_pos) {
		fuse_map[3] = fuse_map[4] = true;
		_vel_pos_innov[3] = _state.pos(0) - _gps_sample_delayed.pos(0);
		_vel_pos_innov[4] = _state.pos(1) - _gps_sample_delayed.pos(1);
		R[3] = _params.gps_pos_noise;
		R[4] = _params.gps_pos_noise;
        gate_size[3] = fmaxf(_params.posNE_innov_gate, 1.0f);
        gate_size[4] = gate_size[3];
	}

	if (_fuse_height) {
		fuse_map[5] = true;
		_vel_pos_innov[5] = _state.pos(2) - (-_baro_sample_delayed.hgt);		// baro measurement has inversed z axis
		R[5] = _params.baro_noise;
        gate_size[5] = fmaxf(_params.baro_innov_gate, 1.0f);
    }

    // calculate innovation test ratios
    for (unsigned obs_index = 0; obs_index < 6; obs_index++) {
        if (fuse_map[obs_index]) {
            // compute the innovation variance SK = HPH + R
            unsigned state_index = obs_index + 3;	// we start with vx and this is the 4. state
            _vel_pos_innov_var[obs_index] = P[state_index][state_index] + R[obs_index];
            // Compute the ratio of innovation to gate size
            _vel_pos_test_ratio[obs_index] = sq(_vel_pos_innov[obs_index]) / (sq(gate_size[obs_index]) * _vel_pos_innov[obs_index]);
        }
    }

    // check position, velocity and height innovations
    // treat 3D velocity, 2D position and height as separate sensors
    // always pass position checks if using synthetic position measurements
    bool vel_check_pass = (_vel_pos_test_ratio[0] <= 1.0f) && (_vel_pos_test_ratio[1] <= 1.0f) && (_vel_pos_test_ratio[2] <= 1.0f);
    innov_check_pass_map[2] = innov_check_pass_map[1] = innov_check_pass_map[0] = vel_check_pass;
    bool using_synthetic_measurements = !_control_status.flags.gps && !_control_status.flags.opt_flow;
    bool pos_check_pass = ((_vel_pos_test_ratio[3] <= 1.0f) && (_vel_pos_test_ratio[4] <= 1.0f)) || using_synthetic_measurements;
    innov_check_pass_map[4] = innov_check_pass_map[3] = pos_check_pass;
    innov_check_pass_map[5] = (_vel_pos_test_ratio[5] <= 1.0f);

    // record the successful velocity fusion time
    if (vel_check_pass && _fuse_hor_vel) {
        _time_last_vel_fuse = _time_last_imu;
    }

    // record the successful position fusion time
    if (pos_check_pass && _fuse_pos) {
        _time_last_pos_fuse = _time_last_imu;
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
		for (int row = 0; row < 24; row++) {
            Kfusion[row] = P[row][state_index] / _vel_pos_innov_var[obs_index];
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

void Ekf::fuse(float *K, float innovation)
{
	for (unsigned i = 0; i < 3; i++) {
		_state.ang_error(i) = _state.ang_error(i) - K[i] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.vel(i) = _state.vel(i) - K[i + 3] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.pos(i) = _state.pos(i) - K[i + 6] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.gyro_bias(i) = _state.gyro_bias(i) - K[i + 9] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.gyro_scale(i) = _state.gyro_scale(i) - K[i + 12] * innovation;
	}

	_state.accel_z_bias -= K[15] * innovation;

	for (unsigned i = 0; i < 3; i++) {
		_state.mag_I(i) = _state.mag_I(i) - K[i + 16] * innovation;
	}

	for (unsigned i = 0; i < 3; i++) {
		_state.mag_B(i) = _state.mag_B(i) - K[i + 19] * innovation;
	}

	for (unsigned i = 0; i < 2; i++) {
		_state.wind_vel(i) = _state.wind_vel(i) - K[i + 22] * innovation;
	}
}
