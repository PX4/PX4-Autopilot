/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @file fake_pos_control.cpp
 * Control functions for ekf fake position fusion
 */

#include "ekf.h"

void Ekf::controlFakePosFusion()
{
	// If we aren't doing any aiding, fake position measurements at the last known position to constrain drift
	// During intial tilt aligment, fake position is used to perform a "quasi-stationary" leveling of the EKF
	const bool fake_pos_data_ready = isTimedOut(_time_last_fake_pos_fuse, (uint64_t)2e5); // Fuse fake position at a limited rate

	if (fake_pos_data_ready) {
		const bool continuing_conditions_passing = !isHorizontalAidingActive();
		const bool starting_conditions_passing = continuing_conditions_passing;

		if (_using_synthetic_position) {
			if (continuing_conditions_passing) {
				fuseFakePosition();

				const bool is_fusion_failing = isTimedOut(_time_last_fake_pos_fuse, (uint64_t)4e5);

				if (is_fusion_failing) {
					resetFakePosFusion();
				}

			} else {
				stopFakePosFusion();
			}

		} else {
			if (starting_conditions_passing) {
				startFakePosFusion();

				if (_control_status.flags.tilt_align) {
					// The fake position fusion is not started for initial alignement
					_warning_events.flags.stopping_navigation = true;
					ECL_WARN("stopping navigation");
				}
			}
		}
	}
}

void Ekf::startFakePosFusion()
{
	if (!_using_synthetic_position) {
		_using_synthetic_position = true;
		_fuse_hpos_as_odom = false; // TODO: needed?
		resetFakePosFusion();
	}
}

void Ekf::resetFakePosFusion()
{
	_last_known_posNE = _state.pos.xy();
	resetHorizontalPositionToLastKnown();
	resetHorizontalVelocityToZero();
	_time_last_fake_pos_fuse = _time_last_imu;
}

void Ekf::stopFakePosFusion()
{
	_using_synthetic_position = false;
}

void Ekf::fuseFakePosition()
{
	Vector3f fake_pos_obs_var;

	if (_control_status.flags.in_air && _control_status.flags.tilt_align) {
		fake_pos_obs_var(0) = fake_pos_obs_var(1) = sq(fmaxf(_params.pos_noaid_noise, _params.gps_pos_noise));

	} else if (!_control_status.flags.in_air && _control_status.flags.vehicle_at_rest) {
		// Accelerate tilt fine alignment by fusing more
		// aggressively when the vehicle is at rest
		fake_pos_obs_var(0) = fake_pos_obs_var(1) = sq(0.01f);

	} else {
		fake_pos_obs_var(0) = fake_pos_obs_var(1) = sq(0.5f);
	}

	_gps_pos_innov.xy() = Vector2f(_state.pos) - _last_known_posNE;

	const float fake_pos_innov_gate = 3.f;

	if (fuseHorizontalPosition(_gps_pos_innov, fake_pos_innov_gate, fake_pos_obs_var,
	                           _gps_pos_innov_var, _gps_pos_test_ratio, true)) {
		_time_last_fake_pos_fuse = _time_last_imu;
	}
}
