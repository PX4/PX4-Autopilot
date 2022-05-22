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
	auto &fake_pos = _aid_src_fake_pos;

	// clear
	resetEstimatorAidStatusFlags(fake_pos);

	// If we aren't doing any aiding, fake position measurements at the last known position to constrain drift
	// During intial tilt aligment, fake position is used to perform a "quasi-stationary" leveling of the EKF
	const bool fake_pos_data_ready = isTimedOut(fake_pos.time_last_fuse[0], (uint64_t)2e5); // Fuse fake position at a limited rate

	if (fake_pos_data_ready) {
		const bool continuing_conditions_passing = !isHorizontalAidingActive();
		const bool starting_conditions_passing = continuing_conditions_passing;

		if (_using_synthetic_position) {
			if (continuing_conditions_passing) {
				fuseFakePosition();

				const bool is_fusion_failing = isTimedOut(fake_pos.time_last_fuse[0], (uint64_t)4e5);

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
		ECL_INFO("start fake position fusion");
		_using_synthetic_position = true;
		_fuse_hpos_as_odom = false; // TODO: needed?
		resetFakePosFusion();
	}
}

void Ekf::resetFakePosFusion()
{
	ECL_INFO("reset fake position fusion");
	_last_known_posNE = _state.pos.xy();

	resetHorizontalPositionToLastKnown();
	resetHorizontalVelocityToZero();

	_aid_src_fake_pos.time_last_fuse[0] = _time_last_imu;
	_aid_src_fake_pos.time_last_fuse[1] = _time_last_imu;
}

void Ekf::stopFakePosFusion()
{
	if (_using_synthetic_position) {
		ECL_INFO("stop fake position fusion");
		_using_synthetic_position = false;

		resetEstimatorAidStatus(_aid_src_fake_pos);
	}
}

void Ekf::fuseFakePosition()
{
	Vector2f obs_var;

	if (_control_status.flags.in_air && _control_status.flags.tilt_align) {
		obs_var(0) = obs_var(1) = sq(fmaxf(_params.pos_noaid_noise, _params.gps_pos_noise));

	} else if (!_control_status.flags.in_air && _control_status.flags.vehicle_at_rest) {
		// Accelerate tilt fine alignment by fusing more
		// aggressively when the vehicle is at rest
		obs_var(0) = obs_var(1) = sq(0.01f);

	} else {
		obs_var(0) = obs_var(1) = sq(0.5f);
	}

	const float innov_gate = 3.f;

	auto &fake_pos = _aid_src_fake_pos;

	for (int i = 0; i < 2; i++) {
		fake_pos.observation[i] = _last_known_posNE(i);
		fake_pos.observation_variance[i] = obs_var(i);

		fake_pos.innovation[i] = _state.pos(i) - _last_known_posNE(i);
		fake_pos.innovation_variance[i] = P(7 + i, 7 + i) + obs_var(i);
	}

	setEstimatorAidStatusTestRatio(fake_pos, innov_gate);

	// fuse
	for (int i = 0; i < 2; i++) {
		// always protect against extreme values that could result in a NaN
		fake_pos.fusion_enabled[i] = fake_pos.test_ratio[i] < sq(100.0f / innov_gate);

		if (fake_pos.fusion_enabled[i] && !fake_pos.innovation_rejected[i]) {
			if (fuseVelPosHeight(fake_pos.innovation[i], fake_pos.innovation_variance[i], 3 + i)) {
				fake_pos.fused[i] = true;
				fake_pos.time_last_fuse[i] = _time_last_imu;
			}
		}
	}

	fake_pos.timestamp_sample = _time_last_imu;
}
