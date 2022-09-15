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
 * @file fake_height_control.cpp
 * Control functions for ekf fake height fusion
 */

#include "ekf.h"

void Ekf::controlFakeHgtFusion()
{
	auto &fake_hgt = _aid_src_fake_hgt;

	// clear
	resetEstimatorAidStatusFlags(fake_hgt);

	// If we aren't doing any aiding, fake position measurements at the last known vertical position to constrain drift
	const bool fake_hgt_data_ready = isTimedOut(fake_hgt.time_last_fuse, (uint64_t)2e5); // Fuse fake height at a limited rate

	if (fake_hgt_data_ready) {
		const bool continuing_conditions_passing = !isVerticalAidingActive();
		const bool starting_conditions_passing = continuing_conditions_passing
							 && _vertical_velocity_deadreckon_time_exceeded
							 && _vertical_position_deadreckon_time_exceeded;

		if (_control_status.flags.fake_hgt) {
			if (continuing_conditions_passing) {
				fuseFakeHgt();

				const bool is_fusion_failing = isTimedOut(fake_hgt.time_last_fuse, (uint64_t)4e5);

				if (is_fusion_failing) {
					resetFakeHgtFusion();
				}

			} else {
				stopFakeHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				startFakeHgtFusion();
			}
		}
	}
}

void Ekf::startFakeHgtFusion()
{
	if (!_control_status.flags.fake_hgt) {
		ECL_INFO("start fake height fusion");
		_control_status.flags.fake_hgt = true;
		resetFakeHgtFusion();
	}
}

void Ekf::resetFakeHgtFusion()
{
	ECL_INFO("reset fake height fusion");
	_last_known_pos(2) = _state.pos(2);

	resetVerticalVelocityToZero();
	resetHeightToLastKnown();

	_aid_src_fake_hgt.time_last_fuse = _imu_sample_delayed.time_us;
}

void Ekf::resetHeightToLastKnown()
{
	_information_events.flags.reset_pos_to_last_known = true;
	ECL_INFO("reset height to last known");
	resetVerticalPositionTo(_last_known_pos(2));
	P.uncorrelateCovarianceSetVariance<1>(9, sq(_params.pos_noaid_noise));
}

void Ekf::stopFakeHgtFusion()
{
	if (_control_status.flags.fake_hgt) {
		ECL_INFO("stop fake height fusion");
		_control_status.flags.fake_hgt = false;

		resetEstimatorAidStatus(_aid_src_fake_hgt);
	}
}

void Ekf::fuseFakeHgt()
{
	const float obs_var = sq(_params.pos_noaid_noise);

	const float innov_gate = 3.f;

	auto &fake_hgt = _aid_src_fake_hgt;

	fake_hgt.observation = _last_known_pos(2);
	fake_hgt.observation_variance = obs_var;

	fake_hgt.innovation = _state.pos(2) - _last_known_pos(2);
	fake_hgt.innovation_variance = P(9, 9) + obs_var;

	setEstimatorAidStatusTestRatio(fake_hgt, innov_gate);

	// always protect against extreme values that could result in a NaN
	fake_hgt.fusion_enabled = fake_hgt.test_ratio < sq(100.0f / innov_gate);

	// fuse
	if (fake_hgt.fusion_enabled && !fake_hgt.innovation_rejected) {
		if (fuseVelPosHeight(fake_hgt.innovation, fake_hgt.innovation_variance, 5)) {
			fake_hgt.fused = true;
			fake_hgt.time_last_fuse = _imu_sample_delayed.time_us;
		}
	}

	fake_hgt.timestamp_sample = _imu_sample_delayed.time_us;
}
