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
	auto &aid_src = _aid_src_fake_hgt;

	// If we aren't doing any aiding, fake position measurements at the last known vertical position to constrain drift
	const bool fake_hgt_data_ready = !isVerticalAidingActive()
					 && isTimedOut(aid_src.time_last_fuse, (uint64_t)2e5); // Fuse fake height at a limited rate

	if (fake_hgt_data_ready) {

		const float obs_var = sq(_params.pos_noaid_noise);
		const float innov_gate = 3.f;

		updateVerticalPositionAidStatus(aid_src, _time_delayed_us, -_last_known_gpos.altitude(), obs_var, innov_gate);

		const bool continuing_conditions_passing = !isVerticalAidingActive();
		const bool starting_conditions_passing = continuing_conditions_passing
				&& _vertical_velocity_deadreckon_time_exceeded
				&& _vertical_position_deadreckon_time_exceeded;

		if (_control_status.flags.fake_hgt) {
			if (continuing_conditions_passing) {

				// always protect against extreme values that could result in a NaN
				if (aid_src.test_ratio < sq(100.0f / innov_gate)) {
					if (!aid_src.innovation_rejected) {
						fuseDirectStateMeasurement(aid_src.innovation, aid_src.innovation_variance, aid_src.observation_variance,
									   State::pos.idx + 2);

						aid_src.fused = true;
						aid_src.time_last_fuse = _time_delayed_us;
					}
				}

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, (uint64_t)4e5);

				if (is_fusion_failing) {
					resetFakeHgtFusion();
				}

			} else {
				stopFakeHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				ECL_INFO("start fake height fusion");
				_control_status.flags.fake_hgt = true;
				resetFakeHgtFusion();
			}
		}

	} else if (_control_status.flags.fake_hgt && isVerticalAidingActive()) {
		stopFakeHgtFusion();
	}
}

void Ekf::resetFakeHgtFusion()
{
	ECL_INFO("reset fake height fusion");
	_last_known_gpos.setAltitude(_gpos.altitude());

	resetVerticalVelocityToZero();
	resetHeightToLastKnown();

	_aid_src_fake_hgt.time_last_fuse = _time_delayed_us;
}

void Ekf::resetHeightToLastKnown()
{
	_information_events.flags.reset_pos_to_last_known = true;
	ECL_INFO("reset height to last known (%.3f)", (double)_last_known_gpos.altitude());
	resetHeightTo(_last_known_gpos.altitude(), sq(_params.pos_noaid_noise));
}

void Ekf::stopFakeHgtFusion()
{
	if (_control_status.flags.fake_hgt) {
		ECL_INFO("stop fake height fusion");
		_control_status.flags.fake_hgt = false;
	}
}
