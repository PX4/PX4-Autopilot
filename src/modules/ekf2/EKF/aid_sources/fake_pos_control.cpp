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
	auto &aid_src = _aid_src_fake_pos;

	// If we aren't doing any aiding, fake position measurements at the last known position to constrain drift
	// During initial tilt alignment, fake position is used to perform a "quasi-stationary" leveling of the EKF
	const bool fake_pos_data_ready = !isHorizontalAidingActive()
					 && isTimedOut(aid_src.time_last_fuse, (uint64_t)2e5); // Fuse fake position at a limited rate

	if (fake_pos_data_ready) {

		Vector2f obs_var;

		if (_control_status.flags.in_air && _control_status.flags.tilt_align) {
			obs_var(0) = obs_var(1) = sq(fmaxf(_params.pos_noaid_noise, 1.f));

		} else if (!_control_status.flags.in_air && _control_status.flags.vehicle_at_rest) {
			// Accelerate tilt fine alignment by fusing more
			// aggressively when the vehicle is at rest
			obs_var(0) = obs_var(1) = sq(0.01f);

		} else {
			obs_var(0) = obs_var(1) = sq(0.5f);
		}

		const Vector2f innovation = (_gpos - _last_known_gpos).xy();

		const float innov_gate = 3.f;

		updateAidSourceStatus(aid_src,
				      _time_delayed_us,
				      Vector2f(_gpos.latitude_deg(), _gpos.longitude_deg()), // observation
				      obs_var,                                               // observation variance
				      innovation,                       // innovation
				      Vector2f(getStateVariance<State::pos>()) + obs_var,    // innovation variance
				      innov_gate);                                           // innovation gate

		const bool enable_valid_fake_pos = _control_status.flags.constant_pos || _control_status.flags.vehicle_at_rest;
		const bool enable_fake_pos = !enable_valid_fake_pos
					     && (getTiltVariance() > sq(math::radians(3.f)))
					     && !(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GravityVector))
					     && _horizontal_deadreckon_time_exceeded;

		_control_status.flags.fake_pos = runFakePosStateMachine(enable_fake_pos, _control_status.flags.fake_pos, aid_src);
		_control_status.flags.valid_fake_pos = runFakePosStateMachine(enable_valid_fake_pos,
						       _control_status.flags.valid_fake_pos, aid_src);

	} else if ((_control_status.flags.fake_pos || _control_status.flags.valid_fake_pos) && isHorizontalAidingActive()) {
		ECL_INFO("stop fake position fusion");
		_control_status.flags.fake_pos = false;
		_control_status.flags.valid_fake_pos = false;
	}
}

void Ekf::resetFakePosFusion()
{
	ECL_INFO("reset fake position fusion");
	_last_known_gpos.setLatLon(_gpos);

	resetHorizontalPositionToLastKnown();
	resetHorizontalVelocityToZero();

	_aid_src_fake_pos.time_last_fuse = _time_delayed_us;
}

bool Ekf::runFakePosStateMachine(const bool enable_conditions_passing, bool status_flag,
				 estimator_aid_source2d_s &aid_src)
{
	if (status_flag) {
		if (enable_conditions_passing) {
			if (!aid_src.innovation_rejected) {
				for (unsigned i = 0; i < 2; i++) {
					fuseDirectStateMeasurement(aid_src.innovation[i], aid_src.innovation_variance[i], aid_src.observation_variance[i],
								   State::pos.idx + i);
				}

				aid_src.fused = true;
				aid_src.time_last_fuse = _time_delayed_us;
			}

			const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, (uint64_t)4e5);

			if (is_fusion_failing) {
				ECL_WARN("fake position fusion failing, resetting");
				resetFakePosFusion();
			}

		} else {
			ECL_INFO("stop fake position fusion");
			status_flag = false;
		}

	} else {
		if (enable_conditions_passing) {
			ECL_INFO("start fake position fusion");
			status_flag = true;

			resetFakePosFusion();
		}
	}

	return status_flag;
}
