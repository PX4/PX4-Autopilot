/****************************************************************************
 *
 *   Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
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

#include "ekf.h"

void Ekf::updateHorizontalVelocityAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var,
		const float innov_gate, estimator_aid_source2d_s &aid_src) const
{
	resetEstimatorAidStatus(aid_src);

	for (int i = 0; i < 2; i++) {
		aid_src.observation[i] = obs(i);
		aid_src.innovation[i] = _state.vel(i) - aid_src.observation[i];

		aid_src.observation_variance[i] = math::max(sq(0.01f), obs_var(i));
		const int state_index = State::vel.idx + i;
		aid_src.innovation_variance[i] = P(state_index, state_index) + aid_src.observation_variance[i];
	}

	setEstimatorAidStatusTestRatio(aid_src, innov_gate);

	aid_src.timestamp_sample = time_us;
}

void Ekf::updateVelocityAidSrcStatus(const uint64_t &time_us, const Vector3f &obs, const Vector3f &obs_var,
				     const float innov_gate, estimator_aid_source3d_s &aid_src) const
{
	resetEstimatorAidStatus(aid_src);

	for (int i = 0; i < 3; i++) {
		aid_src.observation[i] = obs(i);
		aid_src.innovation[i] = _state.vel(i) - aid_src.observation[i];

		aid_src.observation_variance[i] = math::max(sq(0.01f), obs_var(i));
		const int state_index = State::vel.idx + i;
		aid_src.innovation_variance[i] = P(state_index, state_index) + aid_src.observation_variance[i];
	}

	setEstimatorAidStatusTestRatio(aid_src, innov_gate);

	// vz special case if there is bad vertical acceleration data, then don't reject measurement,
	// but limit innovation to prevent spikes that could destabilise the filter
	if (_fault_status.flags.bad_acc_vertical && aid_src.innovation_rejected) {
		const float innov_limit = innov_gate * sqrtf(aid_src.innovation_variance[2]);
		aid_src.innovation[2] = math::constrain(aid_src.innovation[2], -innov_limit, innov_limit);
		aid_src.innovation_rejected = false;
	}

	aid_src.timestamp_sample = time_us;
}

void Ekf::fuseHorizontalVelocity(estimator_aid_source2d_s &aid_src)
{
	// vx, vy
	if (!aid_src.innovation_rejected
	    && fuseDirectStateMeasurement(aid_src.innovation[0], aid_src.innovation_variance[0], aid_src.observation_variance[0], State::vel.idx + 0)
	    && fuseDirectStateMeasurement(aid_src.innovation[1], aid_src.innovation_variance[1], aid_src.observation_variance[1], State::vel.idx + 1)
	   ) {
		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hor_vel_fuse = _time_delayed_us;

	} else {
		aid_src.fused = false;
	}
}

void Ekf::fuseVelocity(estimator_aid_source3d_s &aid_src)
{
	// vx, vy, vz
	if (!aid_src.innovation_rejected
	    && fuseDirectStateMeasurement(aid_src.innovation[0], aid_src.innovation_variance[0], aid_src.observation_variance[0], State::vel.idx + 0)
	    && fuseDirectStateMeasurement(aid_src.innovation[1], aid_src.innovation_variance[1], aid_src.observation_variance[1], State::vel.idx + 1)
	    && fuseDirectStateMeasurement(aid_src.innovation[2], aid_src.innovation_variance[2], aid_src.observation_variance[2], State::vel.idx + 2)
	   ) {
		aid_src.fused = true;
		aid_src.time_last_fuse = _time_delayed_us;

		_time_last_hor_vel_fuse = _time_delayed_us;
		_time_last_ver_vel_fuse = _time_delayed_us;

	} else {
		aid_src.fused = false;
	}
}

void Ekf::resetHorizontalVelocityTo(const Vector2f &new_horz_vel, const Vector2f &new_horz_vel_var)
{
	const Vector2f delta_horz_vel = new_horz_vel - Vector2f(_state.vel);
	_state.vel.xy() = new_horz_vel;

	if (PX4_ISFINITE(new_horz_vel_var(0))) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx, math::max(sq(0.01f), new_horz_vel_var(0)));
	}

	if (PX4_ISFINITE(new_horz_vel_var(1))) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 1, math::max(sq(0.01f), new_horz_vel_var(1)));
	}

	_output_predictor.resetHorizontalVelocityTo(delta_horz_vel);

	// record the state change
	if (_state_reset_status.reset_count.velNE == _state_reset_count_prev.velNE) {
		_state_reset_status.velNE_change = delta_horz_vel;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.velNE_change += delta_horz_vel;
	}

	_state_reset_status.reset_count.velNE++;

	// Reset the timout timer
	_time_last_hor_vel_fuse = _time_delayed_us;
}

void Ekf::resetVerticalVelocityTo(float new_vert_vel, float new_vert_vel_var)
{
	const float delta_vert_vel = new_vert_vel - _state.vel(2);
	_state.vel(2) = new_vert_vel;

	if (PX4_ISFINITE(new_vert_vel_var)) {
		P.uncorrelateCovarianceSetVariance<1>(State::vel.idx + 2, math::max(sq(0.01f), new_vert_vel_var));
	}

	_output_predictor.resetVerticalVelocityTo(delta_vert_vel);

	// record the state change
	if (_state_reset_status.reset_count.velD == _state_reset_count_prev.velD) {
		_state_reset_status.velD_change = delta_vert_vel;

	} else {
		// there's already a reset this update, accumulate total delta
		_state_reset_status.velD_change += delta_vert_vel;
	}

	_state_reset_status.reset_count.velD++;

	// Reset the timout timer
	_time_last_ver_vel_fuse = _time_delayed_us;
}

void Ekf::resetHorizontalVelocityToZero()
{
	ECL_INFO("reset velocity to zero");
	_information_events.flags.reset_vel_to_zero = true;

	// Used when falling back to non-aiding mode of operation
	resetHorizontalVelocityTo(Vector2f{0.f, 0.f}, 25.f);
}

void Ekf::resetVerticalVelocityToZero()
{
	// we don't know what the vertical velocity is, so set it to zero
	// Set the variance to a value large enough to allow the state to converge quickly
	// that does not destabilise the filter
	resetVerticalVelocityTo(0.0f, 10.f);
}

void Ekf::resetVelocityTo(const Vector3f &new_vel, const Vector3f &new_vel_var)
{
	resetHorizontalVelocityTo(Vector2f(new_vel), Vector2f(new_vel_var(0), new_vel_var(1)));
	resetVerticalVelocityTo(new_vel(2), new_vel_var(2));
}
