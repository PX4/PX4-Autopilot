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

void Ekf::updateVelocityAidSrcStatus(const uint64_t &time_us, const Vector2f &obs, const Vector2f &obs_var,
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

void Ekf::fuseVelocity(estimator_aid_source2d_s &aid_src)
{
	if (!aid_src.innovation_rejected) {
		// vx, vy
		if (fuseDirectStateMeasurement(aid_src.innovation[0], aid_src.innovation_variance[0], State::vel.idx)
		    && fuseDirectStateMeasurement(aid_src.innovation[1], aid_src.innovation_variance[1], State::vel.idx + 1)
		   ) {
			aid_src.fused = true;
			aid_src.time_last_fuse = _time_delayed_us;

			_time_last_hor_vel_fuse = _time_delayed_us;

		} else {
			aid_src.fused = false;
		}
	}
}

void Ekf::fuseVelocity(estimator_aid_source3d_s &aid_src)
{
	if (!aid_src.innovation_rejected) {
		// vx, vy, vz
		if (fuseDirectStateMeasurement(aid_src.innovation[0], aid_src.innovation_variance[0], State::vel.idx + 0)
		    && fuseDirectStateMeasurement(aid_src.innovation[1], aid_src.innovation_variance[1], State::vel.idx + 1)
		    && fuseDirectStateMeasurement(aid_src.innovation[2], aid_src.innovation_variance[2], State::vel.idx + 2)
		   ) {
			aid_src.fused = true;
			aid_src.time_last_fuse = _time_delayed_us;

			_time_last_hor_vel_fuse = _time_delayed_us;
			_time_last_ver_vel_fuse = _time_delayed_us;

		} else {
			aid_src.fused = false;
		}
	}
}
