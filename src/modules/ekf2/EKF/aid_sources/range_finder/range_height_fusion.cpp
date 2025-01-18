/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
#include "ekf_derivation/generated/compute_hagl_h.h"

bool Ekf::fuseHaglRng(estimator_aid_source1d_s &aid_src, bool update_height, bool update_terrain)
{
	if (aid_src.innovation_rejected) {
		_innov_check_fail_status.flags.reject_hagl = true;
		return false;
	}

	VectorState H;

	sym::ComputeHaglH(&H);

	// calculate the Kalman gain
	VectorState K = P * H / aid_src.innovation_variance;

	if (!update_terrain) {
		K(State::terrain.idx) = 0.f;
	}

	if (!update_height) {
		const float k_terrain = K(State::terrain.idx);
		K.zero();
		K(State::terrain.idx) = k_terrain;
	}

	measurementUpdate(K, H, aid_src.observation_variance, aid_src.innovation);

	// record last successful fusion event
	_innov_check_fail_status.flags.reject_hagl = false;

	aid_src.time_last_fuse = _time_delayed_us;
	aid_src.fused = true;

	if (update_terrain) {
		_time_last_terrain_fuse = _time_delayed_us;
	}

	return true;
}
