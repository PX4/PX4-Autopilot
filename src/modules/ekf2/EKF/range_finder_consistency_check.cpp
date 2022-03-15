/****************************************************************************
 *
 *   Copyright (c) 2022 PX4. All rights reserved.
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
 * @file range_finder_consistency_check.cpp
 */

#include "range_finder_consistency_check.hpp"

void RangeFinderConsistencyCheck::update(float dist_bottom, float dist_bottom_var, float vz, float vz_var, float time_s)
{
	const float dt = time_s - _time_last_update_s;

	if ((_time_last_update_s < FLT_EPSILON)
	    || (dt < 0.001f) || (dt > 0.5f)) {
		_time_last_update_s = time_s;
		_dist_bottom_prev = dist_bottom;
		return;
	}

	const float vel_bottom = (dist_bottom - _dist_bottom_prev) / dt;
	const float innov = -vel_bottom - vz; // vel_bottom is +up while vz is +down
	const float vel_bottom_var = 2.f * dist_bottom_var / (dt * dt);
	const float innov_var = vel_bottom_var + vz_var;
	const float normalized_innov_sq = (innov * innov) / innov_var;
	_vel_bottom_test_ratio = normalized_innov_sq / (_vel_bottom_gate * _vel_bottom_gate);

	_vel_bottom_signed_test_ratio_lpf.setParameters(dt, _vel_bottom_signed_test_ratio_tau);
	const float signed_test_ratio = matrix::sign(innov) * normalized_innov_sq / (_vel_bottom_signed_gate * _vel_bottom_signed_gate);
	_vel_bottom_signed_test_ratio_lpf.update(signed_test_ratio);

	_time_last_update_s = time_s;
	_dist_bottom_prev = dist_bottom;
}
