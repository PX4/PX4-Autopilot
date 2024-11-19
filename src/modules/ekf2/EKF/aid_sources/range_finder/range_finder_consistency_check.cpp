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

#include <aid_sources/range_finder/range_finder_consistency_check.hpp>

void RangeFinderConsistencyCheck::update(float dist_bottom, float dist_bottom_var, float vz, float vz_var,
		bool horizontal_motion, uint64_t time_us)
{
	if (horizontal_motion) {
		_time_last_horizontal_motion = time_us;
	}

	const float dt = static_cast<float>(time_us - _time_last_update_us) * 1e-6f;

	if ((_time_last_update_us == 0)
	    || (dt < 0.001f) || (dt > 0.5f)) {
		_time_last_update_us = time_us;
		_dist_bottom_prev = dist_bottom;
		return;
	}

	const float vel_bottom = (dist_bottom - _dist_bottom_prev) / dt;
	_innov = -vel_bottom - vz; // vel_bottom is +up while vz is +down

	// Variance of the time derivative of a random variable: var(dz/dt) = 2*var(z) / dt^2
	const float var = 2.f * dist_bottom_var / (dt * dt);
	_innov_var = var + vz_var;

	const float normalized_innov_sq = (_innov * _innov) / _innov_var;
	_test_ratio = normalized_innov_sq / (_gate * _gate);
	_signed_test_ratio_lpf.setParameters(dt, _signed_test_ratio_tau);
	const float signed_test_ratio = matrix::sign(_innov) * _test_ratio;
	_signed_test_ratio_lpf.update(signed_test_ratio);

	updateConsistency(vz, time_us);

	_time_last_update_us = time_us;
	_dist_bottom_prev = dist_bottom;
}

void RangeFinderConsistencyCheck::updateConsistency(float vz, uint64_t time_us)
{
	if (fabsf(_signed_test_ratio_lpf.getState()) >= 1.f) {
		if ((time_us - _time_last_horizontal_motion) > _signed_test_ratio_tau) {
			_is_kinematically_consistent = false;
			_time_last_inconsistent_us = time_us;
		}

	} else {
		if ((fabsf(vz) > _min_vz_for_valid_consistency)
		    && (_test_ratio < 1.f)
		    && ((time_us - _time_last_inconsistent_us) > _consistency_hyst_time_us)
		   ) {
			_is_kinematically_consistent = true;
		}
	}
}
