/****************************************************************************
 *
 *   Copyright (c) 2021 PX4. All rights reserved.
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
 * @file gps_fusion.cpp
 * Function for fusing gps measurements
 */

/* #include <mathlib/mathlib.h> */
#include "ekf.h"

void Ekf::fuseGpsVelPos()
{
	Vector3f gps_pos_obs_var;

	const float lower_limit = fmaxf(_params.gps_pos_noise, 0.01f);

	if (isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
		// if we are using other sources of aiding, then relax the upper observation
		// noise limit which prevents bad GPS perturbing the position estimate
		gps_pos_obs_var(0) = gps_pos_obs_var(1) = sq(fmaxf(_gps_sample_delayed.hacc, lower_limit));

	} else {
		// if we are not using another source of aiding, then we are reliant on the GPS
		// observations to constrain attitude errors and must limit the observation noise value.
		float upper_limit = fmaxf(_params.pos_noaid_noise, lower_limit);
		gps_pos_obs_var(0) = gps_pos_obs_var(1) = sq(math::constrain(_gps_sample_delayed.hacc, lower_limit, upper_limit));
	}



	const float vel_var = sq(_gps_sample_delayed.sacc);
	const Vector3f gps_vel_obs_var{vel_var, vel_var, vel_var * sq(1.5f)};

	// calculate innovations
	_gps_vel_innov = _state.vel - _gps_sample_delayed.vel;
	_gps_pos_innov.xy() = Vector2f(_state.pos) - _gps_sample_delayed.pos;

	// set innovation gate size
	const float pos_innov_gate = fmaxf(_params.gps_pos_innov_gate, 1.f);
	const float vel_innov_gate = fmaxf(_params.gps_vel_innov_gate, 1.f);

	// fuse GPS measurement
	fuseHorizontalVelocity(_gps_vel_innov, vel_innov_gate, gps_vel_obs_var, _gps_vel_innov_var, _gps_vel_test_ratio);
	fuseVerticalVelocity(_gps_vel_innov, vel_innov_gate, gps_vel_obs_var, _gps_vel_innov_var, _gps_vel_test_ratio);
	fuseHorizontalPosition(_gps_pos_innov, pos_innov_gate, gps_pos_obs_var, _gps_pos_innov_var, _gps_pos_test_ratio);
}
