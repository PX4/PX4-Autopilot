/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file terrain_control.cpp
 */

#include "ekf.h"
#include "ekf_derivation/generated/compute_hagl_innov_var.h"

#include <mathlib/mathlib.h>

void Ekf::initTerrain()
{
	// assume a ground clearance
	_state.terrain = -_gpos.altitude() + _params.rng_gnd_clearance;

	// use the ground clearance value as our uncertainty
	P.uncorrelateCovarianceSetVariance<State::terrain.dof>(State::terrain.idx, sq(_params.rng_gnd_clearance));
}

void Ekf::controlTerrainFakeFusion()
{
	// If we are on ground, store the local position and time to use as a reference
	if (!_control_status.flags.in_air) {
		_last_on_ground_posD = -_gpos.altitude();
		_control_status.flags.rng_fault = false;

	} else if (!_control_status_prev.flags.in_air) {
		// Let the estimator run freely before arming for bench testing purposes, but reset on takeoff
		// because when using optical flow measurements, it is safer to start with a small distance to ground
		// as an overestimated distance leads to an overestimated velocity, causing a dangerous behavior.
		initTerrain();
	}

	if (!_control_status.flags.in_air
	    && !_control_status.flags.rng_terrain
	    && !_control_status.flags.opt_flow_terrain) {

		bool recent_terrain_aiding = isRecent(_time_last_terrain_fuse, (uint64_t)1e6);

		if (_control_status.flags.vehicle_at_rest || !recent_terrain_aiding) {
			initTerrain();
		}
	}
}

void Ekf::updateTerrainValidity()
{
	bool valid_opt_flow_terrain = false;
	bool valid_rng_terrain = false;
	bool positive_hagl_var = false;
	bool small_relative_hagl_var = false;

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	if (_control_status.flags.opt_flow_terrain
	    && isRecent(_aid_src_optical_flow.time_last_fuse, _params.hgt_fusion_timeout_max)
	   ) {
		valid_opt_flow_terrain = true;
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

#if defined(CONFIG_EKF2_RANGE_FINDER)

	if (_control_status.flags.rng_terrain
	    && isRecent(_aid_src_rng_hgt.time_last_fuse, _params.hgt_fusion_timeout_max)
	   ) {
		valid_rng_terrain = true;
	}

#endif // CONFIG_EKF2_RANGE_FINDER

	if (_time_last_terrain_fuse != 0) {
		// Assume being valid when the uncertainty is small compared to the height above ground
		float hagl_var = INFINITY;
		sym::ComputeHaglInnovVar(P, 0.f, &hagl_var);

		positive_hagl_var = hagl_var > 0.f;

		if (positive_hagl_var
		    && (hagl_var < sq(fmaxf(0.1f * getHagl(), 0.5f)))
		   ) {
			small_relative_hagl_var = true;
		}
	}

	const bool positive_hagl = getHagl() >= 0.f;

	if (!_terrain_valid) {
		// require valid RNG or optical flow (+valid variance) to initially consider terrain valid
		if (positive_hagl
		    && positive_hagl_var
		    && (valid_rng_terrain
			|| (valid_opt_flow_terrain && small_relative_hagl_var))
		   ) {
			_terrain_valid = true;
		}

	} else {
		// terrain was previously valid, continue considering valid if variance is good
		_terrain_valid = positive_hagl && positive_hagl_var && small_relative_hagl_var;
	}
}
