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
 * @file zero_innovation_heading_update.cpp
 * Control function for ekf heading update when at rest or no other heading source available
 */

#include "ekf.h"

void Ekf::controlZeroInnovationHeadingUpdate()
{
	const bool yaw_aiding = _control_status.flags.mag_hdg || _control_status.flags.mag_3D
				|| _control_status.flags.ev_yaw || _control_status.flags.gps_yaw;

	if (!_control_status.flags.tilt_align) {
		// fuse zero heading innovation during the leveling fine alignment step to keep the yaw variance low
		float innovation = 0.f;
		float obs_var = _control_status.flags.vehicle_at_rest ? 0.001f : 0.1f;
		estimator_aid_source1d_s unused;
		fuseYaw(innovation, obs_var, unused);
		_time_last_heading_fuse = 0;
		_last_static_yaw = NAN;

	} else if (_control_status.flags.vehicle_at_rest) {
		// When at rest or no source of yaw aiding is active yaw fusion is run selectively to enable yaw gyro
		// bias learning when stationary on ground and to prevent uncontrolled yaw variance growth
		const float euler_yaw = getEulerYaw(_R_to_earth);

		if (PX4_ISFINITE(_last_static_yaw)) {
			// fuse last static yaw at a limited rate (every 200 milliseconds)
			if (!yaw_aiding && isTimedOut(_time_last_heading_fuse, (uint64_t)200'000)) {
				float innovation = wrap_pi(euler_yaw - _last_static_yaw);
				float obs_var = 0.01f;
				estimator_aid_source1d_s unused;
				fuseYaw(innovation, obs_var, unused);
			}

		} else {
			// record static yaw when transitioning to at rest
			_last_static_yaw = euler_yaw;
		}

	} else {
		// vehicle moving and tilt alignment completed

		// fuse zero innovation at a limited rate if the yaw variance is too large
		if (!yaw_aiding && isTimedOut(_time_last_heading_fuse, (uint64_t)200'000)) {
			float obs_var = 0.25f;
			estimator_aid_source1d_s aid_src_status;
			Vector24f H_YAW;

			computeYawInnovVarAndH(obs_var, aid_src_status.innovation_variance, H_YAW);

			if ((aid_src_status.innovation_variance - obs_var) > sq(_params.mag_heading_noise)) {
				// The yaw variance is too large, fuse fake measurement
				float innovation = 0.f;
				fuseYaw(innovation, obs_var, aid_src_status, H_YAW);
			}
		}

		_last_static_yaw = NAN;
	}
}
