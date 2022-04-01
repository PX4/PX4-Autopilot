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
	// When at rest or no source of yaw aiding is active yaw fusion is run selectively to enable yaw gyro
	// bias learning when stationary on ground and to prevent uncontrolled yaw variance growth
	if (_control_status.flags.vehicle_at_rest && _control_status.flags.tilt_align) {

		const float euler_yaw = wrap_pi(getEulerYaw(_R_to_earth));

		// record static yaw when transitioning to at rest
		if (!PX4_ISFINITE(_last_static_yaw)) {
			_last_static_yaw = euler_yaw;
		}

		// fuse last static yaw at a limited rate (every 200 milliseconds)
		if (isTimedOut(_time_last_heading_fuse, (uint64_t)200'000)) {
			float innovation = euler_yaw - _last_static_yaw;
			float obs_var = 0.001f;
			updateQuaternion(innovation, obs_var);
		}

	} else {
		// vehicle moving or tilt alignment not yet completed

		const float sumQuatVar = P(0, 0) + P(1, 1) + P(2, 2) + P(3, 3);

		if (sumQuatVar > _params.quat_max_variance) {
			// if necessary fuse zero innovation to prevent unconstrained quaternion variance growth
			float innovation = 0.f;
			float obs_var = 0.25f;
			updateQuaternion(innovation, obs_var);

		} else if (!_control_status.flags.tilt_align) {
			// fuse zero heading innovation during the leveling fine alignment step to keep the yaw variance low
			float innovation = 0.f;
			float obs_var = 0.01f;
			updateQuaternion(innovation, obs_var);
		}

		_last_static_yaw = wrap_pi(getEulerYaw(_R_to_earth));
	}
}
