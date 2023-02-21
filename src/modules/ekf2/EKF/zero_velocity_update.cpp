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
 * @file zero_velocity_update.cpp
 * Control function for ekf zero velocity update
 */

#include "ekf.h"

void Ekf::controlZeroVelocityUpdate()
{
	// Fuse zero velocity at a limited rate (every 200 milliseconds)
	const bool zero_velocity_update_data_ready = isTimedOut(_time_last_zero_velocity_fuse, (uint64_t)2e5);

	if (zero_velocity_update_data_ready) {
		const bool continuing_conditions_passing = _control_status.flags.vehicle_at_rest
				&& _control_status_prev.flags.vehicle_at_rest
				&& !isVerticalVelocityAidingActive(); // otherwise the filter is "too rigid" to follow a position drift

		if (continuing_conditions_passing) {
			Vector3f vel_obs{0, 0, 0};
			Vector3f innovation = _state.vel - vel_obs;

			// Set a low variance initially for faster leveling and higher
			// later to let the states follow the measurements
			const float obs_var = _control_status.flags.tilt_align ? sq(0.2f) : sq(0.001f);
			Vector3f innov_var{
				P(4, 4) + obs_var,
				P(5, 5) + obs_var,
				P(6, 6) + obs_var};

			fuseVelPosHeight(innovation(0), innov_var(0), 0);
			fuseVelPosHeight(innovation(1), innov_var(1), 1);
			fuseVelPosHeight(innovation(2), innov_var(2), 2);

			_time_last_zero_velocity_fuse = _time_delayed_us;
		}
	}
}
