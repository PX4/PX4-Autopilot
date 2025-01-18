/****************************************************************************
 *
 *   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
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

#include "ZeroVelocityUpdate.hpp"

#include "../ekf.h"

ZeroVelocityUpdate::ZeroVelocityUpdate()
{
	reset();
}

void ZeroVelocityUpdate::reset()
{
	_time_last_fuse = 0;
}

bool ZeroVelocityUpdate::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{
	// Fuse zero velocity at a limited rate (every 200 milliseconds)
	const bool zero_velocity_update_data_ready = (_time_last_fuse + 200'000 < imu_delayed.time_us);

	if (zero_velocity_update_data_ready) {
		const bool continuing_conditions_passing = ekf.control_status_flags().vehicle_at_rest
				&& ekf.control_status_prev_flags().vehicle_at_rest
				&& (!ekf.isVerticalVelocityAidingActive()
				    || !ekf.control_status_flags().tilt_align); // otherwise the filter is "too rigid" to follow a position drift

		if (continuing_conditions_passing) {
			Vector3f vel_obs{0.f, 0.f, 0.f};

			// Set a low variance initially for faster leveling and higher
			// later to let the states follow the measurements
			const float obs_var = ekf.control_status_flags().tilt_align ? sq(0.2f) : sq(0.001f);
			Vector3f innov_var = ekf.getVelocityVariance() + obs_var;

			for (unsigned i = 0; i < 3; i++) {
				const float innovation = ekf.state().vel(i) - vel_obs(i);
				ekf.fuseDirectStateMeasurement(innovation, innov_var(i), obs_var, State::vel.idx + i);
			}

			_time_last_fuse = imu_delayed.time_us;

			return true;
		}
	}

	return false;
}
