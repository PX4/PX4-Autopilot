/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "ZeroGyroUpdate.hpp"

#include "../ekf.h"

ZeroGyroUpdate::ZeroGyroUpdate()
{
	reset();
}

void ZeroGyroUpdate::reset()
{
	_zgup_delta_ang.setZero();
	_zgup_delta_ang_dt = 0.f;
}

bool ZeroGyroUpdate::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{
	// When at rest, fuse the gyro data as a direct observation of the gyro bias
	if (ekf.control_status_flags().vehicle_at_rest) {
		// Downsample gyro data to run the fusion at a lower rate
		_zgup_delta_ang += imu_delayed.delta_ang;
		_zgup_delta_ang_dt += imu_delayed.delta_ang_dt;

		static constexpr float zgup_dt = 0.2f;
		const bool zero_gyro_update_data_ready = _zgup_delta_ang_dt >= zgup_dt;

		if (zero_gyro_update_data_ready) {

			Vector3f gyro_bias = _zgup_delta_ang / _zgup_delta_ang_dt;

			const float obs_var = sq(math::constrain(ekf.getGyroNoise(), 0.f, 1.f));

			for (unsigned i = 0; i < 3; i++) {
				const float innovation = ekf.state().gyro_bias(i) - gyro_bias(i);
				const float innov_var = ekf.getGyroBiasVariance()(i) + obs_var;
				ekf.fuseDirectStateMeasurement(innovation, innov_var, obs_var, State::gyro_bias.idx + i);
			}

			// Reset the integrators
			_zgup_delta_ang.setZero();
			_zgup_delta_ang_dt = 0.f;

			return true;
		}

	} else if (ekf.control_status_prev_flags().vehicle_at_rest) {
		// Reset the integrators
		_zgup_delta_ang.setZero();
		_zgup_delta_ang_dt = 0.f;
	}

	return false;
}
