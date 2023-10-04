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

/**
 * @file zero_gyro_update.cpp
 * Control function for ekf zero gyro update
 */

#include "ekf.h"

void Ekf::controlZeroGyroUpdate(const imuSample &imu_delayed)
{
	if (!(_params.imu_ctrl & static_cast<int32_t>(ImuCtrl::GyroBias))) {
		return;
	}

	// When at rest, fuse the gyro data as a direct observation of the gyro bias
	if (_control_status.flags.vehicle_at_rest) {
		// Downsample gyro data to run the fusion at a lower rate
		_zgup_delta_ang += imu_delayed.delta_ang;
		_zgup_delta_ang_dt += imu_delayed.delta_ang_dt;

		static constexpr float zgup_dt = 0.2f;
		const bool zero_gyro_update_data_ready = _zgup_delta_ang_dt >= zgup_dt;

		if (zero_gyro_update_data_ready) {
			Vector3f gyro_bias = _zgup_delta_ang / _zgup_delta_ang_dt;
			Vector3f innovation = _state.gyro_bias - gyro_bias;

			const float obs_var = sq(math::constrain(_params.gyro_noise, 0.f, 1.f));

			const Vector3f innov_var = getGyroBiasVariance() + obs_var;

			for (int i = 0; i < 3; i++) {
				fuseDeltaAngBias(innovation(i), innov_var(i), i);
			}

			// Reset the integrators
			_zgup_delta_ang.setZero();
			_zgup_delta_ang_dt = 0.f;
		}

	} else if (_control_status_prev.flags.vehicle_at_rest) {
		// Reset the integrators
		_zgup_delta_ang.setZero();
		_zgup_delta_ang_dt = 0.f;
	}
}

void Ekf::fuseDeltaAngBias(const float innov, const float innov_var, const int obs_index)
{
	VectorState K;  // Kalman gain vector for any single observation - sequential fusion is used.
	const unsigned state_index = obs_index + State::gyro_bias.idx;

	// calculate kalman gain K = PHS, where S = 1/innovation variance
	for (int row = 0; row < State::size; row++) {
		K(row) = P(row, state_index) / innov_var;
	}

	measurementUpdate(K, innov_var, innov);
}
