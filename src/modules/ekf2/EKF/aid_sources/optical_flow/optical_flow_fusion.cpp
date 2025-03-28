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
 * @file optical_flow_fusion.cpp
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <float.h>
#include <ekf_derivation/generated/compute_flow_xy_innov_var_and_hx.h>
#include <ekf_derivation/generated/compute_flow_y_innov_var_and_h.h>

bool Ekf::fuseOptFlow(VectorState &H, const bool update_terrain)
{
	const auto state_vector = _state.vector();

	// if either axis fails we abort the fusion
	if (_aid_src_optical_flow.innovation_rejected) {
		_innov_check_fail_status.flags.reject_optflow_X = true;
		_innov_check_fail_status.flags.reject_optflow_Y = true;
		return false;
	}

	// fuse observation axes sequentially
	for (uint8_t index = 0; index <= 1; index++) {
		if (index == 0) {
			// everything was already computed before

		} else if (index == 1) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			const float R_LOS = _aid_src_optical_flow.observation_variance[1];
			const float epsilon = 1e-3f;
			sym::ComputeFlowYInnovVarAndH(state_vector, P, R_LOS, epsilon, &_aid_src_optical_flow.innovation_variance[1], &H);

			// recalculate the innovation using the updated state
			const Vector3f flow_gyro_corrected = _flow_sample_delayed.gyro_rate - _flow_gyro_bias;
			_aid_src_optical_flow.innovation[1] = predictFlow(flow_gyro_corrected)(1) - static_cast<float>
							      (_aid_src_optical_flow.observation[1]);

			// recalculate the test ratio as the measurement jacobian is highly non linear
			// when close to the ground (singularity at 0) and the innovation can suddenly become really
			// large and destabilize the filter
			_aid_src_optical_flow.test_ratio[1] = sq(_aid_src_optical_flow.innovation[1]) / (sq(
					_params.flow_innov_gate) * _aid_src_optical_flow.innovation_variance[1]);

			if (_aid_src_optical_flow.test_ratio[1] > 1.f) {
				continue;
			}
		}

		if (_aid_src_optical_flow.innovation_variance[index] < _aid_src_optical_flow.observation_variance[index]) {
			// we need to reinitialise the covariance matrix and abort this fusion step
			ECL_ERR("Opt flow error - covariance reset");
			initialiseCovariance();
			return false;
		}

		VectorState Kfusion = P * H / _aid_src_optical_flow.innovation_variance[index];

		if (!update_terrain) {
			Kfusion(State::terrain.idx) = 0.f;
		}

		measurementUpdate(Kfusion, H, _aid_src_optical_flow.observation_variance[index],
				  _aid_src_optical_flow.innovation[index]);
	}

	_fault_status.flags.bad_optflow_X = false;
	_fault_status.flags.bad_optflow_Y = false;

	_innov_check_fail_status.flags.reject_optflow_X = (_aid_src_optical_flow.test_ratio[0] > 1.f);
	_innov_check_fail_status.flags.reject_optflow_Y = (_aid_src_optical_flow.test_ratio[1] > 1.f);

	_aid_src_optical_flow.time_last_fuse = _time_delayed_us;
	_aid_src_optical_flow.fused = true;

	_time_last_hor_vel_fuse = _time_delayed_us;

	if (update_terrain) {
		_time_last_terrain_fuse = _time_delayed_us;
	}

	return true;
}

float Ekf::predictFlowHagl() const
{
	// calculate the sensor position relative to the IMU
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the sensor position relative to the IMU in earth frame
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	// calculate the height above the ground of the optical flow camera. Since earth frame is NED
	// a positive offset in earth frame leads to a smaller height above the ground.
	const float height_above_gnd_est = fabsf(getHagl() - pos_offset_earth(2));

	// Never return a really small value to avoid generating insanely large flow innovations
	// that could destabilize the filter
	constexpr float min_hagl = 1e-2f;

	return fmaxf(height_above_gnd_est, min_hagl);
}
float Ekf::predictFlowRange() const
{
	// calculate range from focal point to centre of image
	// absolute distance to the frame region in view
	return predictFlowHagl() / _R_to_earth(2, 2);
}

Vector2f Ekf::predictFlow(const Vector3f &flow_gyro) const
{
	// calculate the sensor position relative to the IMU
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// calculate the velocity of the sensor relative to the imu in body frame
	// Note: flow gyro is the negative of the body angular velocity, thus use minus sign
	const Vector3f vel_rel_imu_body = -flow_gyro % pos_offset_body;

	// calculate the velocity of the sensor in the earth frame
	const Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

	// rotate into body frame
	const Vector2f vel_body = _state.quat_nominal.rotateVectorInverse(vel_rel_earth).xy();

	// calculate range from focal point to centre of image
	const float scale = _R_to_earth(2, 2) / predictFlowHagl();

	return Vector2f(vel_body(1) * scale, -vel_body(0) * scale);
}

float Ekf::calcOptFlowMeasVar(const flowSample &flow_sample) const
{
	// calculate the observation noise variance - scaling noise linearly across flow quality range
	const float R_LOS_best = fmaxf(_params.flow_noise, 0.05f);
	const float R_LOS_worst = fmaxf(_params.flow_noise_qual_min, 0.05f);

	// calculate a weighting that varies between 1 when flow quality is best and 0 when flow quality is worst
	float weighting = (255.f - (float)_params.flow_qual_min);

	if (weighting >= 1.f) {
		weighting = math::constrain((float)(flow_sample.quality - _params.flow_qual_min) / weighting, 0.f, 1.f);

	} else {
		weighting = 0.0f;
	}

	// take the weighted average of the observation noise for the best and wort flow quality
	const float R_LOS = sq(R_LOS_best * weighting + R_LOS_worst * (1.f - weighting));

	return R_LOS;
}
