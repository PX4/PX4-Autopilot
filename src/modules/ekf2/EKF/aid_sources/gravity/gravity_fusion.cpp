/****************************************************************************
 *
 *   Copyright (c) 2023-2026 PX4 Development Team. All rights reserved.
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
 * @file gravity_fusion.cpp
 * Fuse observations from the gravity vector to constrain roll
 * and pitch (a la complementary filter).
 *
 * @author Daniel M. Sahu <danielmohansahu@gmail.com>
 */

#include "ekf.h"
#include <ekf_derivation/generated/compute_gravity_xyz_innov_var_and_hx.h>
#include <ekf_derivation/generated/compute_gravity_y_innov_var_and_h.h>
#include <ekf_derivation/generated/compute_gravity_z_innov_var_and_h.h>

#include <mathlib/mathlib.h>

void Ekf::controlGravityFusion(const imuSample &imu)
{
	// Bias-corrected specific force; unit vector is gravity proxy when near 1 g
	const Vector3f accel = imu.delta_vel / imu.delta_vel_dt - _state.accel_bias;
	const float accel_ratio = _accel_lpf.getState().norm() / CONSTANTS_ONE_G;
	const float excess = fabsf(accel_ratio - 1.f);

	// Soft enable 0.5–2 g (was hard 0.9–1.1 g). Inflate noise as |a| leaves 1 g.
	// Cap unit-vector sigma so default EKF2_GRAV_NOISE=1 still corrects tilt (#24299).
	const bool accel_ok = (accel_ratio > 0.5f) && (accel_ratio < 2.f);
	float measurement_var = sq(math::min(math::max(_params.ekf2_grav_noise, 0.05f), 0.3f));
	measurement_var *= math::max(1.f, sq(excess * 10.f));

	const Vector3f measurement = accel.unit();

	_control_status.flags.gravity_vector = (_params.ekf2_imu_ctrl & static_cast<int32_t>(ImuCtrl::GravityVector))
					       && (accel_ok || _control_status.flags.vehicle_at_rest)
					       && !isHorizontalAidingActive()
					       && _control_status.flags.tilt_align;

	Vector3f innovation = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f, -1.f)) - measurement;

	// Floor tilt variance near hover with large residual so over-confident P can re-acquire
	if (_control_status.flags.gravity_vector && (excess < 0.15f)
	    && (sq(innovation(0)) + sq(innovation(1)) > sq(0.15f))) {
		const float min_var = sq(math::radians(15.f));
		P(State::quat_nominal.idx, State::quat_nominal.idx) = math::max(P(State::quat_nominal.idx,
				State::quat_nominal.idx), min_var);
		P(State::quat_nominal.idx + 1, State::quat_nominal.idx + 1) = math::max(P(State::quat_nominal.idx + 1,
				State::quat_nominal.idx + 1), min_var);
	}

	Vector3f innovation_variance;
	const auto state_vector = _state.vector();
	VectorState H;
	sym::ComputeGravityXyzInnovVarAndHx(state_vector, P, measurement_var, &innovation_variance, &H);

	// Gate 1.0 (was 0.25) so large residuals after dropout are not rejected
	updateAidSourceStatus(_aid_src_gravity,
			      imu.time_us,
			      measurement,
			      Vector3f{measurement_var, measurement_var, measurement_var},
			      innovation,
			      innovation_variance,
			      1.f);

	bool fused[3] {};

	for (uint8_t index = 0; index <= 2; index++) {
		if (index == 1) {
			sym::ComputeGravityYInnovVarAndH(state_vector, P, measurement_var, &_aid_src_gravity.innovation_variance[index], &H);
			_aid_src_gravity.innovation[index] = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f,
							     -1.f))(index) - measurement(index);

		} else if (index == 2) {
			sym::ComputeGravityZInnovVarAndH(state_vector, P, measurement_var, &_aid_src_gravity.innovation_variance[index], &H);
			_aid_src_gravity.innovation[index] = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f,
							     -1.f))(index) - measurement(index);
		}

		VectorState K = P * H / _aid_src_gravity.innovation_variance[index];
		const bool accel_clipping = imu.delta_vel_clipping[0] || imu.delta_vel_clipping[1] || imu.delta_vel_clipping[2];

		if (_control_status.flags.gravity_vector && !_aid_src_gravity.innovation_rejected && !accel_clipping) {
			K(State::quat_nominal.idx + 2) = 0.f; // no heading corrections via tilt–heading correlation
			fused[index] = measurementUpdate(K, H,
							 _aid_src_gravity.observation_variance[index], _aid_src_gravity.innovation[index]);
		}
	}

	if (fused[0] && fused[1] && fused[2]) {
		_aid_src_gravity.fused = true;
		_aid_src_gravity.time_last_fuse = imu.time_us;

	} else {
		_aid_src_gravity.fused = false;
	}
}
