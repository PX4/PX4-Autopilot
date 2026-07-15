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
	// get raw accelerometer reading at delayed horizon and expected measurement noise (gaussian)
	const Vector3f measurement = Vector3f(imu.delta_vel / imu.delta_vel_dt - _state.accel_bias).unit();
	const float measurement_var = math::max(sq(_params.ekf2_grav_noise), sq(0.01f));

	// The accel magnitude gate is a proxy check only: |a| close to 1g does not imply the
	// direction is gravity (a horizontal acceleration of 0.45g still passes it), so the
	// fusion must stay weak (measurement noise) and tightly gated (innovation gate) to
	// avoid pulling the attitude towards the thrust axis during manoeuvres.
	const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
	const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;
	const float accel_lpf_norm_sq = _accel_lpf.getState().norm_squared();
	const bool accel_lpf_norm_good = (accel_lpf_norm_sq > sq(lower_accel_limit))
					 && (accel_lpf_norm_sq < sq(upper_accel_limit));

	// fuse gravity observation if our overall acceleration isn't too big
	_control_status.flags.gravity_vector = (_params.ekf2_imu_ctrl & static_cast<int32_t>(ImuCtrl::GravityVector))
					       && (accel_lpf_norm_good || _control_status.flags.vehicle_at_rest)
					       && !isHorizontalAidingActive()
					       && _control_status.flags.tilt_align; // Let fake position do the initial alignment (more robust before takeoff)

	// calculate kalman gains and innovation variances
	Vector3f innovation = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f, -1.f)) - measurement;
	Vector3f innovation_variance;
	const auto state_vector = _state.vector();
	VectorState H;
	sym::ComputeGravityXyzInnovVarAndHx(state_vector, P, measurement_var, &innovation_variance, &H);

	// fill estimator aid source status
	updateAidSourceStatus(_aid_src_gravity,
			      imu.time_us,                                                 // sample timestamp
			      measurement,                                                 // observation
			      Vector3f{measurement_var, measurement_var, measurement_var}, // observation variance
			      innovation,                                                  // innovation
			      innovation_variance,                                         // innovation variance
			      0.25f);                                                      // innovation gate

	// update the states and covariance using sequential fusion
	bool fused[3] {};

	for (uint8_t index = 0; index <= 2; index++) {
		// Calculate Kalman gains and observation jacobians
		if (index == 0) {
			// everything was already computed above

		} else if (index == 1) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeGravityYInnovVarAndH(state_vector, P, measurement_var, &_aid_src_gravity.innovation_variance[index], &H);

			// recalculate innovation using the updated state
			_aid_src_gravity.innovation[index] = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f,
							     -1.f))(index) - measurement(index);

		} else if (index == 2) {
			// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
			sym::ComputeGravityZInnovVarAndH(state_vector, P, measurement_var, &_aid_src_gravity.innovation_variance[index], &H);

			// recalculate innovation using the updated state
			_aid_src_gravity.innovation[index] = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f,
							     -1.f))(index) - measurement(index);
		}

		VectorState K = P * H / _aid_src_gravity.innovation_variance[index];

		const bool accel_clipping = imu.delta_vel_clipping[0] || imu.delta_vel_clipping[1] || imu.delta_vel_clipping[2];

		if (_control_status.flags.gravity_vector && !_aid_src_gravity.innovation_rejected && !accel_clipping) {
			K(State::quat_nominal.idx + 2) = 0.f; // no heading corrections via tilt-heading correlation
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

	// Recovery from a fusion dropout (#24299): without horizontal aiding, tilt error built up
	// while gravity fusion was unavailable (sustained |a| outside the enable band) can exceed
	// the innovation gate and lock fusion out indefinitely. If the vehicle has been quasi-static
	// with fusion intended but rejected for longer than the timeout, re-initialise the tilt from
	// the low-passed accelerometer instead of weakening the gate or the measurement noise.
	// The timeout must be long enough that a sustained horizontal acceleration (which biases the
	// measured direction while keeping |a| near 1g) is unlikely to persist for the whole period.
	static constexpr uint64_t kGravityResetTimeoutUs = 5'000'000;
	static constexpr float kQuasiStaticGyroNormLimit = 0.25f; // rad/s

	if (_control_status.flags.gravity_vector && _control_status.flags.in_air
	    && _aid_src_gravity.innovation_rejected
	    && isTimedOut(_aid_src_gravity.time_last_fuse, kGravityResetTimeoutUs)
	    && (_gyro_lpf.getState().norm() < kQuasiStaticGyroNormLimit)) {

		resetTiltUsingAccelVector();
		resetAidSourceStatusZeroInnovation(_aid_src_gravity);
	}
}

void Ekf::resetTiltUsingAccelVector()
{
	const Quatf quat_before_reset = _state.quat_nominal;

	// minimal rotation bringing the predicted gravity direction (body frame) onto the
	// direction measured by the low-passed accelerometer, leaving heading unchanged
	const Vector3f measured_up = _accel_lpf.getState().unit();
	const Vector3f predicted_up = _state.quat_nominal.rotateVectorInverse(Vector3f(0.f, 0.f, -1.f));

	const Quatf dq(measured_up, predicted_up);
	_state.quat_nominal = (_state.quat_nominal * dq).normalized();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// reset the tilt variance to the alignment accuracy, keep the heading variance
	const float tilt_var = sq(math::max(_params.ekf2_angerr_init, 0.01f));
	P.uncorrelateCovarianceSetVariance<2>(State::quat_nominal.idx, tilt_var);

	propagateQuatReset(quat_before_reset);
}
