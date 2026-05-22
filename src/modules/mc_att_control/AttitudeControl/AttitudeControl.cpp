/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// compensate for the effect of the yaw weight rescaling the output
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}
}

void AttitudeControl::setRefModelFrequency(float omega_n)
{
	_omega_n = math::max(omega_n, 0.1f);
	_kq      = _omega_n * _omega_n;
}

void AttitudeControl::setAttitudeSetpoint(const Quatf &qd, const float yawspeed_setpoint, const float dt)
{
	Quatf qd_normalized = qd;
	qd_normalized.normalize();

	if (_ref_initialized && dt > 0.f) {
		// 2nd-order critically damped ref model with exact (ZOH) discretisation.
		// Repeated eigenvalue at s = -_omega_n; unconditionally stable for any dt.

		// Tangent-space inputs: rotate the analytical yaw rate into q_ref's body
		//    frame, and form the small-angle error vector from q_ref to q_d.
		const Vector3f w_known_in_ref = std::isfinite(yawspeed_setpoint)
						? _q_ref.inversed().dcm_z() * yawspeed_setpoint
						: Vector3f{};

		Quatf q_err = _q_ref.inversed() * qd_normalized;
		q_err.canonicalize();
		const Vector3f e = 2.f * q_err.imag();

		// Entries of exp(A*dt) for A = [0 -1; _kq -2*_omega_n], using emt = exp(-_omega_n*dt).
		const float w_dt  = _omega_n * dt;
		const float emt   = expf(-w_dt);
		const float a     = (1.f + w_dt) * emt;
		const float b     = dt * emt;
		const float gamma = _kq * dt * emt;
		const float delta = (1.f - w_dt) * emt;

		// Propagate in tangent space. delta_phi is the integral of omega over [0, dt];
		//    the w_offset part collapses to e(0) - e(dt) since e_dot = -w_offset.
		const Vector3f w_offset     = _omega_ref - w_known_in_ref;
		const Vector3f delta_phi    = (1.f - a) * e + b * w_offset + w_known_in_ref * dt;
		const Vector3f w_offset_new = gamma * e + delta * w_offset;

		// Lift back to SO(3): undo the offset substitution and apply the integrated
		//    rotation to q_ref by right-multiplying (body-frame composition).
		_omega_ref = w_offset_new + w_known_in_ref;
		_q_ref     = _q_ref * Quatf(AxisAnglef(delta_phi));
		_q_ref.normalize();

	} else {
		// First call (or dt out of range): snap reference to the current setpoint.
		_q_ref = qd_normalized;
		_omega_ref.zero();
		_ref_initialized = true;
	}

	_attitude_setpoint_q = qd_normalized;
}

void AttitudeControl::adaptAttitudeSetpoint(const Quatf &q_delta)
{
	_attitude_setpoint_q = q_delta * _attitude_setpoint_q;
	_attitude_setpoint_q.normalize();
	// Apply the same world-frame delta to the reference attitude. _omega_ref is in
	// the reference body frame and physically invariant under a world relabeling.
	_q_ref = q_delta * _q_ref;
	_q_ref.normalize();
}

matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	// FF off (autotune, MC_REF_FF=0): P targets the raw setpoint. FF on: P targets _q_ref.
	const bool ff_active = _ff_enabled && (_ff_gain > 0.f);
	Quatf qd = ff_active ? _q_ref : _attitude_setpoint_q;

	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		// full attitude control anyways generates no yaw input and directly takes the combination of
		// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
		qd_red = qd;

	} else {
		// Transform rotation from current to desired thrust vector into a world frame reduced desired attitude.
		// This is a right multiplication as the tilt error quaternion is obtained from two Z vectors expressed in the world frame.
		qd_red *= q;
	}

	// With a full desired attitude given by: qd = qd_red * qd_dyaw, extract the delta yaw component.
	// By definition, the delta yaw quaternion has the form (cos(angle/2), 0, 0, sin(angle/2))
	Quatf qd_dyaw = qd_red.inversed() * qd;
	qd_dyaw.canonicalize();
	// catch numerical problems with the domain of acosf and asinf
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	// scale the delta yaw angle and re-combine the desired attitude
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = q.inversed() * qd;

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	const Vector3f eq = 2.f * qe.canonical().imag();

	// calculate angular rates setpoint
	Vector3f rate_setpoint = eq.emult(_proportional_gain);

	if (ff_active) {
		// Rotate _omega_ref from q_ref's body frame into the current body frame.
		Vector3f omega_ff = _ff_gain * q.rotateVectorInverse(_q_ref.rotateVector(_omega_ref));

		if (_ff_max > 0.f) {
			for (int i = 0; i < 3; i++) {
				omega_ff(i) = math::constrain(omega_ff(i), -_ff_max, _ff_max);
			}
		}

		rate_setpoint += omega_ff;
	}

	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
