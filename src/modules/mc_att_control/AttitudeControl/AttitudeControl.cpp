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

static __attribute__((noinline)) Quatf qmul(const Quatf &a, const Quatf &b) { return a * b; }
static __attribute__((noinline)) Quatf qinv(const Quatf &q) { return q.inversed(); }
static __attribute__((noinline)) Vector3f qzaxis(const Quatf &q) { return q.dcm_z(); }

// The acceleration-limited reference is advanced in substeps no longer than this, so that the rate setpoint of
// its angle loop is refreshed from the remaining error often enough; the substep count is bounded to keep the work
// per update finite. Longer gaps between setpoints are still closed, only with longer, more conservative substeps.
static constexpr float kMaxShapingStep = 0.001f; // [s]
static constexpr int kMaxShapingSubsteps = 50;

/**
 * Angle loop of the cascaded reference model: proportional to the error close to the setpoint, and following the
 * braking curve sqrt(2 * accel_max * error) further out so that it never asks for more deceleration than the limit.
 * Both parts meet with the same value and slope at |error| = accel_max / gain^2.
 * @param error [rad] remaining angle error
 * @param gain [1/s] proportional gain of the angle loop
 * @param accel_max [rad/s^2] angular acceleration limit (> 0)
 * @return [rad/s] rate setpoint
 */
static float brakingRate(const float error, const float gain, const float accel_max)
{
	const float linear_error = accel_max / (gain * gain);
	const float abs_error = fabsf(error);
	const float rate = (abs_error <= linear_error) ? gain * abs_error
			   : sqrtf(2.f * accel_max * (abs_error - 0.5f * linear_error));

	return (error < 0.f) ? -rate : rate;
}

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

void AttitudeControl::setRefModelAccelerationLimit(const Vector3f &accel_max)
{
	for (int i = 0; i < 3; i++) {
		_ref_accel_max(i) = math::max(accel_max(i), 0.f);
	}

	_yawspeed_command.setSlewRate(_ref_accel_max(2));
}

bool AttitudeControl::isAxisAccelerationLimited(const int axis) const
{
	return _ref_accel_max(axis) > FLT_EPSILON;
}

bool AttitudeControl::setAttitudeSetpoint(const Quatf &qd, const float yawspeed_setpoint, const float dt)
{
	// The reference model integrates from its previous state, so a single invalid setpoint would make
	// it NaN until reboot. Keep tracking the last valid reference instead.
	// A zero, non-finite or overflowing quaternion has no attitude: normalizing it gives NaN or zero.
	// Keep in sync with handle_message_set_attitude_target() in mavlink_receiver.cpp, which drops
	// these messages so they do not keep offboard alive.
	const float qd_norm = qd.norm();

	if (!PX4_ISFINITE(qd_norm) || (qd_norm < 1e-3f)) {
		return false;
	}

	const Quatf qd_normalized = qd / qd_norm;

	if (_ref_initialized && dt > 0.f) {
		propagateReferenceModel(qd_normalized, yawspeed_setpoint, dt);

	} else {
		// First call (or dt out of range): snap reference to the current setpoint.
		_q_ref = qd_normalized;
		_omega_correction.zero();
		_ref_accel.zero();
		_omega_command.zero();
		_yawspeed_command.setForcedValue(0.f);
		_ref_initialized = true;
	}

	return true;
}

void AttitudeControl::propagateLimitedAxis(const int axis, const float error, const float dt, float &rate,
		float &delta_angle, float &accel) const
{
	// The linear model omega_dot = _kq * e - 2 * _omega_n * omega written as a cascade: an angle loop with gain
	// _omega_n / 2 commands the rate, a rate loop with gain 2 * _omega_n commands the acceleration. The acceleration
	// is saturated at the limit and the rate setpoint at the rate limit. Far from the setpoint the angle loop brakes
	// along a square root (brakingRate), otherwise the rate it builds up could not be stopped within the limit.
	const float accel_max = _ref_accel_max(axis);
	const float angle_gain = 0.5f * _omega_n;

	// Advance in substeps so that the rate setpoint follows the remaining error also over long setpoint intervals.
	const int substeps = math::constrain(static_cast<int>(ceilf(dt / kMaxShapingStep)), 1, kMaxShapingSubsteps);
	const float substep_dt = dt / substeps;

	// Exact (ZOH) step of the linear model over one substep, same closed form as in propagateReferenceModel()
	const float w_dt  = _omega_n * substep_dt;
	const float emt   = expf(-w_dt);
	const float a     = (1.f + w_dt) * emt;
	const float b     = substep_dt * emt;
	const float gamma = _kq * substep_dt * emt;
	const float delta = (1.f - w_dt) * emt;

	delta_angle = 0.f;
	accel = 0.f;

	for (int i = 0; i < substeps; i++) {
		const float rate_setpoint = math::constrain(brakingRate(error - delta_angle, angle_gain, accel_max),
					    -_rate_limit(axis), _rate_limit(axis));

		// Without saturation the cascade is the linear model driven by the error that maps to this rate setpoint,
		// which close to the setpoint is the error itself: then the step is exactly the one of the linear model.
		const float error_equivalent = rate_setpoint / angle_gain;
		const float rate_linear = gamma * error_equivalent + delta * rate;
		accel = (rate_linear - rate) / substep_dt;

		if (fabsf(accel) <= accel_max) {
			delta_angle += (1.f - a) * error_equivalent + b * rate;
			rate = rate_linear;

		} else {
			// saturated rate loop: constant acceleration at the limit over the substep
			accel = math::constrain(accel, -accel_max, accel_max);
			const float rate_next = rate + accel * substep_dt;
			delta_angle += 0.5f * (rate + rate_next) * substep_dt;
			rate = rate_next;
		}
	}
}

void AttitudeControl::propagateReferenceModel(const Quatf &qd, const float yawspeed_setpoint, const float dt)
{
	// 2nd-order critically damped ref model with exact (ZOH) discretisation.
	// Repeated eigenvalue at s = -_omega_n; unconditionally stable for any dt.

	// Tangent-space inputs: rotate the analytical yaw rate into q_ref's body
	//    frame, and form the small-angle error vector from q_ref to q_d.
	const Quatf q_ref_inv = qinv(_q_ref);
	const Vector3f yaw_axis_body = qzaxis(q_ref_inv); // world yaw axis expressed in q_ref's body frame

	// Commanded (analytical) yaw rate. With a yaw acceleration limit it changes at most at that limit, so that yaw
	// rate commands (stick, auto) respect it too.
	const float yawspeed_command = PX4_ISFINITE(yawspeed_setpoint) ? yawspeed_setpoint : 0.f;
	const float yawspeed_command_prev = _yawspeed_command.getState();

	if (isAxisAccelerationLimited(2)) {
		_yawspeed_command.update(yawspeed_command, dt);

	} else {
		_yawspeed_command.setForcedValue(yawspeed_command);
	}

	// Commanded reference rate, kept separate so update() can feed it forward at unity.
	_omega_command = yaw_axis_body * _yawspeed_command.getState();

	Quatf q_err = qmul(q_ref_inv, qd);
	q_err.canonicalize();
	const Vector3f e = 2.f * q_err.imag();

	// Entries of exp(A*dt) for A = [0 -1; _kq -2*_omega_n]. A has the repeated eigenvalue lambda = -_omega_n.
	// The matrix N = A - lambda*I is then nilpotent (a matrix is nilpotent when
	// N^k = 0 for some k, and here N^2 = 0). Writing exp(A*dt) = e^(lambda*dt) * exp(N*dt) and expanding the
	// series for exp(N*dt) = I + N*dt + (N*dt)^2/2! + ... , every term from (N*dt)^2
	// onward vanishes, so the exponential truncates to the exact closed form
	//     exp(A*dt) = e^(-_omega_n*dt) * [ (1 + _omega_n*dt) I + dt*A ]
	//               = emt * [ a  -b ;  gamma  delta ].
	const float w_dt  = _omega_n * dt;
	const float emt   = expf(-w_dt);
	const float a     = (1.f + w_dt) * emt;
	const float b     = dt * emt;
	const float gamma = _kq * dt * emt;
	const float delta = (1.f - w_dt) * emt;

	// While a yaw rate is commanded, StickYaw unlocks the heading and sets the setpoint yaw to the
	// measurement, so on that axis q_d carries no reference for the model to filter.
	const bool heading_unlocked = fabsf(yawspeed_command) > FLT_EPSILON;
	// Once the command ended StickYaw locks the heading where the vehicle is. A limited command that is still
	// decaying keeps turning the reference (and the vehicle with it) past that heading, and only once it stopped the
	// model closes the error to the locked heading, again within the limit.
	const bool command_decaying = !heading_unlocked && isAxisAccelerationLimited(2)
				      && ((fabsf(yawspeed_command_prev) > FLT_EPSILON) || (fabsf(_yawspeed_command.getState()) > FLT_EPSILON));
	const Vector3f e_yaw = e.dot(yaw_axis_body) * yaw_axis_body;
	const Vector3f e_filtered = (heading_unlocked || command_decaying) ? e - e_yaw : e;

	// Heading rotation applied to the reference unfiltered: the unlocked setpoint heading, or the turn of the decaying
	// command over the step (it slews linearly until it reaches the command, then stays there)
	Vector3f e_heading{};

	if (heading_unlocked) {
		e_heading = e_yaw;

	} else if (command_decaying) {
		const float yawspeed = _yawspeed_command.getState();
		const float slew_time = fabsf(yawspeed - yawspeed_command_prev) / _ref_accel_max(2);
		e_heading = yaw_axis_body * (0.5f * (yawspeed_command_prev + yawspeed) * slew_time + yawspeed * (dt - slew_time));
	}

	// Propagate the error-driven correction in tangent space (the 2nd-order state). delta_phi is the integral
	//    of omega over [0, dt]; the correction part collapses to e(0) - e(dt) since e_dot = -correction.
	Vector3f delta_phi = (1.f - a) * e_filtered + b * _omega_correction + e_heading;
	Vector3f omega_correction = gamma * e_filtered + delta * _omega_correction;

	// Axes with an angular acceleration limit run the same model in cascaded form with the acceleration saturated
	// (see propagateLimitedAxis), starting from the same state, so the limit can also be enabled at runtime. The
	// unlocked heading component is applied unfiltered on them too.
	// End-of-step error of the linear model (first row of exp(A*dt)), used for its instantaneous acceleration
	// omega_dot = _kq * e - 2 * _omega_n * omega so that the reference acceleration is also observable on
	// unconstrained axes.
	const Vector3f e_end = a * e_filtered - b * _omega_correction;

	for (int i = 0; i < 3; i++) {
		if (isAxisAccelerationLimited(i)) {
			float delta_angle;
			omega_correction(i) = _omega_correction(i);
			propagateLimitedAxis(i, e_filtered(i), dt, omega_correction(i), delta_angle, _ref_accel(i));
			delta_phi(i) = delta_angle + e_heading(i);

		} else {
			_ref_accel(i) = _kq * e_end(i) - 2.f * _omega_n * omega_correction(i);
		}
	}

	_omega_correction = omega_correction;

	_q_ref     = qmul(_q_ref, Quatf(AxisAnglef(delta_phi)));
	_q_ref.normalize();
}

void AttitudeControl::adaptAttitudeSetpoint(const Quatf &q_delta)
{
	// Apply the world-frame delta to the reference attitude. _omega_correction and _omega_command are
	// in the reference body frame and physically invariant under a world relabeling.
	_q_ref = qmul(q_delta, _q_ref);
	_q_ref.normalize();
}

matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	// The P controller always tracks the reference-model attitude.
	Quatf qd = _q_ref;

	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	const Vector3f e_z = qzaxis(q);
	const Vector3f e_z_d = qzaxis(qd);
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
	Quatf qd_dyaw = qmul(qinv(qd_red), qd);
	qd_dyaw.canonicalize();
	// catch numerical problems with the domain of acosf and asinf
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	// scale the delta yaw angle and re-combine the desired attitude
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = qmul(qinv(q), qd);

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	const Vector3f eq = 2.f * qe.canonical().imag();

	// calculate angular rates setpoint
	Vector3f rate_setpoint = eq.emult(_proportional_gain);

	// Map reference-frame rates into the current body frame.
	const Quatf q_rel = qmul(qinv(q), _q_ref);

	// The commanded reference rate (e.g. manual/auto yaw rate) is a setpoint, not a model prediction, so it
	// bypasses the reference model: it is fed forward at unity regardless of the feedforward gain and limit.
	rate_setpoint += q_rel.rotateVector(_omega_command);

	// the gain scales and the limit caps the model's error-driven anticipation (zero at gain 0)
	Vector3f omega_ff = _ff_gain * q_rel.rotateVector(_omega_correction);

	if (_ff_max > 0.f) {
		for (int i = 0; i < 3; i++) {
			omega_ff(i) = math::constrain(omega_ff(i), -_ff_max, _ff_max);
		}
	}

	rate_setpoint += omega_ff;

	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
