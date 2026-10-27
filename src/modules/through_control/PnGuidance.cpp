/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "PnGuidance.hpp"

#include <ControlMath.hpp>
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

namespace through
{

Vector3f PnGuidance::hoverAcceleration(const Vector3f &v_ned) const
{
	return -_params.k_vdamp * v_ned;
}

Vector3f PnGuidance::specificForce(const Vector3f &a_ned)
{
	return a_ned - Vector3f(0.f, 0.f, GRAVITY);
}

GuidanceCommand PnGuidance::guide(const Vector3f &u, const Vector3f &omega, const Vector3f &v_ned,
				  float thr_base, float hover_thrust) const
{
	GuidanceCommand cmd{};
	const Vector3f g_vec{0.f, 0.f, GRAVITY};

	// Step 3: closing velocity (target at rest)
	cmd.vc = v_ned.dot(u);
	const float vc = fmaxf(cmd.vc, _params.vc_min);

	// Step 4: PN lateral acceleration, perpendicular to u by construction
	Vector3f a_perp = _params.pn_n * vc * omega.cross(u);
	a_perp -= a_perp.dot(u) * u; // remove numerical residue along u

	const float a_perp_norm = a_perp.norm();

	if (a_perp_norm > _params.a_max) {
		a_perp *= _params.a_max / a_perp_norm;
		cmd.a_perp_limited = true;
	}

	// Step 5: specific force f = w + s * u with w = a_perp - g_vec
	const float f0 = GRAVITY * thr_base / hover_thrust;
	const float f_max = GRAVITY * _params.thr_max / hover_thrust;

	const Vector3f g_perp = g_vec - g_vec.dot(u) * u;
	Vector3f w_perp = a_perp - g_perp;

	// 5a. feasibility: the force perpendicular to the LOS must not exceed f_max. Keep the
	// gravity compensation and shrink a_perp: largest k in [0, 1] with |k * a_perp - g_perp| <= f_max
	if (w_perp.norm() > f_max) {
		const float aa = a_perp.dot(a_perp);
		const float ag = a_perp.dot(g_perp);
		const float c = g_perp.dot(g_perp) - f_max * f_max;
		float k = 0.f;

		if (aa > FLT_EPSILON && c < 0.f) {
			k = math::constrain((ag + sqrtf(ag * ag - aa * c)) / aa, 0.f, 1.f);
		}

		a_perp *= k;
		w_perp = a_perp - g_perp;
		cmd.a_perp_limited = true;
	}

	const float wu = -g_vec.dot(u);

	// 5b. along-LOS acceleration giving |f| == f0 (forward root); if f0 cannot cover the
	// perpendicular demand, take the minimum thrust solution (thrust above the base)
	const float disc = f0 * f0 - w_perp.dot(w_perp);
	const float s_thr = (disc >= 0.f) ? -wu + sqrtf(disc) : -wu;

	// 5c. speed limit along the LOS
	const float s_vlim = _params.k_vlim * (_params.v_max - cmd.vc);

	cmd.a_perp = a_perp;
	cmd.s = fminf(s_thr, s_vlim);
	cmd.a = a_perp + cmd.s * u;
	return cmd;
}

AttitudeCommand PnGuidance::forceToAttitude(const Vector3f &f_ned, float yaw_sp, float hover_thrust) const
{
	AttitudeCommand cmd{};

	const float f_norm = f_ned.norm();

	// body z axis (FRD, pointing down when level) is opposite to the specific force
	Vector3f body_z = (f_norm > FLT_EPSILON) ? Vector3f(-f_ned / f_norm) : Vector3f(0.f, 0.f, 1.f);

	const Vector3f world_down{0.f, 0.f, 1.f};
	const float tilt = acosf(math::constrain(body_z.dot(world_down), -1.f, 1.f));

	if (tilt > _params.tilt_max) {
		ControlMath::limitTilt(body_z, world_down, _params.tilt_max);
		cmd.tilt_limited = true;
	}

	// thrust from the force magnitude: hover thrust produces exactly 1 g
	const float thrust_raw = f_norm * hover_thrust / GRAVITY;
	cmd.thrust = math::constrain(thrust_raw, _params.thr_min, _params.thr_max);
	cmd.thrust_saturated = (thrust_raw < _params.thr_min) || (thrust_raw > _params.thr_max);

	cmd.f_cmd = -body_z * (cmd.thrust * GRAVITY / hover_thrust);

	vehicle_attitude_setpoint_s att_sp{};
	ControlMath::bodyzToAttitude(body_z, yaw_sp, att_sp);
	cmd.q_d = Quatf(att_sp.q_d);

	return cmd;
}

float PnGuidance::losYaw(const Vector3f &u_ned, float yaw_prev, float min_horizontal)
{
	const Vector2f u_xy(u_ned(0), u_ned(1));

	if (u_xy.norm() < min_horizontal) {
		return yaw_prev;
	}

	return atan2f(u_ned(1), u_ned(0));
}

float PnGuidance::rateLimitYaw(float yaw_prev, float yaw_target, float max_rate, float dt)
{
	const float max_step = fmaxf(max_rate * dt, 0.f);
	const float error = wrap_pi(yaw_target - yaw_prev);
	return wrap_pi(yaw_prev + math::constrain(error, -max_step, max_step));
}

} // namespace through
