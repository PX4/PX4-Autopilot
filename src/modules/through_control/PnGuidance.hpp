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

/**
 * @file PnGuidance.hpp
 *
 * Pure guidance math of the THROUGH mode (spec section 7, steps 3-6 and the
 * common hover command). No uORB I/O, so it can be unit tested in isolation.
 *
 * Frames: NED world frame, gravity vector g_vec = [0, 0, g].
 * Specific force: f = a - g_vec (what the rotors must produce per unit mass).
 */

#pragma once

#include <matrix/matrix/math.hpp>

namespace through
{

struct AttitudeCommand {
	matrix::Quatf q_d{};              ///< attitude setpoint (FRD body -> NED)
	float thrust{0.f};                ///< normalized collective thrust [0, 1]
	matrix::Vector3f f_cmd{};         ///< specific force command after limiting [m/s^2] (NED)
	bool tilt_limited{false};
	bool thrust_saturated{false};
};

struct GuidanceCommand {
	matrix::Vector3f a_perp{};        ///< PN lateral acceleration after limiting [m/s^2] (NED)
	float s{0.f};                     ///< along-LOS acceleration [m/s^2]
	matrix::Vector3f a{};             ///< total acceleration a_perp + s * u [m/s^2] (NED)
	float vc{0.f};                    ///< closing velocity v . u before the Vc_min clamp [m/s]
	bool a_perp_limited{false};
};

class PnGuidance
{
public:
	static constexpr float GRAVITY = 9.80665f;

	struct Params {
		float thr_min{0.3f};          ///< [norm]
		float thr_max{0.9f};          ///< [norm]
		float tilt_max{1.0472f};      ///< [rad]
		float k_vdamp{1.5f};          ///< [1/s]
		float pn_n{3.5f};             ///< PN navigation constant
		float a_max{9.8f};            ///< [m/s^2] limit of |a_perp|
		float v_max{10.f};            ///< [m/s] speed limit along the LOS
		float k_vlim{2.f};            ///< [1/s] gain of the speed limiter
		float vc_min{1.f};            ///< [m/s] minimum closing velocity used by PN
	};

	void setParams(const Params &params) { _params = params; }
	const Params &params() const { return _params; }

	/**
	 * Common hover command used in ALIGN and RAMP: velocity damping only,
	 * a_hover = -k_vdamp * v (altitude is allowed to drift slightly).
	 */
	matrix::Vector3f hoverAcceleration(const matrix::Vector3f &v_ned) const;

	/** Specific force for a desired acceleration: f = a - g_vec */
	static matrix::Vector3f specificForce(const matrix::Vector3f &a_ned);

	/**
	 * Steps 3-5: PN lateral acceleration and the along-LOS acceleration that keeps
	 * the thrust at thr_base (fixed base thrust with limited float).
	 *
	 * @param u unit LOS vector (NED)
	 * @param omega LOS rate omega_LOS [rad/s] (NED)
	 * @param v_ned vehicle velocity [m/s]
	 * @param thr_base base thrust [norm]; RAMP ramps it from the hover thrust to THRU_THR_BASE
	 * @param hover_thrust hover thrust T_h [norm]
	 */
	GuidanceCommand guide(const matrix::Vector3f &u, const matrix::Vector3f &omega, const matrix::Vector3f &v_ned,
			      float thr_base, float hover_thrust) const;

	/**
	 * Step 6: specific force + yaw -> attitude setpoint and normalized thrust.
	 * The tilt of body_z = -f/|f| from the vertical is limited to tilt_max
	 * keeping its horizontal direction. thrust = |f| * T_h / g, constrained to
	 * [thr_min, thr_max].
	 */
	AttitudeCommand forceToAttitude(const matrix::Vector3f &f_ned, float yaw_sp, float hover_thrust) const;

	/**
	 * Yaw target pointing along the LOS azimuth. Returns yaw_prev if the LOS is
	 * nearly vertical (|horizontal part| < min_horizontal), where atan2 is unstable.
	 */
	static float losYaw(const matrix::Vector3f &u_ned, float yaw_prev, float min_horizontal = 0.1f);

	/** Move yaw from yaw_prev towards yaw_target by at most max_rate * dt (shortest way). */
	static float rateLimitYaw(float yaw_prev, float yaw_target, float max_rate, float dt);

private:
	Params _params{};
};

} // namespace through
