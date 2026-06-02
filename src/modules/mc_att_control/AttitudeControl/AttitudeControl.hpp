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
 * @file AttitudeControl.hpp
 *
 * A quaternion based attitude controller.
 *
 * @author Matthias Grob	<maetugr@gmail.com>
 *
 * Publication documenting the implemented Quaternion Attitude Control:
 * Nonlinear Quadrocopter Attitude Control (2013)
 * by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
 * Institute for Dynamic Systems and Control (IDSC), ETH Zurich
 *
 * https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

class AttitudeControl
{
public:
	AttitudeControl() = default;
	~AttitudeControl() = default;

	/**
	 * Set proportional attitude control gain
	 * @param proportional_gain 3D vector containing gains for roll, pitch, yaw
	 * @param yaw_weight A fraction [0,1] deprioritizing yaw compared to roll and pitch
	 */
	void setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight);

	/// Set the reference-model natural frequency [rad/s] (zeta=1).
	void setRefModelFrequency(float omega_n);

	/// Set the FF magnitude scaling [0..1]; 0 disables the FF.
	void setFeedForwardGain(float gain) { _ff_gain = math::constrain(gain, 0.f, 1.f); }

	/// Set per-axis saturation on the FF angular-velocity contribution [rad/s]; 0 = disabled.
	void setFeedForwardLimit(float limit) { _ff_max = math::max(limit, 0.f); }

	/**
	 * Set hard limit for output rate setpoints
	 * @param rate_limit [rad/s] 3D vector containing limits for roll, pitch, yaw
	 */
	void setRateLimit(const matrix::Vector3f &rate_limit) { _rate_limit = rate_limit; }

	/**
	 * Set a new attitude setpoint replacing the one tracked before
	 * @param qd desired vehicle attitude setpoint
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 * @param dt [s] time since previous setpoint; <= 0 skips the FF derivative update
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint, const float dt = -1.f);

	/**
	 * Adjust last known attitude setpoint by a delta rotation
	 * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
	 * @param q_delta delta rotation to apply
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta);

	/**
	 * Gate the setpoint-derivative feedforward (the filter keeps running, only
	 * its addition to the rate setpoint is suppressed). Used during autotune.
	 */
	void setFeedForwardEnabled(bool enabled) { _ff_enabled = enabled; }

	/**
	 * Run one control loop cycle calculation
	 * @param q estimation of the current vehicle attitude unit quaternion
	 * @return [rad/s] body frame 3D angular rate setpoint vector to be executed by the rate controller
	 */
	matrix::Vector3f update(const matrix::Quatf &q) const;

	/**
	 * Reference attitude tracked by the model (the smoothed target the cascaded
	 * controller actually follows). Exposed for tests and diagnostics.
	 */
	const matrix::Quatf &getReferenceAttitude() const { return _q_ref; }

private:
	/**
	 * Advance the 2nd-order reference model by one step toward the desired attitude
	 * using an exact (ZOH) discretization, updating _q_ref and _omega_ref.
	 * @param qd normalized desired attitude setpoint
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 * @param dt [s] time since previous setpoint (> 0)
	 */
	void propagateReferenceModel(const matrix::Quatf &qd, const float yawspeed_setpoint, const float dt);

	matrix::Vector3f _proportional_gain;
	matrix::Vector3f _rate_limit;
	float _yaw_w{0.f}; ///< yaw weight [0,1] to deprioritize compared to roll and pitch

	matrix::Quatf _attitude_setpoint_q; ///< latest known raw attitude setpoint e.g. from position control

	matrix::Quatf _q_ref;                  ///< reference attitude tracked by the 2nd-order ref model
	matrix::Vector3f _omega_ref;           ///< reference body angular velocity (FF output)
	bool _ref_initialized{false};

	bool _ff_enabled{true};

	float _omega_n{50.f};                  ///< ref-model natural frequency [rad/s] (MC_REF_W_N)
	float _kq{_omega_n * _omega_n};        ///< stiffness coefficient, kept in sync with _omega_n

	float _ff_gain{1.f};                   ///< FF magnitude scaling 0..1 (MC_REF_FF)
	float _ff_max{0.f};                    ///< per-axis FF rate cap [rad/s]; 0 = disabled (MC_REF_FF_MAX)
};
