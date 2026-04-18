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

#pragma once

#include <stdint.h>

/**
 * Closed-loop RPM controller driving a throttle output from measured RPM.
 *
 * Runs an independent PID per motor in normalized throttle space. The
 * measured RPM is expected to already be pole-corrected mechanical RPM
 * (e.g., as computed by the DShot driver from bidirectional DShot eRPM
 * with MOT_POLE_COUNT).
 *
 * Intended usage from a driver (e.g., DShot):
 *   RpmControl _rpm_ctrl;
 *   _rpm_ctrl.setGains({kp, ki, kd, i_lim});
 *   _rpm_ctrl.setMaxRpm(max_rpm);
 *   ...
 *   if (disarm) { _rpm_ctrl.reset(i); }
 *   else       { cmd_norm = _rpm_ctrl.update(i, sp_norm, meas_rpm, dt); }
 */
class RpmControl
{
public:
	static constexpr int MAX_MOTORS = 16;

	struct Gains {
		float kp{0.f};
		float ki{0.f};
		float kd{0.f};
		float i_lim{0.5f};
	};

	RpmControl() = default;

	void setGains(const Gains &gains) { _gains = gains; }
	void setMaxRpm(float max_rpm) { _max_rpm = (max_rpm > 1.f) ? max_rpm : 1.f; }

	/** Clear the PID state for a single motor. */
	void reset(int motor);

	/**
	 * Run one PID step.
	 * @param motor     motor index in [0, MAX_MOTORS)
	 * @param sp_norm   normalized throttle setpoint from the mixer in [0, 1]
	 * @param meas_rpm  measured mechanical RPM (pole-corrected)
	 * @param dt        elapsed time since the previous call [s]
	 * @return          corrected normalized throttle command in [0, 1]
	 */
	float update(int motor, float sp_norm, float meas_rpm, float dt);

	// Accessors for status publication.
	float setpointRpm(int motor) const { return inRange(motor) ? _sp_rpm[motor] : 0.f; }
	float measuredRpm(int motor) const { return inRange(motor) ? _meas_rpm[motor] : 0.f; }
	float lastCmdNorm(int motor) const { return inRange(motor) ? _last_cmd_norm[motor] : 0.f; }
	float integral(int motor)    const { return inRange(motor) ? _integral[motor]     : 0.f; }

private:
	static bool inRange(int motor) { return (motor >= 0) && (motor < MAX_MOTORS); }

	Gains _gains{};
	float _max_rpm{1.f};

	float _sp_rpm[MAX_MOTORS]{};
	float _meas_rpm[MAX_MOTORS]{};
	float _integral[MAX_MOTORS]{};
	float _prev_err[MAX_MOTORS]{};
	float _last_cmd_norm[MAX_MOTORS]{};
};
