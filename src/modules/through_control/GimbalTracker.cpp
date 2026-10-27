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

#include "GimbalTracker.hpp"

#include <mathlib/mathlib.h>

using namespace matrix;
using namespace time_literals;

namespace through
{

float GimbalTracker::bodyPitchDown(const Quatf &q_nb, const Vector3f &dir_ned)
{
	const Vector3f d_b = q_nb.rotateVectorInverse(dir_ned);
	return atan2f(d_b(2), d_b(0));
}

float GimbalTracker::pitchCommand(float pitch_meas, float pitch_des, float gain)
{
	if (!PX4_ISFINITE(pitch_meas)) {
		pitch_meas = pitch_des;
	}

	const float cmd = pitch_meas + math::constrain(gain, 0.f, 1.f) * (pitch_des - pitch_meas);
	return math::constrain(cmd, PITCH_DOWN_MIN, PITCH_DOWN_MAX);
}

void GimbalTracker::updateStatus()
{
	gimbal_manager_status_s status;

	if (_gimbal_manager_status_sub.update(&status)) {
		_in_control = (status.primary_control_sysid == _sysid) && (status.primary_control_compid == _compid);
	}
}

void GimbalTracker::acquire(hrt_abstime now)
{
	updateStatus();

	// also re-acquire if the GCS took the control while the mode runs (VERIFICATION.md V6)
	if (!_in_control && (!_requested || now > _last_request_time + 1_s)) {
		publishConfigure(_sysid, _compid);
		_requested = true;
		_last_request_time = now;
	}
}

void GimbalTracker::release()
{
	updateStatus();

	if (_requested) {
		// the gimbal manager keeps the last setpoint after the release: go back to neutral first
		// (handled before the configure command, gimbal/input_mavlink.cpp)
		if (_in_control) {
			commandPitch(0.f);
		}

		publishConfigure(-3.f, -3.f); // remove control if we have it
		_requested = false;
		_in_control = false;
		_last_command = NAN;
	}
}

void GimbalTracker::publishConfigure(float sysid, float compid)
{
	vehicle_command_s command{};
	command.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
	command.param1 = sysid;   // primary control sysid
	command.param2 = compid;  // primary control compid
	command.param3 = -1.f;    // secondary control: leave unchanged
	command.param4 = -1.f;
	command.source_system = _sysid;
	command.source_component = _compid;
	command.target_system = _sysid;
	command.target_component = _compid;
	command.from_external = false;
	command.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(command);
}

void GimbalTracker::commandPitch(float pitch_down)
{
	gimbal_manager_set_attitude_s setpoint{};
	setpoint.origin_sysid = _sysid;
	setpoint.origin_compid = _compid;
	setpoint.flags = 0; // no lock: angles relative to the vehicle, yaw follows
	Quatf(Eulerf(0.f, -pitch_down, 0.f)).copyTo(setpoint.q);
	// NaN rates: a zero rate makes the simulated gimbal hold its last angle (VERIFICATION.md V6)
	setpoint.angular_velocity_x = NAN;
	setpoint.angular_velocity_y = NAN;
	setpoint.angular_velocity_z = NAN;
	setpoint.timestamp = hrt_absolute_time();
	_gimbal_manager_set_attitude_pub.publish(setpoint);

	_last_command = pitch_down;
}

} // namespace through
