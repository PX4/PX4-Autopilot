/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "DifferentialAttControl.hpp"

using namespace time_literals;

DifferentialAttControl::DifferentialAttControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_rate_setpoint_pub.advertise();
	_rover_attitude_status_pub.advertise();
	updateParams();
}

void DifferentialAttControl::updateParams()
{
	ModuleParams::updateParams();

	if (_param_ro_yaw_rate_limit.get() > FLT_EPSILON) {
		_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;
	}

	// Set up PID controller
	_pid_yaw.setGains(_param_ro_yaw_p.get(), 0.f, 0.f);
	_pid_yaw.setIntegralLimit(_max_yaw_rate);
	_pid_yaw.setOutputLimit(_max_yaw_rate);

	// Set up slew rate
	_adjusted_yaw_setpoint.setSlewRate(_max_yaw_rate);
}

void DifferentialAttControl::updateAttControl()
{
	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 10_ms) * 1e-6f;

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		matrix::Quatf vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(vehicle_attitude_quaternion).psi();
	}

	if (_rover_attitude_setpoint_sub.updated()) {
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		_rover_attitude_setpoint_sub.copy(&rover_attitude_setpoint);
		_yaw_setpoint = rover_attitude_setpoint.yaw_setpoint;
	}

	if (PX4_ISFINITE(_yaw_setpoint)) {
		const float yaw_rate_setpoint = RoverControl::attitudeControl(_adjusted_yaw_setpoint, _pid_yaw, _max_yaw_rate,
						_vehicle_yaw, _yaw_setpoint, dt);
		rover_rate_setpoint_s rover_rate_setpoint{};
		rover_rate_setpoint.timestamp = _timestamp;
		rover_rate_setpoint.yaw_rate_setpoint = math::constrain(yaw_rate_setpoint, -_max_yaw_rate, _max_yaw_rate);
		_rover_rate_setpoint_pub.publish(rover_rate_setpoint);

	}

	// Publish attitude controller status (logging only)
	rover_attitude_status_s rover_attitude_status;
	rover_attitude_status.timestamp = _timestamp;
	rover_attitude_status.measured_yaw = _vehicle_yaw;
	rover_attitude_status.adjusted_yaw_setpoint = matrix::wrap_pi(_adjusted_yaw_setpoint.getState());
	_rover_attitude_status_pub.publish(rover_attitude_status);

}

bool DifferentialAttControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_yaw_rate_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ro_yaw_p.get() < FLT_EPSILON) {
		ret = false;
		events::send<float>(events::ID("differential_att_control_conf_invalid_yaw_p"), events::Log::Error,
				    "Invalid configuration of necessary parameter RO_YAW_P", _param_ro_yaw_p.get());
	}

	return ret;
}
