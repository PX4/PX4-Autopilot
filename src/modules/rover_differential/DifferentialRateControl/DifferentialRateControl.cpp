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

#include "DifferentialRateControl.hpp"

using namespace time_literals;

DifferentialRateControl::DifferentialRateControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_steering_setpoint_pub.advertise();
	_rover_rate_status_pub.advertise();
	updateParams();
}

void DifferentialRateControl::updateParams()
{
	ModuleParams::updateParams();

	// Set up PID controller
	_pid_yaw_rate.setGains(_param_ro_yaw_rate_p.get(), _param_ro_yaw_rate_i.get(), 0.f);
	_pid_yaw_rate.setIntegralLimit(1.f);
	_pid_yaw_rate.setOutputLimit(1.f);

	// Set up slew rate
	_adjusted_yaw_rate_setpoint.setSlewRate(_param_ro_yaw_accel_limit.get() * M_DEG_TO_RAD_F);
}

void DifferentialRateControl::updateRateControl()
{
	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 10_ms) * 1e-6f;

	if (_vehicle_angular_velocity_sub.updated()) {
		vehicle_angular_velocity_s vehicle_angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
		_vehicle_yaw_rate = fabsf(vehicle_angular_velocity.xyz[2]) > _param_ro_yaw_rate_th.get() * M_DEG_TO_RAD_F ?
				    vehicle_angular_velocity.xyz[2] : 0.f;
	}

	if (_rover_rate_setpoint_sub.updated()) {
		rover_rate_setpoint_s rover_rate_setpoint{};
		_rover_rate_setpoint_sub.copy(&rover_rate_setpoint);
		_yaw_rate_setpoint = rover_rate_setpoint.yaw_rate_setpoint;
	}

	if (PX4_ISFINITE(_yaw_rate_setpoint)) {
		const float yaw_rate_setpoint = fabsf(_yaw_rate_setpoint) > _param_ro_yaw_rate_th.get() * M_DEG_TO_RAD_F ?
						_yaw_rate_setpoint : 0.f;
		const float speed_diff_normalized = RoverControl::rateControl(_adjusted_yaw_rate_setpoint, _pid_yaw_rate,
						    yaw_rate_setpoint, _vehicle_yaw_rate, _param_ro_max_thr_speed.get(), _param_ro_yaw_rate_corr.get(),
						    _param_ro_yaw_accel_limit.get() * M_DEG_TO_RAD_F,
						    _param_ro_yaw_decel_limit.get() * M_DEG_TO_RAD_F, _param_rd_wheel_track.get(), dt);
		rover_steering_setpoint_s rover_steering_setpoint{};
		rover_steering_setpoint.timestamp = _timestamp;
		rover_steering_setpoint.normalized_steering_setpoint = speed_diff_normalized;
		_rover_steering_setpoint_pub.publish(rover_steering_setpoint);

	} else {
		_pid_yaw_rate.resetIntegral();
	}

	// Publish rate controller status (logging only)
	rover_rate_status_s rover_rate_status;
	rover_rate_status.timestamp = _timestamp;
	rover_rate_status.measured_yaw_rate = _vehicle_yaw_rate;
	rover_rate_status.adjusted_yaw_rate_setpoint = _adjusted_yaw_rate_setpoint.getState();
	rover_rate_status.pid_yaw_rate_integral = _pid_yaw_rate.getIntegral();
	_rover_rate_status_pub.publish(rover_rate_status);

}

bool DifferentialRateControl::runSanityChecks()
{
	bool ret = true;

	if ((_param_rd_wheel_track.get() < FLT_EPSILON || _param_ro_max_thr_speed.get() < FLT_EPSILON)
	    && _param_ro_yaw_rate_p.get() < FLT_EPSILON) {
		ret = false;
		events::send<float, float, float>(events::ID("differential_rate_control_conf_invalid_rate_control"), events::Log::Error,
						  "Invalid configuration for rate control: Neither feed forward nor feedback is setup", _param_rd_wheel_track.get(),
						  _param_ro_max_thr_speed.get(), _param_ro_yaw_rate_p.get());
	}

	return ret;
}
