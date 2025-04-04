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

#include "AckermannAttControl.hpp"

using namespace time_literals;

AckermannAttControl::AckermannAttControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_rate_setpoint_pub.advertise();
	_rover_throttle_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
	_rover_attitude_status_pub.advertise();
	updateParams();
}

void AckermannAttControl::updateParams()
{
	ModuleParams::updateParams();

	if (_param_ro_yaw_rate_limit.get() > FLT_EPSILON) {
		_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;
	}

	_pid_yaw.setGains(_param_ro_yaw_p.get(), 0.f, 0.f);
	_pid_yaw.setIntegralLimit(0.f);
	_pid_yaw.setOutputLimit(_max_yaw_rate);
	_adjusted_yaw_setpoint.setSlewRate(_max_yaw_rate);
}

void AckermannAttControl::updateAttControl()
{
	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		matrix::Quatf vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(vehicle_attitude_quaternion).psi();
	}

	if (_vehicle_control_mode.flag_control_attitude_enabled && _vehicle_control_mode.flag_armed && runSanityChecks()) {
		// Estimate forward speed based on throttle
		if (_actuator_motors_sub.updated()) {
			actuator_motors_s actuator_motors;
			_actuator_motors_sub.copy(&actuator_motors);
			_estimated_speed_body_x = math::interpolate<float> (actuator_motors.control[0], -1.f, 1.f,
						  -_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get());
		}

		if (_vehicle_control_mode.flag_control_manual_enabled) {
			generateAttitudeAndThrottleSetpoint();
		}

		generateRateSetpoint();

	} else { // Reset pid and slew rate when attitude control is not active
		_pid_yaw.resetIntegral();
		_adjusted_yaw_setpoint.setForcedValue(0.f);
	}

	// Publish attitude controller status (logging only)
	rover_attitude_status_s rover_attitude_status;
	rover_attitude_status.timestamp = _timestamp;
	rover_attitude_status.measured_yaw = _vehicle_yaw;
	rover_attitude_status.adjusted_yaw_setpoint = matrix::wrap_pi(_adjusted_yaw_setpoint.getState());
	_rover_attitude_status_pub.publish(rover_attitude_status);

}

void AckermannAttControl::generateAttitudeAndThrottleSetpoint()
{
	const bool stab_mode_enabled = _vehicle_control_mode.flag_control_manual_enabled
				       && !_vehicle_control_mode.flag_control_position_enabled && _vehicle_control_mode.flag_control_attitude_enabled;

	if (stab_mode_enabled && _manual_control_setpoint_sub.updated()) { // Stab Mode
		manual_control_setpoint_s manual_control_setpoint{};

		if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {

			rover_throttle_setpoint_s rover_throttle_setpoint{};
			rover_throttle_setpoint.timestamp = _timestamp;
			rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.throttle;
			rover_throttle_setpoint.throttle_body_y = 0.f;
			_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);

			const float yaw_delta = math::interpolate<float>(math::deadzone(manual_control_setpoint.roll,
						_param_ro_yaw_stick_dz.get()), -1.f, 1.f, -_max_yaw_rate / _param_ro_yaw_p.get(),
						_max_yaw_rate / _param_ro_yaw_p.get());

			if (fabsf(yaw_delta) > FLT_EPSILON
			    || fabsf(rover_throttle_setpoint.throttle_body_x) < FLT_EPSILON) { // Closed loop yaw rate control
				_stab_yaw_ctl = false;
				const float yaw_setpoint = matrix::wrap_pi(_vehicle_yaw + matrix::sign(manual_control_setpoint.throttle) * yaw_delta);
				rover_attitude_setpoint_s rover_attitude_setpoint{};
				rover_attitude_setpoint.timestamp = _timestamp;
				rover_attitude_setpoint.yaw_setpoint = yaw_setpoint;
				_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

			} else { // Closed loop yaw control if the yaw rate input is zero (keep current yaw)
				if (!_stab_yaw_ctl) {
					_stab_yaw_setpoint = _vehicle_yaw;
					_stab_yaw_ctl = true;
				}

				rover_attitude_setpoint_s rover_attitude_setpoint{};
				rover_attitude_setpoint.timestamp = _timestamp;
				rover_attitude_setpoint.yaw_setpoint = _stab_yaw_setpoint;
				_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
			}


		}

	}
}

void AckermannAttControl::generateRateSetpoint()
{
	if (_rover_attitude_setpoint_sub.updated()) {
		_rover_attitude_setpoint_sub.copy(&_rover_attitude_setpoint);
	}

	if (_rover_rate_setpoint_sub.updated()) {
		_rover_rate_setpoint_sub.copy(&_rover_rate_setpoint);
	}

	// Check if a new rate setpoint was already published from somewhere else
	if (_rover_rate_setpoint.timestamp > _last_rate_setpoint_update
	    && _rover_rate_setpoint.timestamp > _rover_attitude_setpoint.timestamp) {
		return;
	}

	// Calculate yaw rate limit for slew rate
	float max_possible_yaw_rate = fabsf(_estimated_speed_body_x) * tanf(_param_ra_max_str_ang.get()) /
				      _param_ra_wheel_base.get(); // Maximum possible yaw rate at current velocity
	float yaw_slew_rate = math::min(max_possible_yaw_rate, _max_yaw_rate);

	float yaw_rate_setpoint = RoverControl::attitudeControl(_adjusted_yaw_setpoint, _pid_yaw, yaw_slew_rate,
				  _vehicle_yaw, _rover_attitude_setpoint.yaw_setpoint, _dt);

	_last_rate_setpoint_update = _timestamp;
	rover_rate_setpoint_s rover_rate_setpoint{};
	rover_rate_setpoint.timestamp = _timestamp;
	rover_rate_setpoint.yaw_rate_setpoint = math::constrain(yaw_rate_setpoint, -_max_yaw_rate, _max_yaw_rate);
	_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
}

bool AckermannAttControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_max_thr_speed.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ra_wheel_base.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ra_max_str_ang.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ro_yaw_rate_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	return ret;
}
