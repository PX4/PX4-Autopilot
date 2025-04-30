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

#include "AckermannRateControl.hpp"

using namespace time_literals;

AckermannRateControl::AckermannRateControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_rate_setpoint_pub.advertise();
	_rover_throttle_setpoint_pub.advertise();
	_rover_steering_setpoint_pub.advertise();
	_rover_rate_status_pub.advertise();
	updateParams();
}

void AckermannRateControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;
	_pid_yaw_rate.setGains(_param_ro_yaw_rate_p.get(), _param_ro_yaw_rate_i.get(), 0.f);
	_pid_yaw_rate.setIntegralLimit(1.f);
	_pid_yaw_rate.setOutputLimit(1.f);
	_yaw_rate_setpoint.setSlewRate(_param_ro_yaw_accel_limit.get() * M_DEG_TO_RAD_F);
}

void AckermannRateControl::updateRateControl()
{
	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	if (_vehicle_angular_velocity_sub.updated()) {
		vehicle_angular_velocity_s vehicle_angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
		_vehicle_yaw_rate = fabsf(vehicle_angular_velocity.xyz[2]) > _param_ro_yaw_rate_th.get() * M_DEG_TO_RAD_F ?
				    vehicle_angular_velocity.xyz[2] : 0.f;
	}

	// Estimate forward speed based on throttle
	if (_actuator_motors_sub.updated()) {
		actuator_motors_s actuator_motors;
		_actuator_motors_sub.copy(&actuator_motors);
		_estimated_speed_body_x = math::interpolate<float>(actuator_motors.control[0], -1.f, 1.f,
					  -_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get());
		_estimated_speed_body_x = fabsf(_estimated_speed_body_x) >  _param_ro_speed_th.get() ? _estimated_speed_body_x : 0.f;
	}

	generateSteeringSetpoint();

	// Publish rate controller status (logging only)
	rover_rate_status_s rover_rate_status;
	rover_rate_status.timestamp = _timestamp;
	rover_rate_status.measured_yaw_rate = _vehicle_yaw_rate;
	rover_rate_status.adjusted_yaw_rate_setpoint = _yaw_rate_setpoint.getState();
	rover_rate_status.pid_yaw_rate_integral = _pid_yaw_rate.getIntegral();
	_rover_rate_status_pub.publish(rover_rate_status);

}

void AckermannRateControl::acroMode()
{
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);
	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = hrt_absolute_time();
	rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.throttle;
	rover_throttle_setpoint.throttle_body_y = 0.f;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
	rover_rate_setpoint_s rover_rate_setpoint{};
	rover_rate_setpoint.timestamp = hrt_absolute_time();
	rover_rate_setpoint.yaw_rate_setpoint = matrix::sign(manual_control_setpoint.throttle) * math::interpolate<float>
						(manual_control_setpoint.roll, -1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);
	_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
}

void AckermannRateControl::generateSteeringSetpoint()
{
	if (_rover_rate_setpoint_sub.updated()) {
		_rover_rate_setpoint_sub.copy(&_rover_rate_setpoint);

	}

	float steering_setpoint{0.f};

	if (fabsf(_estimated_speed_body_x) > FLT_EPSILON) {
		// Set up feasible yaw rate setpoint
		float max_possible_yaw_rate = fabsf(_estimated_speed_body_x) * tanf(_param_ra_max_str_ang.get()) /
					      _param_ra_wheel_base.get(); // Maximum possible yaw rate at current velocity
		float yaw_rate_limit = math::min(max_possible_yaw_rate, _max_yaw_rate);
		float constrained_yaw_rate = math::constrain(_rover_rate_setpoint.yaw_rate_setpoint, -yaw_rate_limit, yaw_rate_limit);

		if (_param_ro_yaw_accel_limit.get() > FLT_EPSILON) { // Apply slew rate if configured
			if (fabsf(_yaw_rate_setpoint.getState() - _vehicle_yaw_rate) > fabsf(constrained_yaw_rate -
					_vehicle_yaw_rate)) {
				_yaw_rate_setpoint.setForcedValue(_vehicle_yaw_rate);
			}

			_yaw_rate_setpoint.update(constrained_yaw_rate, _dt);

		} else {
			_yaw_rate_setpoint.setForcedValue(constrained_yaw_rate);
		}

		// Feed forward
		steering_setpoint = atanf(_yaw_rate_setpoint.getState() * _param_ra_wheel_base.get() / _estimated_speed_body_x);

		// Feedback (Only when driving forwards because backwards driving is NMP and can introduce instability)
		if (_estimated_speed_body_x > FLT_EPSILON) {
			_pid_yaw_rate.setSetpoint(_yaw_rate_setpoint.getState());
			steering_setpoint += _pid_yaw_rate.update(_vehicle_yaw_rate, _dt);
		}

	} else {
		_pid_yaw_rate.resetIntegral();
	}

	rover_steering_setpoint_s rover_steering_setpoint{};
	rover_steering_setpoint.timestamp = _timestamp;
	rover_steering_setpoint.normalized_steering_angle = math::interpolate<float>(steering_setpoint,
			-_param_ra_max_str_ang.get(), _param_ra_max_str_ang.get(), -1.f, 1.f); // Normalize steering setpoint
	_rover_steering_setpoint_pub.publish(rover_steering_setpoint);
}

bool AckermannRateControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_max_thr_speed.get() < FLT_EPSILON) {
		ret = false;
		events::send<float>(events::ID("ackermann_rate_control_conf_invalid_max_thr_speed"), events::Log::Error,
				    "Invalid configuration of necessary parameter RO_MAX_THR_SPEED", _param_ro_max_thr_speed.get());

	}

	if (_param_ra_wheel_base.get() < FLT_EPSILON) {
		ret = false;
		events::send<float>(events::ID("ackermann_rate_control_conf_invalid_wheel_base"), events::Log::Error,
				    "Invalid configuration of necessary parameter RA_WHEEL_BASE", _param_ra_wheel_base.get());

	}

	if (_param_ra_max_str_ang.get() < FLT_EPSILON) {
		ret = false;
		events::send<float>(events::ID("ackermann_rate_control_conf_invalid_max_str_ang"), events::Log::Error,
				    "Invalid configuration of necessary parameter RA_MAX_STR_ANG", _param_ra_max_str_ang.get());

	}

	if (_param_ro_yaw_rate_limit.get() < FLT_EPSILON) {
		ret = false;
		events::send<float>(events::ID("ackermann_rate_control_conf_invalid_yaw_rate_lim"), events::Log::Error,
				    "Invalid configuration of necessary parameter RO_YAW_RATE_LIM", _param_ro_yaw_rate_limit.get());

	}

	return ret;
}
