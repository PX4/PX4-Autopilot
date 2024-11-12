/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "RoverAckermannControl.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;

RoverAckermannControl::RoverAckermannControl(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_rover_ackermann_status_pub.advertise();
}

void RoverAckermannControl::updateParams()
{
	ModuleParams::updateParams();

	_pid_throttle.setGains(_param_ra_speed_p.get(), _param_ra_speed_i.get(), 0.f);
	_pid_throttle.setIntegralLimit(1.f);
	_pid_throttle.setOutputLimit(1.f);

	_pid_lat_accel.setGains(_param_ra_lat_accel_p.get(), _param_ra_lat_accel_i.get(), 0.f);
	_pid_lat_accel.setIntegralLimit(1.f);
	_pid_lat_accel.setOutputLimit(1.f);

	// Update slew rates
	if (_param_ra_max_accel.get() > FLT_EPSILON && _param_ra_max_speed.get() > FLT_EPSILON) {
		_forward_speed_setpoint_with_accel_limit.setSlewRate(_param_ra_max_accel.get() / _param_ra_max_speed.get());
	}

	if (_param_ra_max_steering_rate.get() > FLT_EPSILON && _param_ra_max_steer_angle.get() > FLT_EPSILON) {
		_steering_with_rate_limit.setSlewRate((M_DEG_TO_RAD_F * _param_ra_max_steering_rate.get()) /
						      _param_ra_max_steer_angle.get());
	}
}

void RoverAckermannControl::computeMotorCommands(const float vehicle_forward_speed, const float vehicle_yaw,
		const float vehicle_lateral_acceleration)
{
	// Timestamps
	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5_s) * 1e-6f;

	// Update ackermann setpoint
	_rover_ackermann_setpoint_sub.update(&_rover_ackermann_setpoint);

	// Speed control
	float forward_speed_normalized{0.f};

	if (PX4_ISFINITE(_rover_ackermann_setpoint.forward_speed_setpoint)) {
		forward_speed_normalized = calcNormalizedSpeedSetpoint(_rover_ackermann_setpoint.forward_speed_setpoint,
					   vehicle_forward_speed, dt, false);

	} else if (PX4_ISFINITE(_rover_ackermann_setpoint.forward_speed_setpoint_normalized)) { // Use normalized setpoint
		forward_speed_normalized = calcNormalizedSpeedSetpoint(_rover_ackermann_setpoint.forward_speed_setpoint_normalized,
					   vehicle_forward_speed, dt, true);

	}

	// Closed loop lateral acceleration control (overrides steering setpoint)
	if (PX4_ISFINITE(_rover_ackermann_setpoint.lateral_acceleration_setpoint)) {
		float vehicle_forward_speed_temp{0.f};

		if (PX4_ISFINITE(_rover_ackermann_setpoint.forward_speed_setpoint)) { // Use valid measurement if available
			vehicle_forward_speed_temp = vehicle_forward_speed;

		} else if (PX4_ISFINITE(forward_speed_normalized) && _param_ra_max_thr_speed.get() > FLT_EPSILON) {
			vehicle_forward_speed_temp = math::interpolate<float>(forward_speed_normalized,
						     -1.f, 1.f, -_param_ra_max_thr_speed.get(), _param_ra_max_thr_speed.get());
		}

		if (fabsf(vehicle_forward_speed_temp) > FLT_EPSILON) {
			float steering_setpoint = atanf(_param_ra_wheel_base.get() *
							_rover_ackermann_setpoint.lateral_acceleration_setpoint / powf(
								vehicle_forward_speed_temp, 2.f));

			if (sign(vehicle_forward_speed_temp) ==
			    1) { // Only do closed loop control when driving forwards (backwards driving is non-minimum phase and can therefor introduce instability)
				_pid_lat_accel.setSetpoint(_rover_ackermann_setpoint.lateral_acceleration_setpoint);
				steering_setpoint += _pid_lat_accel.update(vehicle_lateral_acceleration, dt);
			}

			_rover_ackermann_setpoint.steering_setpoint = math::constrain(steering_setpoint, -_param_ra_max_steer_angle.get(),
					_param_ra_max_steer_angle.get());

		} else {
			_rover_ackermann_setpoint.steering_setpoint = 0.f;
		}
	}

	// Steering control
	float steering_normalized{0.f};

	if (PX4_ISFINITE(_rover_ackermann_setpoint.steering_setpoint)) {
		steering_normalized = math::interpolate<float>(_rover_ackermann_setpoint.steering_setpoint,
				      -_param_ra_max_steer_angle.get(),
				      _param_ra_max_steer_angle.get(), -1.f, 1.f); // Normalize steering setpoint

	} else { // Use normalized setpoint
		steering_normalized = PX4_ISFINITE(_rover_ackermann_setpoint.steering_setpoint_normalized) ? math::constrain(
					      _rover_ackermann_setpoint.steering_setpoint_normalized, -1.f, 1.f) : 0.f;
	}

	if (_param_ra_max_steering_rate.get() > FLT_EPSILON
	    && _param_ra_max_steer_angle.get() > FLT_EPSILON) { // Apply slew rate
		_steering_with_rate_limit.update(steering_normalized, dt);

	} else {
		_steering_with_rate_limit.setForcedValue(steering_normalized);
	}

	// Publish rover Ackermann status (logging)
	_rover_ackermann_status.timestamp = _timestamp;
	_rover_ackermann_status.measured_forward_speed = vehicle_forward_speed;
	_rover_ackermann_status.steering_setpoint_normalized = steering_normalized;
	_rover_ackermann_status.adjusted_steering_setpoint_normalized = _steering_with_rate_limit.getState();
	_rover_ackermann_status.measured_lateral_acceleration = vehicle_lateral_acceleration;
	_rover_ackermann_status.pid_throttle_integral = _pid_throttle.getIntegral();
	_rover_ackermann_status.pid_lat_accel_integral = _pid_lat_accel.getIntegral();
	_rover_ackermann_status_pub.publish(_rover_ackermann_status);

	// Publish to motor
	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get();
	actuator_motors.control[0] = forward_speed_normalized;
	actuator_motors.timestamp = _timestamp;
	_actuator_motors_pub.publish(actuator_motors);

	// Publish to servo
	actuator_servos_s actuator_servos{};
	actuator_servos.control[0] = _steering_with_rate_limit.getState();
	actuator_servos.timestamp = _timestamp;
	_actuator_servos_pub.publish(actuator_servos);

}

float RoverAckermannControl::calcNormalizedSpeedSetpoint(const float forward_speed_setpoint,
		const float vehicle_forward_speed, const float dt, const bool normalized)
{
	float slew_rate_normalization{1.f};

	if (normalized) { // Slew rate needs to be normalized if the setpoint is normalized
		slew_rate_normalization = _param_ra_max_thr_speed.get() > FLT_EPSILON ? _param_ra_max_thr_speed.get() : 0.f;
	}

	// Apply acceleration and deceleration limit
	if (fabsf(forward_speed_setpoint) >= fabsf(_forward_speed_setpoint_with_accel_limit.getState())) {
		if (_param_ra_max_accel.get() > FLT_EPSILON && slew_rate_normalization > FLT_EPSILON) {
			_forward_speed_setpoint_with_accel_limit.setSlewRate(_param_ra_max_accel.get() / slew_rate_normalization);
			_forward_speed_setpoint_with_accel_limit.update(forward_speed_setpoint, dt);

		} else {
			_forward_speed_setpoint_with_accel_limit.setForcedValue(forward_speed_setpoint);

		}

	} else if (_param_ra_max_decel.get() > FLT_EPSILON && slew_rate_normalization > FLT_EPSILON) {
		_forward_speed_setpoint_with_accel_limit.setSlewRate(_param_ra_max_decel.get() / slew_rate_normalization);
		_forward_speed_setpoint_with_accel_limit.update(forward_speed_setpoint, dt);

	} else {
		_forward_speed_setpoint_with_accel_limit.setForcedValue(forward_speed_setpoint);
	}

	// Calculate normalized forward speed setpoint
	float forward_speed_normalized{0.f};

	if (normalized) {
		forward_speed_normalized = _forward_speed_setpoint_with_accel_limit.getState();

	} else { // Closed loop speed control
		_rover_ackermann_status.adjusted_forward_speed_setpoint = _forward_speed_setpoint_with_accel_limit.getState();

		if (_param_ra_max_thr_speed.get() > FLT_EPSILON) { // Feedforward
			forward_speed_normalized = math::interpolate<float>(_forward_speed_setpoint_with_accel_limit.getState(),
						   -_param_ra_max_thr_speed.get(), _param_ra_max_thr_speed.get(),
						   -1.f, 1.f);
		}

		_pid_throttle.setSetpoint(_forward_speed_setpoint_with_accel_limit.getState());
		forward_speed_normalized += _pid_throttle.update(vehicle_forward_speed, dt);
	}

	return math::constrain(forward_speed_normalized, -1.f, 1.f);

}

void RoverAckermannControl::resetControllers()
{
	_pid_throttle.resetIntegral();
	_pid_lat_accel.resetIntegral();
	_forward_speed_setpoint_with_accel_limit.setForcedValue(0.f);
	_steering_with_rate_limit.setForcedValue(0.f);

}
