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

#include "AckermannActControl.hpp"

using namespace time_literals;

AckermannActControl::AckermannActControl(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
}

void AckermannActControl::updateParams()
{
	ModuleParams::updateParams();

	if (_param_ra_str_rate_limit.get() > FLT_EPSILON && _param_ra_max_str_ang.get() > FLT_EPSILON) {
		_servo_setpoint.setSlewRate((M_DEG_TO_RAD_F * _param_ra_str_rate_limit.get()) / _param_ra_max_str_ang.get());
	}

	if (_param_ro_accel_limit.get() > FLT_EPSILON && _param_ro_max_thr_speed.get() > FLT_EPSILON) {
		_motor_setpoint.setSlewRate(_param_ro_accel_limit.get() / _param_ro_max_thr_speed.get());
	}
}

void AckermannActControl::updateActControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	// Motor control
	rover_throttle_setpoint_s rover_throttle_setpoint{};
	_rover_throttle_setpoint_sub.copy(&rover_throttle_setpoint);
	actuator_motors_s actuator_motors_sub{};
	_actuator_motors_sub.copy(&actuator_motors_sub);
	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get();
	actuator_motors.control[0] = RoverControl::throttleControl(_motor_setpoint,
				     rover_throttle_setpoint.throttle_body_x, actuator_motors_sub.control[0], _param_ro_accel_limit.get(),
				     _param_ro_decel_limit.get(), _param_ro_max_thr_speed.get(), _dt);
	actuator_motors.timestamp = _timestamp;
	_actuator_motors_pub.publish(actuator_motors);

	// Servo control
	rover_steering_setpoint_s rover_steering_setpoint{};
	_rover_steering_setpoint_sub.copy(&rover_steering_setpoint);
	actuator_servos_s actuator_servos_sub{};
	_actuator_servos_sub.copy(&actuator_servos_sub);

	if (_param_ra_str_rate_limit.get() > FLT_EPSILON
	    && _param_ra_max_str_ang.get() > FLT_EPSILON) { // Apply slew rate if configured
		if (fabsf(_servo_setpoint.getState() - actuator_servos_sub.control[0]) > fabsf(
			    rover_steering_setpoint.normalized_steering_angle -
			    actuator_servos_sub.control[0])) {
			_servo_setpoint.setForcedValue(actuator_servos_sub.control[0]);
		}

		_servo_setpoint.update(rover_steering_setpoint.normalized_steering_angle, _dt);

	} else {
		_servo_setpoint.setForcedValue(rover_steering_setpoint.normalized_steering_angle);
	}

	actuator_servos_s actuator_servos{};
	actuator_servos.control[0] = _servo_setpoint.getState();
	actuator_servos.timestamp = _timestamp;
	_actuator_servos_pub.publish(actuator_servos);

}

void AckermannActControl::manualMode()
{
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);
	rover_steering_setpoint_s rover_steering_setpoint{};
	rover_steering_setpoint.timestamp = hrt_absolute_time();
	rover_steering_setpoint.normalized_steering_angle = manual_control_setpoint.roll;
	_rover_steering_setpoint_pub.publish(rover_steering_setpoint);
	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = hrt_absolute_time();
	rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.throttle;
	rover_throttle_setpoint.throttle_body_y = 0.f;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
}
