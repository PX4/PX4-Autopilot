/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

#include "RoverMecanumControl.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;
using namespace time_literals;

RoverMecanumControl::RoverMecanumControl(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_rover_mecanum_status_pub.advertise();
}

void RoverMecanumControl::updateParams()
{
	ModuleParams::updateParams();

	_pid_yaw_rate.setGains(_param_rm_yaw_rate_p.get(), _param_rm_yaw_rate_i.get(), 0.f);
	_pid_yaw_rate.setIntegralLimit(1.f);
	_pid_yaw_rate.setOutputLimit(1.f);

	_pid_forward_throttle.setGains(_param_rm_p_gain_speed.get(), _param_rm_i_gain_speed.get(), 0.f);
	_pid_forward_throttle.setIntegralLimit(1.f);
	_pid_forward_throttle.setOutputLimit(1.f);

	_pid_lateral_throttle.setGains(_param_rm_p_gain_speed.get(), _param_rm_i_gain_speed.get(), 0.f);
	_pid_lateral_throttle.setIntegralLimit(1.f);
	_pid_lateral_throttle.setOutputLimit(1.f);

	_max_yaw_rate = _param_rm_max_yaw_rate.get() * M_DEG_TO_RAD_F;
	_pid_yaw.setGains(_param_rm_p_gain_yaw.get(), _param_rm_i_gain_yaw.get(), 0.f);
	_pid_yaw.setIntegralLimit(_max_yaw_rate);
	_pid_yaw.setOutputLimit(_max_yaw_rate);
}

void RoverMecanumControl::computeMotorCommands(const float vehicle_yaw, const float vehicle_yaw_rate,
		const float vehicle_forward_speed, const float vehicle_lateral_speed)
{
	// Timestamps
	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	// Update mecanum setpoint
	_rover_mecanum_setpoint_sub.update(&_rover_mecanum_setpoint);

	// Closed loop yaw control
	if (PX4_ISFINITE(_rover_mecanum_setpoint.yaw_setpoint)) {
		_pid_yaw.setSetpoint(
			matrix::wrap_pi(_rover_mecanum_setpoint.yaw_setpoint - vehicle_yaw));  // error as setpoint to take care of wrapping
		_rover_mecanum_setpoint.yaw_rate_setpoint = _pid_yaw.update(0.f, dt);

	} else {
		_pid_yaw.resetIntegral();
	}

	// Yaw rate control
	float speed_diff_normalized{0.f};

	if (PX4_ISFINITE(_rover_mecanum_setpoint.yaw_rate_setpoint)) { 	// Closed loop yaw rate control
		if (_param_rm_max_thr_yaw_r.get() > FLT_EPSILON) { // Feedforward
			const float speed_diff = _rover_mecanum_setpoint.yaw_rate_setpoint * _param_rm_wheel_track.get();
			speed_diff_normalized = math::interpolate<float>(speed_diff, -_param_rm_max_thr_yaw_r.get(),
						_param_rm_max_thr_yaw_r.get(), -1.f, 1.f);
		}

		_pid_yaw_rate.setSetpoint(_rover_mecanum_setpoint.yaw_rate_setpoint);
		speed_diff_normalized = math::constrain(speed_diff_normalized +
							_pid_yaw_rate.update(vehicle_yaw_rate, dt),
							-1.f, 1.f); // Feedback

	} else { // Use normalized setpoint
		speed_diff_normalized = PX4_ISFINITE(_rover_mecanum_setpoint.speed_diff_setpoint_normalized) ? math::constrain(
						_rover_mecanum_setpoint.speed_diff_setpoint_normalized, -1.f, 1.f) : 0.f;
	}

	// Speed control
	float forward_throttle{0.f};
	float lateral_throttle{0.f};

	if (PX4_ISFINITE(_rover_mecanum_setpoint.forward_speed_setpoint)
	    && PX4_ISFINITE(_rover_mecanum_setpoint.lateral_speed_setpoint)) { // Closed loop speed control
		// Closed loop forward speed control
		if (_param_rm_max_thr_spd.get() > FLT_EPSILON) { // Feedforward
			forward_throttle = math::interpolate<float>(_rover_mecanum_setpoint.forward_speed_setpoint,
					   -_param_rm_max_thr_spd.get(), _param_rm_max_thr_spd.get(), -1.f, 1.f);
		}

		_pid_forward_throttle.setSetpoint(_rover_mecanum_setpoint.forward_speed_setpoint);
		forward_throttle += _pid_forward_throttle.update(vehicle_forward_speed, dt);

		// Closed loop lateral speed control
		if (_param_rm_max_thr_spd.get() > FLT_EPSILON) { // Feedforward
			lateral_throttle = math::interpolate<float>(_rover_mecanum_setpoint.lateral_speed_setpoint,
					   -_param_rm_max_thr_spd.get(), _param_rm_max_thr_spd.get(), -1.f, 1.f);
		}

		_pid_lateral_throttle.setSetpoint(_rover_mecanum_setpoint.lateral_speed_setpoint);
		lateral_throttle += _pid_lateral_throttle.update(vehicle_lateral_speed, dt);

	} else { // Use normalized setpoint
		forward_throttle = PX4_ISFINITE(_rover_mecanum_setpoint.forward_speed_setpoint_normalized) ? math::constrain(
					   _rover_mecanum_setpoint.forward_speed_setpoint_normalized, -1.f, 1.f) : 0.f;
		lateral_throttle = PX4_ISFINITE(_rover_mecanum_setpoint.lateral_speed_setpoint_normalized) ? math::constrain(
					   _rover_mecanum_setpoint.lateral_speed_setpoint_normalized, -1.f, 1.f) : 0.f;
	}

	// Publish rover mecanum status (logging)
	rover_mecanum_status_s rover_mecanum_status{};
	rover_mecanum_status.timestamp = _timestamp;
	rover_mecanum_status.measured_forward_speed = vehicle_forward_speed;
	rover_mecanum_status.measured_lateral_speed = vehicle_lateral_speed;
	rover_mecanum_status.adjusted_yaw_rate_setpoint = _rover_mecanum_setpoint.yaw_rate_setpoint;
	rover_mecanum_status.measured_yaw_rate = vehicle_yaw_rate;
	rover_mecanum_status.measured_yaw = vehicle_yaw;
	rover_mecanum_status.pid_yaw_rate_integral = _pid_yaw_rate.getIntegral();
	rover_mecanum_status.pid_yaw_integral = _pid_yaw.getIntegral();
	rover_mecanum_status.pid_forward_throttle_integral = _pid_forward_throttle.getIntegral();
	rover_mecanum_status.pid_lateral_throttle_integral = _pid_lateral_throttle.getIntegral();
	_rover_mecanum_status_pub.publish(rover_mecanum_status);

	// Publish to motors
	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get();
	computeInverseKinematics(forward_throttle, lateral_throttle,
				 speed_diff_normalized).copyTo(actuator_motors.control);
	actuator_motors.timestamp = _timestamp;
	_actuator_motors_pub.publish(actuator_motors);

}

matrix::Vector4f RoverMecanumControl::computeInverseKinematics(float forward_throttle, float lateral_throttle,
		float speed_diff)
{
	// Prioritize ratio between forward and lateral speed over either magnitude
	float combined_speed =  fabsf(forward_throttle) + fabsf(lateral_throttle);

	if (combined_speed > 1.f) {
		forward_throttle /= combined_speed;
		lateral_throttle /= combined_speed;
		combined_speed = 1.f;
	}

	// Prioritize yaw rate over forward and lateral speed
	const float total_speed = combined_speed + fabsf(speed_diff);

	if (total_speed > 1.f) {
		const float excess_velocity = fabsf(total_speed - 1.f);
		const float forward_throttle_temp = forward_throttle - sign(forward_throttle) * 0.5f * excess_velocity;
		const float lateral_throttle_temp = lateral_throttle - sign(lateral_throttle) * 0.5f * excess_velocity;

		if (sign(forward_throttle_temp) == sign(forward_throttle) && sign(lateral_throttle) == sign(lateral_throttle_temp)) {
			forward_throttle = forward_throttle_temp;
			lateral_throttle = lateral_throttle_temp;

		} else {
			forward_throttle = lateral_throttle = 0.f;
		}
	}

	// Calculate motor commands
	const float input_data[3] = {forward_throttle, lateral_throttle, speed_diff};
	const Matrix<float, 3, 1> input(input_data);
	const float m_data[12] = {1.f, -1.f, -1.f, 1.f, 1.f, 1.f, 1.f, 1.f, -1.f, 1.f, -1.f, 1.f};
	const Matrix<float, 4, 3> m(m_data);
	const Vector4f motor_commands = m * input;

	return motor_commands;
}

void RoverMecanumControl::resetControllers()
{
	_pid_forward_throttle.resetIntegral();
	_pid_lateral_throttle.resetIntegral();
	_pid_yaw_rate.resetIntegral();
	_pid_yaw.resetIntegral();
}
