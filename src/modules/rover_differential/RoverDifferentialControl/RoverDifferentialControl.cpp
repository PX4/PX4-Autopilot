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

#include "RoverDifferentialControl.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;
using namespace time_literals;

RoverDifferentialControl::RoverDifferentialControl(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_rover_differential_status_pub.advertise();
	pid_init(&_pid_yaw_rate, PID_MODE_DERIVATIV_NONE, 0.001f);
	pid_init(&_pid_throttle, PID_MODE_DERIVATIV_NONE, 0.001f);
	pid_init(&_pid_yaw, PID_MODE_DERIVATIV_NONE, 0.001f);
}

void RoverDifferentialControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_rd_max_yaw_rate.get() * M_DEG_TO_RAD_F;
	pid_set_parameters(&_pid_yaw_rate,
			   _param_rd_yaw_rate_p.get(), // Proportional gain
			   _param_rd_yaw_rate_i.get(), // Integral gain
			   0.f, // Derivative gain
			   1.f, // Integral limit
			   1.f); // Output limit
	pid_set_parameters(&_pid_throttle,
			   _param_rd_p_gain_speed.get(), // Proportional gain
			   _param_rd_i_gain_speed.get(), // Integral gain
			   0.f, // Derivative gain
			   1.f, // Integral limit
			   1.f); // Output limit
	pid_set_parameters(&_pid_yaw,
			   _param_rd_p_gain_yaw.get(),  // Proportional gain
			   _param_rd_i_gain_yaw.get(),  // Integral gain
			   0.f,  // Derivative gain
			   _max_yaw_rate,  // Integral limit
			   _max_yaw_rate);  // Output limit
}

void RoverDifferentialControl::computeMotorCommands(const float vehicle_yaw, const float vehicle_yaw_rate,
		const float vehicle_forward_speed)
{
	// Timestamps
	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	// Update differential setpoint
	_rover_differential_setpoint_sub.update(&_rover_differential_setpoint);

	// Closed loop yaw control (Overrides yaw rate setpoint)
	if (PX4_ISFINITE(_rover_differential_setpoint.yaw_setpoint)) {
		float heading_error = matrix::wrap_pi(_rover_differential_setpoint.yaw_setpoint - vehicle_yaw);
		_rover_differential_setpoint.yaw_rate_setpoint = pid_calculate(&_pid_yaw, heading_error, 0, 0, dt);
	}

	// Yaw rate control
	float speed_diff_normalized{0.f};

	if (PX4_ISFINITE(_rover_differential_setpoint.yaw_rate_setpoint)) { // Closed loop yaw rate control
		if (_param_rd_wheel_track.get() > FLT_EPSILON && _param_rd_max_thr_yaw_r.get() > FLT_EPSILON) { // Feedforward
			const float speed_diff = _rover_differential_setpoint.yaw_rate_setpoint * _param_rd_wheel_track.get() /
						 2.f;
			speed_diff_normalized = math::interpolate<float>(speed_diff, -_param_rd_max_thr_yaw_r.get(),
						_param_rd_max_thr_yaw_r.get(), -1.f, 1.f);
		}

		speed_diff_normalized = math::constrain(speed_diff_normalized +
							pid_calculate(&_pid_yaw_rate, _rover_differential_setpoint.yaw_rate_setpoint, vehicle_yaw_rate, 0, dt),
							-1.f, 1.f); // Feedback

	} else { // Use normalized setpoint
		speed_diff_normalized = PX4_ISFINITE(_rover_differential_setpoint.yaw_rate_setpoint_normalized) ?
					math::constrain(_rover_differential_setpoint.yaw_rate_setpoint_normalized, -1.f, 1.f) : 0.f;
	}

	// Speed control
	float forward_speed_normalized{0.f};

	if (PX4_ISFINITE(_rover_differential_setpoint.forward_speed_setpoint)) { // Closed loop speed control
		if (_param_rd_max_thr_spd.get() > FLT_EPSILON) { // Feedforward
			forward_speed_normalized = math::interpolate<float>(_rover_differential_setpoint.forward_speed_setpoint,
						   -_param_rd_max_thr_spd.get(), _param_rd_max_thr_spd.get(),
						   -1.f, 1.f);
		}

		forward_speed_normalized = math::constrain(forward_speed_normalized + pid_calculate(&_pid_throttle,
					   _rover_differential_setpoint.forward_speed_setpoint,
					   vehicle_forward_speed, 0,
					   dt), -1.f, 1.f); // Feedback

	} else { // Use normalized setpoint
		forward_speed_normalized = PX4_ISFINITE(_rover_differential_setpoint.forward_speed_setpoint_normalized) ?
					   math::constrain(_rover_differential_setpoint.forward_speed_setpoint_normalized, -1.f, 1.f) : 0.f;
	}

	// Publish rover differential status (logging)
	rover_differential_status_s rover_differential_status{};
	rover_differential_status.timestamp = _timestamp;
	rover_differential_status.actual_speed = vehicle_forward_speed;
	rover_differential_status.actual_yaw = vehicle_yaw;
	rover_differential_status.desired_yaw_rate = _rover_differential_setpoint.yaw_rate_setpoint;
	rover_differential_status.actual_yaw_rate = vehicle_yaw_rate;
	rover_differential_status.forward_speed_normalized = forward_speed_normalized;
	rover_differential_status.speed_diff_normalized = speed_diff_normalized;
	rover_differential_status.pid_yaw_rate_integral = _pid_yaw_rate.integral;
	rover_differential_status.pid_throttle_integral = _pid_throttle.integral;
	rover_differential_status.pid_yaw_integral = _pid_yaw.integral;
	_rover_differential_status_pub.publish(rover_differential_status);

	// Publish to motors
	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get();
	computeInverseKinematics(forward_speed_normalized, speed_diff_normalized).copyTo(actuator_motors.control);
	actuator_motors.timestamp = _timestamp;
	_actuator_motors_pub.publish(actuator_motors);

}

matrix::Vector2f RoverDifferentialControl::computeInverseKinematics(float forward_speed_normalized,
		const float speed_diff_normalized)
{
	float max_motor_command = fabsf(forward_speed_normalized) + fabsf(speed_diff_normalized);

	if (max_motor_command > 1.0f) { // Prioritize yaw rate if a normalized motor command exceeds limit of 1
		float excess = fabsf(max_motor_command - 1.0f);
		forward_speed_normalized -= sign(forward_speed_normalized) * excess;
	}

	// Calculate the left and right wheel speeds
	return Vector2f(forward_speed_normalized - speed_diff_normalized,
			forward_speed_normalized + speed_diff_normalized);
}

void RoverDifferentialControl::resetControllers()
{
	pid_reset_integral(&_pid_throttle);
	pid_reset_integral(&_pid_yaw_rate);
	pid_reset_integral(&_pid_yaw);
}
