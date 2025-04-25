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

#include "AckermannVelControl.hpp"

using namespace time_literals;

AckermannVelControl::AckermannVelControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_throttle_setpoint_pub.advertise();
	_rover_velocity_status_pub.advertise();
	_ackermann_velocity_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
	updateParams();
}

void AckermannVelControl::updateParams()
{
	ModuleParams::updateParams();
	_pid_speed.setGains(_param_ro_speed_p.get(), _param_ro_speed_i.get(), 0.f);
	_pid_speed.setIntegralLimit(1.f);
	_pid_speed.setOutputLimit(1.f);

	if (_param_ro_accel_limit.get() > FLT_EPSILON) {
		_speed_setpoint.setSlewRate(_param_ro_accel_limit.get());
	}

}

void AckermannVelControl::updateVelControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	updateSubscriptions();

	if ((_vehicle_control_mode.flag_control_velocity_enabled) && _vehicle_control_mode.flag_armed && runSanityChecks()) {
		if (_vehicle_control_mode.flag_control_offboard_enabled) { // Offboard Velocity Control
			generateVelocitySetpoint();
		}

		generateAttitudeAndThrottleSetpoint();

	} else { // Reset controller and slew rate when position control is not active
		_pid_speed.resetIntegral();
		_speed_setpoint.setForcedValue(0.f);
	}

	// Publish position controller status (logging only)
	rover_velocity_status_s rover_velocity_status;
	rover_velocity_status.timestamp = _timestamp;
	rover_velocity_status.measured_speed_body_x = _vehicle_speed_body_x;
	rover_velocity_status.adjusted_speed_body_x_setpoint = _speed_setpoint.getState();
	rover_velocity_status.measured_speed_body_y = _vehicle_speed_body_y;
	rover_velocity_status.adjusted_speed_body_y_setpoint = NAN;
	rover_velocity_status.pid_throttle_body_x_integral = _pid_speed.getIntegral();
	rover_velocity_status.pid_throttle_body_y_integral = NAN;
	_rover_velocity_status_pub.publish(rover_velocity_status);
}

void AckermannVelControl::updateSubscriptions()
{
	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);

		Vector3f velocity_in_local_frame(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
		_vehicle_speed_body_x = fabsf(velocity_in_body_frame(0)) > _param_ro_speed_th.get() ? velocity_in_body_frame(0) : 0.f;
		_vehicle_speed_body_y = fabsf(velocity_in_body_frame(1)) > _param_ro_speed_th.get() ? velocity_in_body_frame(1) : 0.f;
	}

}

void AckermannVelControl::generateVelocitySetpoint()
{
	trajectory_setpoint_s trajectory_setpoint{};
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	if (_offboard_control_mode_sub.updated()) {
		_offboard_control_mode_sub.copy(&_offboard_control_mode);
	}

	const bool offboard_vel_control = _offboard_control_mode.velocity && !_offboard_control_mode.position;

	const Vector2f velocity_in_local_frame(trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1]);

	if (offboard_vel_control && velocity_in_local_frame.isAllFinite()) {
		ackermann_velocity_setpoint_s ackermann_velocity_setpoint{};
		ackermann_velocity_setpoint.timestamp = _timestamp;
		ackermann_velocity_setpoint.velocity_ned[0] = velocity_in_local_frame(0);
		ackermann_velocity_setpoint.velocity_ned[1] = velocity_in_local_frame(1);
		ackermann_velocity_setpoint.backwards = false;
		_ackermann_velocity_setpoint_pub.publish(ackermann_velocity_setpoint);
	}
}

void AckermannVelControl::generateAttitudeAndThrottleSetpoint()
{
	if (_ackermann_velocity_setpoint_sub.updated()) {
		_ackermann_velocity_setpoint_sub.copy(&_ackermann_velocity_setpoint);
	}

	// Attitude Setpoint
	if (fabsf(_ackermann_velocity_setpoint.velocity_ned[1]) < FLT_EPSILON
	    && fabsf(_ackermann_velocity_setpoint.velocity_ned[0]) < FLT_EPSILON) {
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = _timestamp;
		rover_attitude_setpoint.yaw_setpoint = _vehicle_yaw;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

	} else {
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = _timestamp;
		const float yaw_setpoint = atan2f(_ackermann_velocity_setpoint.velocity_ned[1],
						  _ackermann_velocity_setpoint.velocity_ned[0]);
		rover_attitude_setpoint.yaw_setpoint = _ackermann_velocity_setpoint.backwards ? matrix::wrap_pi(
				yaw_setpoint + M_PI_F) : yaw_setpoint;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
	}

	// Throttle Setpoint
	const float speed_magnitude = math::min(sqrtf(powf(_ackermann_velocity_setpoint.velocity_ned[0],
						2) + powf(_ackermann_velocity_setpoint.velocity_ned[1], 2)), _param_ro_speed_limit.get());
	const float speed_body_x_setpoint = _ackermann_velocity_setpoint.backwards ? -speed_magnitude : speed_magnitude;
	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = _timestamp;
	rover_throttle_setpoint.throttle_body_x = RoverControl::speedControl(_speed_setpoint, _pid_speed,
			speed_body_x_setpoint, _vehicle_speed_body_x, _param_ro_accel_limit.get(), _param_ro_decel_limit.get(),
			_param_ro_max_thr_speed.get(), _dt);
	rover_throttle_setpoint.throttle_body_y = 0.f;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);

}

bool AckermannVelControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_max_thr_speed.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;

		if (_prev_param_check_passed) {
			events::send<float>(events::ID("ackermann_vel_control_conf_invalid_speed_lim"), events::Log::Error,
					    "Invalid configuration of necessary parameter RO_SPEED_LIM", _param_ro_speed_limit.get());
		}

	}

	_prev_param_check_passed = ret;
	return ret;
}
