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

#include "BoatControl.hpp"

using namespace matrix;

BoatControl::BoatControl(ModuleParams *parent) : ModuleParams(parent)
{
	pid_init(&_pid_angular_velocity, PID_MODE_DERIVATIV_NONE, 0.001f);
	pid_init(&_pid_speed, PID_MODE_DERIVATIV_NONE, 0.001f);
}

void BoatControl::updateParams()
{
	ModuleParams::updateParams();

	pid_set_parameters(&_pid_angular_velocity,
			   _param_bt_ang_p.get(),
			   _param_bt_ang_i.get(),
			   0,
			   _param_bt_ang_imax.get(),
			   _param_bt_ang_outlim.get());

	pid_set_parameters(&_pid_speed,
			   _param_bt_spd_p.get(),
			   _param_bt_spd_i.get(),
			   0,
			   _param_bt_spd_imax.get(),
			   _param_bt_spd_outlim.get());
}

void BoatControl::control(float dt)
{
	if (_vehicle_angular_velocity_sub.updated()) {
		vehicle_angular_velocity_s vehicle_angular_velocity{};

		if (_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
			_vehicle_body_yaw_rate = vehicle_angular_velocity.xyz[2];
		}
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};

		if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {
			_vehicle_attitude_quaternion = Quatf(vehicle_attitude.q);
			_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
		}
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};

		if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
			_vehicle_local_position = vehicle_local_position;
			Vector3f velocity_in_local_frame(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
			Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
			_vehicle_forward_speed = velocity_in_body_frame(0);
		}
	}

	_boat_setpoint_sub.update(&_boat_setpoint);

	// PID to reach setpoint using control_output
	boat_setpoint_s boat_control_output = _boat_setpoint;

	if (_boat_setpoint.closed_loop_speed_control) {
		boat_control_output.speed +=
			pid_calculate(&_pid_speed, _boat_setpoint.speed, _vehicle_forward_speed, 0, dt);
	}

	if (_boat_setpoint.closed_loop_yaw_rate_control) {
		boat_control_output.yaw_rate +=
			pid_calculate(&_pid_angular_velocity, _boat_setpoint.yaw_rate, _vehicle_body_yaw_rate, 0, dt);
	}

	boat_control_output.timestamp = hrt_absolute_time();
	_boat_control_output_pub.publish(boat_control_output);
}
