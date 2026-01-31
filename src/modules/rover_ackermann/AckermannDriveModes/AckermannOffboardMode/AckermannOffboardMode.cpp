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

#include "AckermannOffboardMode.hpp"

using namespace time_literals;

AckermannOffboardMode::AckermannOffboardMode(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_rover_speed_setpoint_pub.advertise();
	_rover_position_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
}

void AckermannOffboardMode::updateParams()
{
	ModuleParams::updateParams();
}

void AckermannOffboardMode::offboardControl()
{
	offboard_control_mode_s offboard_control_mode{};
	_offboard_control_mode_sub.copy(&offboard_control_mode);

	trajectory_setpoint_s trajectory_setpoint{};
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	if (offboard_control_mode.position) {
		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.position_ned[0] = trajectory_setpoint.position[0];
		rover_position_setpoint.position_ned[1] = trajectory_setpoint.position[1];
		rover_position_setpoint.start_ned[0] = NAN;
		rover_position_setpoint.start_ned[1] = NAN;
		rover_position_setpoint.cruising_speed = NAN;
		rover_position_setpoint.arrival_speed = NAN;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);

	} else if (offboard_control_mode.velocity) {
		const Vector2f velocity_ned(trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1]);
		rover_speed_setpoint_s rover_speed_setpoint{};
		rover_speed_setpoint.timestamp = hrt_absolute_time();
		rover_speed_setpoint.speed_body_x = velocity_ned.norm();
		_rover_speed_setpoint_pub.publish(rover_speed_setpoint);
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = hrt_absolute_time();
		rover_attitude_setpoint.yaw_setpoint = atan2f(velocity_ned(1), velocity_ned(0));
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
	}
}
