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

#include "MecanumCourseControl.hpp"

using namespace time_literals;

MecanumCourseControl::MecanumCourseControl(ModuleParams *parent) :
	ModuleParams(parent),
	RoverControllerWorkItem("rover_mecanum_course_control", ORB_ID(vehicle_local_position))
{
	_surface_vehicle_speed_setpoint_pub.advertise();
	_surface_vehicle_attitude_setpoint_pub.advertise();

	updateParams();
}

void MecanumCourseControl::updateParams()
{
	ModuleParams::updateParams();
}

void MecanumCourseControl::updateCourseControl()
{
	updateSubscriptions();

	// The last valid course setpoint is controlled until a NaN course is received
	if (!PX4_ISFINITE(_course_setpoint) && !PX4_ISFINITE(_speed_setpoint)) {
		return;
	}

	const hrt_abstime timestamp = hrt_absolute_time();
	surface_vehicle_speed_setpoint_s surface_vehicle_speed_setpoint{};
	surface_vehicle_speed_setpoint.timestamp = timestamp;

	if (PX4_ISFINITE(_course_setpoint) && PX4_ISFINITE(_speed_setpoint)) {
		// The rover drives along the course independent of its heading.
		// The speed controller tracks the ground velocity in body frame, so a velocity along the course setpoint needs no drift angle correction.
		const Vector3f velocity_in_local_frame(_speed_setpoint * cosf(_course_setpoint), _speed_setpoint * sinf(_course_setpoint),
						       0.f);
		const Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
		surface_vehicle_speed_setpoint.x = velocity_in_body_frame(0);
		surface_vehicle_speed_setpoint.y = velocity_in_body_frame(1);

	} else if (PX4_ISFINITE(_course_setpoint)) {
		// No speed setpoint: the course is held with whatever thrust is commanded.
		// The heading is turned so that the commanded thrust direction in body frame points along the course,
		// e.g. thrust to the right drives along the course with the course on the left of the heading.
		// The drift angle is estimated relative to the thrust direction, so it compensates the slip in any direction of travel.
		surface_vehicle_attitude_setpoint_s surface_vehicle_attitude_setpoint{};
		surface_vehicle_attitude_setpoint.timestamp = timestamp;
		surface_vehicle_attitude_setpoint.yaw_setpoint = matrix::wrap_pi(_course_setpoint - _thrust_direction
				- _drift_angle_estimator.getDriftAngle());
		_surface_vehicle_attitude_setpoint_pub.publish(surface_vehicle_attitude_setpoint);

		// NaN deactivates the speed controller, the thrust then comes from elsewhere
		surface_vehicle_speed_setpoint.x = NAN;
		surface_vehicle_speed_setpoint.y = NAN;

	} else {
		// Without a course the speed is still passed on along the body x axis, e.g. the position controller stopping at its target
		surface_vehicle_speed_setpoint.x = _speed_setpoint;
		surface_vehicle_speed_setpoint.y = 0.f;
	}

	_surface_vehicle_speed_setpoint_pub.publish(surface_vehicle_speed_setpoint);
}

void MecanumCourseControl::updateSubscriptions()
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
	}

	if (_surface_vehicle_thrust_setpoint_sub.updated()) {
		surface_vehicle_thrust_setpoint_s surface_vehicle_thrust_setpoint{};
		_surface_vehicle_thrust_setpoint_sub.copy(&surface_vehicle_thrust_setpoint);
		const float thrust_x = PX4_ISFINITE(surface_vehicle_thrust_setpoint.x) ? surface_vehicle_thrust_setpoint.x : 0.f;
		const float thrust_y = PX4_ISFINITE(surface_vehicle_thrust_setpoint.y) ? surface_vehicle_thrust_setpoint.y : 0.f;

		// Without thrust the direction is kept, so the vehicle does not turn around when the thrust passes through zero
		if (fabsf(thrust_x) > FLT_EPSILON || fabsf(thrust_y) > FLT_EPSILON) {
			_thrust_direction = atan2f(thrust_y, thrust_x);
		}
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		_drift_angle_estimator.update(vehicle_local_position, _thrust_direction);
	}

	if (_surface_vehicle_course_setpoint_sub.updated()) {
		surface_vehicle_course_setpoint_s surface_vehicle_course_setpoint{};
		_surface_vehicle_course_setpoint_sub.copy(&surface_vehicle_course_setpoint);
		_course_setpoint = surface_vehicle_course_setpoint.course;
		_speed_setpoint = surface_vehicle_course_setpoint.speed;
	}
}

bool MecanumCourseControl::runSanityChecks()
{
	return true;
}
