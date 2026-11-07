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
/**
 * @file course.cpp
 *
 * Course mode: maintain constant course, altitude, and airspeed.
 */

#include "course.h"
#include "navigator.h"

#include <matrix/math.hpp>

Course::Course(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_GUIDED_COURSE)
{
}

void
Course::on_activation()
{
	// Snapshot the setpoint the previous mode left before resetting the triplet, so we can continue
	// a course that is already being flown, for example handed over by the fixed-wing takeoff.
	const position_setpoint_s previous_setpoint = _navigator->get_position_setpoint_triplet()->current;

	// reset triplets, modes should be explicit about which fields they want to set
	_navigator->reset_triplets();

	const bool continue_course = previous_setpoint.valid
				     && previous_setpoint.type == position_setpoint_s::SETPOINT_TYPE_POSITION
				     && PX4_ISFINITE(previous_setpoint.course);

	if (continue_course) {
		_course = previous_setpoint.course;
		_altitude = previous_setpoint.alt;

	} else {
		const vehicle_local_position_s *lpos = _navigator->get_local_position();

		_altitude = _navigator->get_global_position()->alt;

		const bool is_rover = _navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER;
		bool course_from_velocity = lpos->v_xy_valid;

		if (is_rover && PX4_ISFINITE(lpos->heading)) {
			// The velocity of a rover has no usable direction at rest, so it only takes the course over ground
			// while driving forwards fast enough, and its heading otherwise
			const float forward_speed = lpos->vx * cosf(lpos->heading) + lpos->vy * sinf(lpos->heading);
			course_from_velocity &= forward_speed > math::max(ROVER_COURSE_MIN_SPEED, ROVER_COURSE_EVH_FACTOR * lpos->evh);

			if (!course_from_velocity) {
				_course = matrix::wrap_2pi(lpos->heading);
			}
		}

		if (course_from_velocity) {
			_course = matrix::wrap_2pi(atan2f(lpos->vy, lpos->vx));
		}
	}

	_navigator->reset_cruising_speed();

	publishCourseHoldSetpoint(_course, _altitude);
}

void
Course::on_active()
{
}

bool
Course::set_course(float course_rad)
{
	if (!_navigator->get_local_position()->v_xy_valid) {
		// No valid velocity estimate - cannot compute or maintain a ground track
		return false;
	}

	_course = course_rad;
	publishCourseHoldSetpoint(_course, _altitude);
	return true;
}
