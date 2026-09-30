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
	// reset triplets, modes should be explicit about which fields they want to set
	_navigator->reset_triplets();

	const vehicle_local_position_s *lpos = _navigator->get_local_position();

	_altitude = _navigator->get_global_position()->alt;

	if (lpos->v_xy_valid) {
		_course = matrix::wrap_2pi(atan2f(lpos->vy, lpos->vx));
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
