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
 * @file course.h
 *
 * Course mode: maintain constant course, altitude, and airspeed.
 *
 * Accepts MAV_CMD_DO_CHANGE_ALTITUDE, MAV_CMD_GUIDED_CHANGE_HEADING and MAV_CMD_DO_CHANGE_SPEED commands.
 * - MAV_CMD_GUIDED_CHANGE_HEADING (param1=HEADING_TYPE_COURSE_OVER_GROUND): sets course (ground track direction, requires valid position).
 */

#pragma once

#include "navigator_mode.h"
#include "mission_block.h"

class Course : public MissionBlock
{
public:
	Course(Navigator *navigator);
	~Course() = default;

	void on_activation() override;
	void on_active() override;

	/**
	 * Set course (ground track direction). Requires valid horizontal velocity.
	 * @param course_rad Course angle in radians, 0 = north
	 * @return true if horizontal velocity available and course set, false otherwise
	 */
	bool set_course(float course_rad);

	void set_altitude(float alt_amsl) { _altitude = alt_amsl; update_setpoint_triplet(); }

private:
	/**
	 * Update the position setpoint triplet to fly the current course.
	 */
	void update_setpoint_triplet();

	float _course{0.f};		///< [rad] current course bearing (ground track)
	float _altitude{0.f};		///< [m] AMSL altitude setpoint
};
