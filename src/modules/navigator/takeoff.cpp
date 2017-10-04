/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file Takeoff.cpp
 *
 * Helper class to Takeoff
 *
 * @author Lorenz Meier <lorenz@px4.io
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>

#include "takeoff.h"
#include "navigator.h"

Takeoff::Takeoff(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_param_min_alt(this, "MIS_TAKEOFF_ALT", false)
{
	// load initial params
	updateParams();
}

Takeoff::~Takeoff()
{
}

void
Takeoff::on_inactive()
{
}

void
Takeoff::on_activation()
{
	set_takeoff_position();
}

void
Takeoff::on_active()
{
	struct position_setpoint_triplet_s *rep = _navigator->get_takeoff_triplet();

	if (rep->current.valid) {
		// reset the position
		set_takeoff_position();

	} else if (is_navigator_item_reached() && !_navigator->get_mission_result()->finished) {
		_navigator->get_mission_result()->finished = true;
		_navigator->set_mission_result_updated();

		// set loiter item so position controllers stop doing takeoff logic
		set_loiter_item(&_navigator_item);
		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		navigator_apply_limitation(_navigation_item);
		navigator_item_to_position_setpoint(_navigator_item, &pos_sp_triplet->current);
		_navigator->set_position_setpoint_triplet_updated();
	}
}

void
Takeoff::set_takeoff_position()
{
	struct position_setpoint_triplet_s *rep = _navigator->get_takeoff_triplet();

	// default is minimum altitude
	float height_above_ground_sp =  _param_min_alt.get();
	float reference_height = _navigator->get_local_reference_alt();

	// check if desired setpoint and home position is valid
	if (rep->current.valid && PX4_ISFINITE(rep->current.z)) {

		//get desired height
		float desired_height = -rep->current.z - _navigator->get_local_reference_alt();

		if (_navigator->home_position_valid()) {
			desired_height = -rep->current.z - _navigator->get_home_position()->z;
			reference_height = _navigator->get_home_position()->z;
		}

		if (desired_height > height_above_ground_sp) {
			height_above_ground_sp = desired_height;
		}

	} else {
		mavlink_log_info(_navigator->get_mavlink_log_pub(),
				 "Using minimum takeoff altitude: %.2f m", (double)_param_min_alt.get());
	}

	// if we are already heigher, don't go down
	if (height_above_ground_sp < -(_navigator->get_local_position()->z - _navigator->get_home_position()->z)) {

		height_above_ground_sp = -(_navigator->get_local_position()->z - _navigator->get_home_position()->z);
		mavlink_log_critical(_navigator->get_mavlink_log_pub(),
				     "Already higher than takeoff altitude");

	}

	float lpos_z = reference_height - height_above_ground_sp;

	// set current mission item to takeoff
	set_takeoff_item(&_navigator_item, lpos_z);
	_navigator->get_mission_result()->reached = false;
	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();
	reset_navigator_item_reached();

	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	navigator_apply_limitation(_navigation_item);
	navigator_item_to_position_setpoint(_navigator_item, &pos_sp_triplet->current);
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->current.yaw_valid = true;
	pos_sp_triplet->next.valid = false;


	//TODO: what is the purpose of this
	if (rep->current.valid) {

		// Go on and check which changes had been requested
		if (PX4_ISFINITE(rep->current.yaw)) {
			pos_sp_triplet->current.yaw = rep->current.yaw;
		}

		if (PX4_ISFINITE(rep->current.x) && PX4_ISFINITE(rep->current.y)) {
			pos_sp_triplet->current.x = rep->current.x;
			pos_sp_triplet->current.y = rep->current.y;
		}

		// mark this as done
		memset(rep, 0, sizeof(*rep));
	}

	_navigator->set_can_loiter_at_sp(true);

	_navigator->set_position_setpoint_triplet_updated();
}
