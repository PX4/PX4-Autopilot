/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file followme.cpp
 *
 * Helper class to track and follow a given position
 *
 * @author Jimmy Johnson <catch22@fastmail.net>
 */

#include "circle.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/circle.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/math/Limits.hpp>

#include "navigator.h"
hrt_abstime t_prev = 0;
Circle::Circle(Navigator *navigator, const char *name) :
		MissionBlock(navigator, name), _navigator(navigator), _param_min_alt(
				this, "MIS_LTRMIN_ALT", false), _param_yawmode(this,
				"MIS_YAWMODE", false), _circle_pos_set(false), center_x(0), center_y(
				0), alt(0), deg_per_sec(0.1), _angle(0.0f), _yaw_angle(1.0f) {
	target_circle= {};
	updateParams();
}

Circle::~Circle() {

}

void Circle::on_inactive() {
	_circle_pos_set = false;
}

void Circle::on_activation() {
	if (_circle_pos_set) {
		reposition();

	} else {
		set_circle_position();
	}
}

void Circle::on_active() {
	if (_circle_pos_set) {
		reposition();
	}
	// reset the loiter position if we get disarmed
	if (_navigator->get_vstatus()->arming_state
			!= vehicle_status_s::ARMING_STATE_ARMED) {
		_circle_pos_set = false;
	}
}
void Circle::reposition() {
	// we can't reposition if we are not armed yet
	if (_navigator->get_vstatus()->arming_state
			!= vehicle_status_s::ARMING_STATE_ARMED) {
		return;
	}
	struct map_projection_reference_s target_ref;
	hrt_abstime t = hrt_absolute_time();
	float dt = t_prev != 0 ? (t - t_prev) / 1e6f : 0.004f;
	t_prev = t;
	float angular_change = deg_per_sec * dt;
	_angle += angular_change;
	target_circle.x = target_circle.radius * cosf(-_angle);
	target_circle.y = target_circle.radius * sinf(-_angle);
	map_projection_init(&target_ref, center_x, center_y);
	map_projection_reproject(&target_ref, target_circle.x, target_circle.y,
			&target_circle.lat, &target_circle.lon);
	set_circle_item(&_mission_item, 5, target_circle, _yaw_angle);
	struct position_setpoint_triplet_s *pos_sp_triplet =
			_navigator->get_position_setpoint_triplet();
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->current.type =
			position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
	pos_sp_triplet->current.position_valid = true;
	pos_sp_triplet->current.velocity_valid = false;
	pos_sp_triplet->next.valid = false;
	pos_sp_triplet->current.yawspeed_valid = false;
	pos_sp_triplet->current.yawspeed = 0;
	_navigator->set_position_setpoint_triplet_updated();

}
void Circle::set_circle_position() {
	if (_navigator->get_vstatus()->arming_state
			!= vehicle_status_s::ARMING_STATE_ARMED
			&& _navigator->get_land_detected()->landed) {

		_navigator->set_can_loiter_at_sp(false);
		_navigator->get_position_setpoint_triplet()->current.type =
				position_setpoint_s::SETPOINT_TYPE_IDLE;
		_navigator->set_position_setpoint_triplet_updated();
		_circle_pos_set = false;
		return;

	} else if (_circle_pos_set) {
		// Already set, nothing to do.
		return;
	}

	_circle_pos_set = true;
	target_circle.lat =_navigator->get_global_position()->lat;
	target_circle.lon =_navigator->get_global_position()->lon;
	target_circle.radius = 0;
	// set current mission item to loiter
	alt = _navigator->get_global_position()->alt;//pos_sp_triplet->current.alt;
	set_circle_item(&_mission_item, alt, target_circle, _yaw_angle);
	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet =
			_navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.velocity_valid = false;
	pos_sp_triplet->previous.valid = false;
	center_x = _navigator->get_global_position()->lat;//pos_sp_triplet->current.lat;
	center_y = _navigator->get_global_position()->lon;//pos_sp_triplet->current.lon;
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}
