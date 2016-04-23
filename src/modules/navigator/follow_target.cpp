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

#include "follow_target.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/follow_target.h>
#include <lib/geo/geo.h>

#include "navigator.h"

FollowTarget::FollowTarget(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_navigator(navigator),
	_param_min_alt(this, "MIS_TAKEOFF_ALT", false),
	_follow_target_state(WAIT_FOR_TARGET_POSITION),
	_follow_target_sub(-1),
	_step_time_in_ms(0.0f),
	_target_updates(0),
	_last_update_time(0),
	_current_target_motion( {}),
			_previous_target_motion({})
{
	updateParams();
	_current_vel.zero();
	_step_vel.zero();
	_target_vel.zero();
	_target_distance.zero();
}

FollowTarget::~FollowTarget()
{
}

void FollowTarget::on_inactive()
{
	reset_target_validity();
}

void FollowTarget::on_activation()
{
	if (_follow_target_sub < 0) {
		_follow_target_sub = orb_subscribe(ORB_ID(follow_target));
	}
}

void FollowTarget::on_active()
{
	struct map_projection_reference_s target_ref;
	math::Vector<3> target_position(0, 0, 0);
	uint64_t current_time = hrt_absolute_time();
	bool _radius_entered = false;
	bool _radius_exited = false;
	bool updated = false;
	float dt_ms = 0;

	orb_check(_follow_target_sub, &updated);

	if (updated) {

		_target_updates++;

		// save last known motion topic

		_previous_target_motion = _current_target_motion;

		orb_copy(ORB_ID(follow_target), _follow_target_sub, &_current_target_motion);

	} else if (((current_time - _previous_target_motion.timestamp) / 1000 / 1000) > TARGET_TIMEOUT_S
		   && target_velocity_valid()) {
		reset_target_validity();
	}

	// update target velocity

	if (target_velocity_valid() && updated) {

		dt_ms = ((_current_target_motion.timestamp - _previous_target_motion.timestamp) / 1000);

		// get last gps known reference for target

		map_projection_init(&target_ref, _previous_target_motion.lat, _previous_target_motion.lon);

		// calculate distance the target has moved

		map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon, &(target_position(0)),
				       &(target_position(1)));

		// update the average velocity of the target based on the position

		_target_vel = target_position / (dt_ms / 1000.0f);

		// to keep the velocity increase/decrease smooth
		// calculate how many velocity increments/decrements
		// it will take to reach the targets velocity
		// with the given amount of steps also add a feed forward input that adjusts the
		// velocity as the position gap increases since
		// just traveling at the exact velocity of the target will not
		// get any closer to the target

		_step_vel = (_target_vel - _current_vel) + _target_distance * FF_K;
		_step_vel /= (dt_ms / 1000.0f * (float) INTERPOLATION_PNTS);
		_step_time_in_ms = dt_ms / (float) INTERPOLATION_PNTS;
	}

	// update distance to target

	if (target_position_valid()) {

		// get distance to target

		map_projection_init(&target_ref, _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);
		map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon, &_target_distance(0),
				       &_target_distance(1));

		// are we within the target acceptance radius?
		// give a buffer to exit/enter the radius to give the velocity controller
		// a chance to catch up

		_radius_exited = (_target_distance.length() > (float) TARGET_ACCEPTANCE_RADIUS_M * 1.5f);
		_radius_entered = (_target_distance.length() < (float) TARGET_ACCEPTANCE_RADIUS_M);
	}

	// update state machine

	switch (_follow_target_state) {

	case TRACK_POSITION: {

			if (_radius_entered == true) {
				_follow_target_state = TRACK_VELOCITY;

			} else if (target_velocity_valid()) {
				set_follow_target_item(&_mission_item, _param_min_alt.get(), _current_target_motion, NAN);
				// keep the current velocity updated with the target velocity for when it's needed
				_current_vel = _target_vel;
				update_position_sp(true, true);
			}

			break;
		}

	case TRACK_VELOCITY: {

			if (_radius_exited == true) {
				_follow_target_state = TRACK_POSITION;

			} else if (target_velocity_valid()) {
				set_follow_target_item(&_mission_item, _param_min_alt.get(), _current_target_motion, NAN);

				if ((current_time - _last_update_time) / 1000 >= _step_time_in_ms) {
					_current_vel += _step_vel;
					_last_update_time = current_time;
				}

				update_position_sp(true, false);
			}

			break;
		}

	case WAIT_FOR_TARGET_POSITION: {

			// Climb to the minimum altitude
			// and wait until a position is received

			follow_target_s target = { };

			// for now set the target at the minimum height above the uav

			target.lat = _navigator->get_global_position()->lat;
			target.lon = _navigator->get_global_position()->lon;
			target.alt = 0.0F;

			set_follow_target_item(&_mission_item, _param_min_alt.get(), target, NAN);

			update_position_sp(false, false);

			if (is_mission_item_reached() && target_velocity_valid()) {
				_follow_target_state = TRACK_POSITION;
			}

			break;
		}
	}
}

void FollowTarget::update_position_sp(bool use_velocity, bool use_position)
{
	// convert mission item to current setpoint

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// activate line following in pos control if position is valid

	pos_sp_triplet->previous.valid = use_position;
	pos_sp_triplet->previous = pos_sp_triplet->current;
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
	pos_sp_triplet->current.position_valid = use_position;
	pos_sp_triplet->current.velocity_valid = use_velocity;
	pos_sp_triplet->current.vx = _current_vel(0);
	pos_sp_triplet->current.vy = _current_vel(1);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

void FollowTarget::reset_target_validity()
{
	_previous_target_motion = {};
	_current_target_motion = {};
	_target_updates = 0;
	_current_vel.zero();
	_step_vel.zero();
	_target_vel.zero();
	_target_distance.zero();
	reset_mission_item_reached();
	_follow_target_state = WAIT_FOR_TARGET_POSITION;
}

bool FollowTarget::target_velocity_valid()
{
	// need at least 2 data points for velocity estimate
	return (_target_updates >= 2);
}

bool FollowTarget::target_position_valid()
{
	// need at least 1 data point for position estimate
	return (_target_updates >= 1);
}
