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

#include <mavlink/mavlink_log.h>
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
	_current_target_state(ACSEND),
	_target_motion_position_sub(-1),
	_previous_target_gps_pos_valid(false),
	_radius_entered(false),
    _radius_exited(false),
    _last_publish_time(0),
    _current_target_motion({0}),
    _previous_target_motion({0}),
    _dt(0)
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

void
FollowTarget::on_inactive()
{
    _previous_target_gps_pos_valid = false;
    _current_target_state = ACSEND;
}

void
FollowTarget::on_activation()
{
	if(_target_motion_position_sub < 0) {
		_target_motion_position_sub = orb_subscribe(ORB_ID(follow_target));
	}

	update_target_motion();

	reset_mission_item_reached();
}

void
FollowTarget::loiter() {
    math::Vector<3> vel;

    _current_vel(0) = 0;
    _current_vel(1) = 0;

    set_loiter_item(&_mission_item, _param_min_alt.get());

    update_position_sp(vel);

    _current_target_motion.lat = _navigator->get_global_position()->lat;
    _current_target_motion.lon = _navigator->get_global_position()->lon;
}

void
FollowTarget::on_active() {
    struct map_projection_reference_s target_ref;

    update_target_motion();

    // get distance to target

    map_projection_init(&target_ref, _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);
    map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon, &_target_distance(0), &_target_distance(1));

    // are we within the target acceptance radius?
    // give a buffer to exit/enter the radius to give the controller
    // a chance to catch up

    _radius_entered = (_target_distance.length() < (float) TARGET_ACCEPTANCE_RADIUS_M * 1.5f);
    _radius_exited = (_target_distance.length() > (float) TARGET_ACCEPTANCE_RADIUS_M);

    switch (_current_target_state) {
        case TRACK_POSITION:
        {
            if (_radius_entered == true) {
                _current_target_state = TRACK_VELOCITY;
            } else {
                track_position();
            }
            break;
        }
        case TRACK_VELOCITY:
        {
            if (_radius_exited == true) {
                _current_target_state = TRACK_POSITION;
            } else {
                track_velocity();
            }
            break;
        }
        case ACSEND:
        {
            // ascend to the minimum altitude

            loiter();

            _mission_item.nav_cmd = NAV_CMD_WAYPOINT;

            if(is_mission_item_reached()) {
                _current_target_state = TRACK_POSITION;
            }
            break;
        }
        case TARGET_TIMEOUT:
        {
            // Loiter until signal is regained

            loiter();
            break;
        }
    }

    update_position_sp(_current_vel);
}

void
FollowTarget::track_position() {

    set_follow_target_item(&_mission_item, _param_min_alt.get(), _current_target_motion, NAN);
    _mission_item.nav_cmd = NAV_CMD_WAYPOINT;

    // keep the current velocity updated

    _current_vel(0) = _navigator->get_global_position()->vel_n;
    _current_vel(1) = _navigator->get_global_position()->vel_e;
}

void
FollowTarget::track_velocity() {

    uint64_t current_time = hrt_absolute_time();

    if (_previous_target_gps_pos_valid == false) {
        return;
    }

    set_follow_target_item(&_mission_item, _param_min_alt.get(), _current_target_motion, NAN);

    // publish a new velocity every 100 ms

    if ((((double) (current_time - _last_publish_time) * 1e-3) >= 100)) {
        _current_vel += _step_vel;
        _last_publish_time = current_time;
    }
}

void
FollowTarget::update_target_motion() {
    bool updated;

    orb_check(_target_motion_position_sub, &updated);

    if (updated) {

        // save last known motion topic

        _previous_target_motion = _current_target_motion;

        orb_copy(ORB_ID(follow_target), _target_motion_position_sub, &_current_target_motion);

        update_target_velocity();

        if (_previous_target_gps_pos_valid == false) {
            _previous_target_motion = _current_target_motion;
            _previous_target_gps_pos_valid = true;
        }

            warnx(" lat %f (%f) lon %f (%f), dist = %f",
                    _current_target_motion.lat,
                    (double)_navigator->get_global_position()->lat,
                    _current_target_motion.lon,
                    (double)_navigator->get_global_position()->lon,
                    (double) _target_distance.length());
    }

    if ((((double)(hrt_absolute_time() - _previous_target_motion.timestamp) * 1e-6) > 10)) {
        _current_target_state = TARGET_TIMEOUT;
    } else if(_current_target_state == TARGET_TIMEOUT) {
        _current_target_state = TRACK_POSITION;
    }
}

void
FollowTarget::update_target_velocity() {
    struct map_projection_reference_s target_ref;
    math::Vector<3> target_position(0,0,0);

    _dt = ((double) (_current_target_motion.timestamp - _previous_target_motion.timestamp) * 1e-6);

    // get last gps reference for target

    map_projection_init(&target_ref, _previous_target_motion.lat, _previous_target_motion.lon);

    // calculate distance the target has moved

    map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon, &(target_position(0)), &(target_position(1)));

    // update the average velocity of the target

    _target_vel = target_position / _dt;

    // to keep the velocity increase/decrease smooth
    // calculate how many velocity increments/decrements
    // it will take to reach the targets velocity
    // in 10 steps also add a feed forward input that increases
    // velocity as the position gap increases

    _step_vel = (_target_vel - _current_vel) + _target_distance * .15f;
    _step_vel /= (_dt * 10);
}

void
FollowTarget::update_position_sp(math::Vector<3> & vel) {

    /* convert mission item to current setpoint */
    struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

    // activate line following in pos control
    pos_sp_triplet->previous.valid = true;
    pos_sp_triplet->previous = pos_sp_triplet->current;
    mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
    pos_sp_triplet->current.vx = vel(0);
    pos_sp_triplet->current.vy = vel(1);
    pos_sp_triplet->current.velocity_valid = true;
    pos_sp_triplet->next.valid = false;

    _navigator->set_position_setpoint_triplet_updated();
}
