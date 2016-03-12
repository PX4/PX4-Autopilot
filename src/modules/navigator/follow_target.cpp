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
	_follow_target_state(ASCEND),
	_follow_target_sub(-1),
	_step_time_in_ms(0.0f),
	_previous_target_gps_pos_valid(false),
	_radius_entered(false),
    _radius_exited(false),
    _last_update_time(0),
    _current_target_motion({0}),
    _previous_target_motion({0})
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
    _follow_target_state = ASCEND;
}

void
FollowTarget::on_activation()
{
	if(_follow_target_sub < 0) {
		_follow_target_sub = orb_subscribe(ORB_ID(follow_target));
	}

	update_target_motion();
	reset_mission_item_reached();
}

void
FollowTarget::pause() {
    math::Vector<3> vel(0,0,0);

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

    switch (_follow_target_state) {
        case TRACK_POSITION:
        {
            if (_radius_entered == true) {
                _follow_target_state = TRACK_VELOCITY;
            } else {
                track_target_position();
            }
            break;
        }
        case TRACK_VELOCITY:
        {
            if (_radius_exited == true) {
                _follow_target_state = TRACK_POSITION;
            } else {
                track_target_velocity();
            }
            break;
        }
        case ASCEND:
        {
            // ascend to the minimum altitude

            pause();

            _mission_item.nav_cmd = NAV_CMD_WAYPOINT;

            if(is_mission_item_reached()) {
                _follow_target_state = TRACK_POSITION;
            }
            break;
        }
        case TARGET_TIMEOUT:
        {
            // Loiter until signal is regained

            pause();
            break;
        }
    }

    update_position_sp(_current_vel);
}

void
FollowTarget::track_target_position() {

    set_follow_target_item(&_mission_item, _param_min_alt.get(), _current_target_motion, NAN);
    _mission_item.nav_cmd = NAV_CMD_WAYPOINT;

    // keep the current velocity updated

    _current_vel(0) = _navigator->get_global_position()->vel_n;
    _current_vel(1) = _navigator->get_global_position()->vel_e;
}

void
FollowTarget::track_target_velocity() {

    uint64_t current_time = hrt_absolute_time();

    if (_previous_target_gps_pos_valid == false) {
        return;
    }

    set_follow_target_item(&_mission_item, _param_min_alt.get(), _current_target_motion, NAN);

    if ((current_time - _last_update_time)/1000 >= _step_time_in_ms) {
        _current_vel += _step_vel;
        _last_update_time = current_time;
    }
}

void
FollowTarget::update_target_motion() {
    bool updated;

    orb_check(_follow_target_sub, &updated);

    if (updated) {

        // save last known motion topic

        _previous_target_motion = _current_target_motion;

        orb_copy(ORB_ID(follow_target), _follow_target_sub, &_current_target_motion);

        update_target_velocity();

        if (_previous_target_gps_pos_valid == false) {
            _previous_target_motion = _current_target_motion;
            _previous_target_gps_pos_valid = true;
        }

//            warnx(" lat %f (%f) lon %f (%f), dist = %f",
//                    _current_target_motion.lat,
//                    (double)_navigator->get_global_position()->lat,
//                    _current_target_motion.lon,
//                    (double)_navigator->get_global_position()->lon,
//                    (double) _target_distance.length());
    }

    if ((float) ((hrt_absolute_time() - _previous_target_motion.timestamp)/1000/1000) > TARGET_TIMEOUT_S) {
        _follow_target_state = TARGET_TIMEOUT;
    } else if(_follow_target_state == TARGET_TIMEOUT) {
        _follow_target_state = TRACK_POSITION;
    }
}

void
FollowTarget::update_target_velocity() {
    float dt_ms;
    math::Vector<3> target_position(0,0,0);
    struct map_projection_reference_s target_ref;

    dt_ms = ((_current_target_motion.timestamp - _previous_target_motion.timestamp)/1000);

    // get last gps known reference for target

    map_projection_init(&target_ref, _previous_target_motion.lat, _previous_target_motion.lon);

    // calculate distance the target has moved

    map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon, &(target_position(0)), &(target_position(1)));

    // update the average velocity of the target based on the position

    _target_vel = target_position / (dt_ms/1000.0f);

    // to keep the velocity increase/decrease smooth
    // calculate how many velocity increments/decrements
    // it will take to reach the targets velocity
    // with the given amount of steps also add a feed forward input that adjusts the
    // velocity as the position gap increases since
    // just traveling at the exact velocity of the target will not
    // get any closer to the target

    _step_vel = (_target_vel - _current_vel) + _target_distance * FF_K;
    _step_vel /= (dt_ms/1000.0f * (float) INTERPOLATION_PNTS);
    _step_time_in_ms = dt_ms / (float) INTERPOLATION_PNTS;
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
