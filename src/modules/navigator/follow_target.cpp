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

using namespace matrix;

FollowTarget::FollowTarget(Navigator *navigator, const char *name) :
    MissionBlock(navigator, name),
	_navigator(navigator),
	_tracker_motion_position_sub(-1),
	_param_min_alt(this, "MIS_TAKEOFF_ALT", false),
	gps_valid(false),
	_last_message_time(0),
	_index(0)
{
    /* load initial params */
    updateParams();
    follow_target_reached = false;
    pos_pair[0].setZero();
    pos_pair[1].setZero();
    _current_vel.setZero();
    _steps = 0.0f;
    target_dist_x = target_dist_y = 0.0f;
    target = {};
}

FollowTarget::~FollowTarget()
{
}

void
FollowTarget::on_inactive()
{
    gps_valid = false;
}

void
FollowTarget::on_activation()
{
    Vector2f vel;
    vel.setZero();

	if(_tracker_motion_position_sub < 0) {
		_tracker_motion_position_sub = orb_subscribe(ORB_ID(follow_target));
	}

	// inital set point is same as loiter sp

    set_loiter_item(&_mission_item, _param_min_alt.get());

    update_position_sp(vel);

    target.lat = _navigator->get_global_position()->lat;
    target.lon = _navigator->get_global_position()->lon;
}

void
FollowTarget::on_active() {

    bool updated;
    struct map_projection_reference_s target_ref;
    uint64_t current_time = hrt_absolute_time();

    // get distance to target

    map_projection_init(&target_ref, _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);
    map_projection_project(&target_ref, target.lat, target.lon, &target_dist_x, &target_dist_y);

    follow_target_reached = (sqrtf(target_dist_x * target_dist_x + target_dist_y * target_dist_y) < 5);

    if(follow_target_reached == false) {
        _current_vel(0) = _navigator->get_global_position()->vel_n;
        _current_vel(1) = _navigator->get_global_position()->vel_e;
    }

    orb_check(_tracker_motion_position_sub, &updated);

    if (updated) {
        if (orb_copy(ORB_ID(follow_target), _tracker_motion_position_sub, &target) == OK) {

            float dt = ((double)(current_time - _last_message_time) * 1e-6);

            if ((gps_valid == false) ) {
                gps_pair(0) = target.lat;
                gps_pair(1) = target.lon;
                gps_valid = true;

                return;
            }

            // get last gps reference

            map_projection_init(&target_ref, gps_pair(0), gps_pair(1));
            map_projection_project(&target_ref, target.lat, target.lon, &(pos_pair[1](0)), &(pos_pair[1](1)));

            target_vel = pos_pair[1]/(dt);

            target_vel(0) += target_dist_x*.1f;
            target_vel(1) += target_dist_y*.1f;

            warnx("tracker x %f y %f m, x %f m/s, y %f m/s gs = %f m/s dis = %f m",
                    (double) pos_pair[1](0),
                    (double) pos_pair[1](1),
                    (double) target_vel(0),
                    (double) target_vel(1),
                    (double) sqrtf(target_vel(0)*target_vel(0) + target_vel(1)*target_vel(1)),
                    (double) sqrtf(target_dist_x * target_dist_x + target_dist_y * target_dist_y));

            set_follow_target_item(&_mission_item, _param_min_alt.get(), target);

            gps_pair(0) = target.lat;
            gps_pair(1) = target.lon;

            _last_message_time = current_time;

            if(follow_target_reached == false) {
                 warnx("WP not reached lat %f (%f) lon %f (%f), %f",  target.lat,  (double)_navigator->get_global_position()->lat,  (double)_navigator->get_global_position()->lon, target.lon, (double)sqrtf(target_dist_x * target_dist_x + target_dist_y * target_dist_y));
                 _mission_item.nav_cmd = NAV_CMD_WAYPOINT;
                 update_position_sp(target_vel);
                 return;
             }

            _steps =  fabs((double)((sqrtf(_current_vel(0)*_current_vel(0) + _current_vel(1)*_current_vel(1)) -
                      sqrtf(target_vel(0)*target_vel(0) + target_vel(1)*target_vel(1))))) / (double) (dt*10);

        }
    } else if (((double)(current_time - _last_message_time) * 1e-6) > 10) {
        on_activation();
    }

    if ((((double) (current_time - _last_publish_time) * 1e-3) >= 100) && (follow_target_reached == true)) {

        if (_current_vel(0) <= target_vel(0)) {
            _current_vel(0) += (_steps);
        }

        if (_current_vel(0) >= target_vel(0)) {
            _current_vel(0) -= (_steps);
        }

        if (_current_vel(1) <= target_vel(1)) {
            _current_vel(1) += (_steps);
        }

        if (_current_vel(1) >= target_vel(1)) {
            _current_vel(1) -= (_steps);
        }

        update_position_sp(_current_vel);

        warnx("updating x %f m y %f m x %f m/s y %f m/s %f ( uav gs %f)", (double )target_dist_x, (double )target_dist_y,
                (double )_current_vel(0), (double )_current_vel(1), (double ) _steps,
                (double ) sqrtf(_navigator->get_global_position()->vel_e * _navigator->get_global_position()->vel_e + _navigator->get_global_position()->vel_n * _navigator->get_global_position()->vel_n));

        _last_publish_time = current_time;
    }
}

void
FollowTarget::update_position_sp(Vector2f & vel) {

    /* convert mission item to current setpoint */
    struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

    // activate line following in pos control
    pos_sp_triplet->previous.valid = true;
    pos_sp_triplet->previous = pos_sp_triplet->current;
    mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
    pos_sp_triplet->current.vx = vel(0);
    pos_sp_triplet->current.vy = vel(1);
    pos_sp_triplet->next.valid = false;

    _navigator->set_position_setpoint_triplet_updated();
}
