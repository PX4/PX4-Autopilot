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
#include "navigator.h"

FollowTarget::FollowTarget(Navigator *navigator, const char *name) :
    MissionBlock(navigator, name),
	_navigator(navigator),
	_tracker_motion_position_sub(-1),
	_param_min_alt(this, "MIS_TAKEOFF_ALT", false)
{
    /* load initial params */
    updateParams();
}

FollowTarget::~FollowTarget()
{
}

void
FollowTarget::on_inactive()
{
}

void
FollowTarget::on_activation()
{
	if(_tracker_motion_position_sub < 0) {
		_tracker_motion_position_sub = orb_subscribe(ORB_ID(follow_target));
	}

	// inital set point is same as loiter sp

    set_loiter_item(&_mission_item, _param_min_alt.get());

    convert_mission_item_to_sp();
}

void
FollowTarget::on_active() {
    follow_target_s target;
    bool updated;

    orb_check(_tracker_motion_position_sub, &updated);

    if (updated) {
        if (orb_copy(ORB_ID(follow_target), _tracker_motion_position_sub, &target) == OK) {
            set_follow_target_item(&_mission_item, _param_min_alt.get(), target);
            convert_mission_item_to_sp();
        }
    }
}

void
FollowTarget::convert_mission_item_to_sp() {

    /* convert mission item to current setpoint */
    struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

    // activate line following in pos control

    pos_sp_triplet->previous.valid = true;
    pos_sp_triplet->previous = pos_sp_triplet->current;
    mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
    pos_sp_triplet->next.valid = false;

    _navigator->set_position_setpoint_triplet_updated();
}
