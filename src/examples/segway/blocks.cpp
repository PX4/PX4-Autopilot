/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file uorb_blocks.cpp
 *
 * uorb block library code
 */

#include "blocks.hpp"
#include <lib/ecl/geo/geo.h>
#include <matrix/math.hpp>

using matrix::wrap_2pi;

namespace control
{

BlockWaypointGuidance::BlockWaypointGuidance(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	_xtYawLimit(this, "XT2YAW"),
	_xt2Yaw(this, "XT2YAW"),
	_psiCmd(0)
{
}

void BlockWaypointGuidance::update(
	const vehicle_global_position_s &pos,
	const vehicle_attitude_s &att,
	const position_setpoint_s &missionCmd,
	const position_setpoint_s &lastMissionCmd)
{

	// heading to waypoint
	float psiTrack = get_bearing_to_next_waypoint(
				 (double)pos.lat / (double)1e7,
				 (double)pos.lon / (double)1e7,
				 missionCmd.lat,
				 missionCmd.lon);

	// cross track
	struct crosstrack_error_s xtrackError;
	get_distance_to_line(&xtrackError,
			     (double)pos.lat / (double)1e7,
			     (double)pos.lon / (double)1e7,
			     lastMissionCmd.lat,
			     lastMissionCmd.lon,
			     missionCmd.lat,
			     missionCmd.lon);

	_psiCmd = wrap_2pi(psiTrack -
			   _xtYawLimit.update(_xt2Yaw.update(xtrackError.distance)));
}

BlockUorbEnabledAutopilot::BlockUorbEnabledAutopilot(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	// subscriptions
	_att(ORB_ID(vehicle_attitude), 20),

	_manual(ORB_ID(manual_control_setpoint), 20, 0, &getSubscriptions()),
	_param_update(ORB_ID(parameter_update), 1000, 0, &getSubscriptions()), // limit to 1 Hz
	_missionCmd(ORB_ID(position_setpoint_triplet), 20, 0, &getSubscriptions()),
	_attCmd(ORB_ID(vehicle_attitude_setpoint), 20, 0, &getSubscriptions()),
	_pos(ORB_ID(vehicle_global_position), 20, 0, &getSubscriptions()),
	_ratesCmd(ORB_ID(vehicle_rates_setpoint), 20, 0, &getSubscriptions()),
	_status(ORB_ID(vehicle_status), 20, 0, &getSubscriptions()),

	// publications
	_actuators(ORB_ID(actuator_controls_0), -1, &getPublications())
{
}

}

