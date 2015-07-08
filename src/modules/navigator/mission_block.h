/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file mission_block.h
 *
 * Helper class to use mission items
 *
 * @author Julian Oes <julian@oes.ch>
 */

#ifndef NAVIGATOR_MISSION_BLOCK_H
#define NAVIGATOR_MISSION_BLOCK_H

#include <drivers/drv_hrt.h>

#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>

#include "navigator_mode.h"

class Navigator;

class MissionBlock : public NavigatorMode
{
public:
	/**
	 * Constructor
	 */
	MissionBlock(Navigator *navigator, const char *name);

	/**
	 * Destructor
	 */
	virtual ~MissionBlock();

protected:
	/**
	 * Check if mission item has been reached
	 * @return true if successfully reached
	 */
	bool is_mission_item_reached();
	/**
	 * Reset all reached flags
	 */
	void reset_mission_item_reached();

	/**
	 * Convert a mission item to a position setpoint
	 *
	 * @param the mission item to convert
	 * @param the position setpoint that needs to be set
	 */
	void mission_item_to_position_setpoint(const mission_item_s *item, position_setpoint_s *sp);

	/**
	 * Set previous position setpoint to current setpoint
	 */
	void set_previous_pos_setpoint();

	/**
	 * Set a loiter mission item, if possible reuse the position setpoint, otherwise take the current position
	 */
	void set_loiter_item(struct mission_item_s *item, float min_clearance = -1.0f);

	mission_item_s _mission_item;
	bool _waypoint_position_reached;
	bool _waypoint_yaw_reached;
	hrt_abstime _time_first_inside_orbit;
};

#endif
