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
 * @file mission.h
 *
 * Navigator mode to access missions
 *
 * @author Julian Oes <julian@oes.ch>
 */

#ifndef NAVIGATOR_MISSION_H
#define NAVIGATOR_MISSION_H

#include <drivers/drv_hrt.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <dataman/dataman.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>

#include "navigator_mode.h"
#include "mission_block.h"
#include "mission_feasibility_checker.h"

class Navigator;

class Mission : public MissionBlock
{
public:
	/**
	 * Constructor
	 */
	Mission(Navigator *navigator, const char *name);

	/**
	 * Destructor
	 */
	virtual ~Mission();

	/**
	 * This function is called while the mode is inactive
	 */
	virtual void on_inactive();

	/**
	 * This function is called while the mode is active
	 */
	virtual bool on_active(struct position_setpoint_triplet_s *pos_sp_triplet);

private:
	/**
	 * Update onboard mission topic
	 */
	void update_onboard_mission();

	/**
	 * Update offboard mission topic
	 */
	void update_offboard_mission();

	/**
	 * Move on to next mission item or switch to loiter
	 */
	void advance_mission();

	/**
	 * Set new mission items
	 */
	void set_mission_items(struct position_setpoint_triplet_s *pos_sp_triplet);

	/**
	 * Try to set the current position setpoint from an onboard mission item
	 * @return true if mission item successfully set
	 */
	bool is_current_onboard_mission_item_set(struct position_setpoint_s *current_pos_sp);

	/**
	 * Try to set the current position setpoint from an offboard mission item
	 * @return true if mission item successfully set
	 */
	bool is_current_offboard_mission_item_set(struct position_setpoint_s *current_pos_sp);

	/**
	 * Try to set the next position setpoint from an onboard mission item
	 */
	void get_next_onboard_mission_item(struct position_setpoint_s *next_pos_sp);

	/**
	 * Try to set the next position setpoint from an offboard mission item
	 */
	void get_next_offboard_mission_item(struct position_setpoint_s *next_pos_sp);

	/**
	 * Read a mission item from the dataman and watch out for DO_JUMPS
	 * @return true if successful
	 */
	bool read_mission_item(const dm_item_t dm_item, bool is_current, int *mission_index,
			       struct mission_item_s *new_mission_item);

	/**
	 * Report that a mission item has been reached
	 */
	void report_mission_item_reached();

	/**
	 * Rport the current mission item
	 */
	void report_current_offboard_mission_item();

	/**
	 * Report that the mission is finished if one exists or that none exists
	 */
	void report_mission_finished();

	/**
	 * Publish the mission result so commander and mavlink know what is going on
	 */
	void publish_mission_result();

	control::BlockParamFloat _param_onboard_enabled;

	struct mission_s _onboard_mission;
	struct mission_s _offboard_mission;

	int _current_onboard_mission_index;
	int _current_offboard_mission_index;

	orb_advert_t _mission_result_pub;
	struct mission_result_s _mission_result;

	enum {
		MISSION_TYPE_NONE,
		MISSION_TYPE_ONBOARD,
		MISSION_TYPE_OFFBOARD
	} _mission_type;

	MissionFeasibilityChecker missionFeasiblityChecker; /**< class that checks if a mission is feasible */
};

#endif
