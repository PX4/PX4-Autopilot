/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
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
	Mission(Navigator *navigator, const char *name);

	virtual ~Mission();

	virtual void on_inactive();

	virtual void on_activation();

	virtual void on_active();

	enum mission_altitude_mode {
		MISSION_ALTMODE_ZOH = 0,
		MISSION_ALTMODE_FOH = 1
	};

	enum mission_yaw_mode {
		MISSION_YAWMODE_NONE = 0,
		MISSION_YAWMODE_FRONT_TO_WAYPOINT = 1,
		MISSION_YAWMODE_FRONT_TO_HOME = 2,
		MISSION_YAWMODE_BACK_TO_HOME = 3,
		MISSION_YAWMODE_MAX = 4
	};

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
	 * Check distance to first waypoint (with lat/lon)
	 * @return true only if it's not too far from home (< MIS_DIST_1WP)
	 */
	bool check_dist_1wp();

	/**
	 * Set new mission items
	 */
	void set_mission_items();

	/**
	 * Returns true if we need to do a takeoff at the current state
	 */
	bool do_need_takeoff();

	/**
	 * Returns true if we need to move to waypoint location before starting descent
	 */
	bool do_need_move_to_land();

	/**
	 * Returns true if we need to move to waypoint location after vtol takeoff
	 */
	bool do_need_move_to_takeoff();

	/**
	 * Copies position from setpoint if valid, otherwise copies current position
	 */
	void copy_positon_if_valid(struct mission_item_s *mission_item, struct position_setpoint_s *setpoint);

	/**
	 * Create mission item to align towards next waypoint
	 */
	void set_align_mission_item(struct mission_item_s *mission_item, struct mission_item_s *mission_item_next);

	/**
	 * Calculate takeoff height for mission item considering ground clearance
	 */
	float calculate_takeoff_altitude(struct mission_item_s *mission_item);

	/**
	 * Updates the heading of the vehicle. Rotary wings only.
	 */
	void heading_sp_update();

	/**
	 * Updates the altitude sp to follow a foh
	 */
	void altitude_sp_foh_update();

	/**
	 * Resets the altitude sp foh logic
	 */
	void altitude_sp_foh_reset();

	float get_absolute_altitude_for_item(struct mission_item_s &mission_item);

	/**
	 * Read the current and the next mission item. The next mission item read is the
	 * next mission item that contains a position.
	 *
	 * @return true if current mission item available
	 */
	bool prepare_mission_items(bool onboard, struct mission_item_s *mission_item,
		struct mission_item_s *next_position_mission_item, bool *has_next_position_item);

	/**
	 * Read current (offset == 0) or a specific (offset > 0) mission item
	 * from the dataman and watch out for DO_JUMPS
	 *
	 * @return true if successful
	 */
	bool read_mission_item(bool onboard, int offset, struct mission_item_s *mission_item);

	/**
	 * Save current offboard mission state to dataman
	 */
	void save_offboard_mission_state();

	/**
	 * Inform about a changed mission item after a DO_JUMP
	 */
	void report_do_jump_mission_changed(int index, int do_jumps_remaining);

	/**
	 * Set a mission item as reached
	 */
	void set_mission_item_reached();

	/**
	 * Set the current offboard mission item
	 */
	void set_current_offboard_mission_item();

	/**
	 * Set that the mission is finished if one exists or that none exists
	 */
	void set_mission_finished();

	/**
	 * Check wether a mission is ready to go
	 */
	bool check_mission_valid();

	/**
	 * Reset offboard mission
	 */
	void reset_offboard_mission(struct mission_s &mission);

	/**
	 * Returns true if we need to reset the mission
	 */
	bool need_to_reset_mission(bool active);

	control::BlockParamInt _param_onboard_enabled;
	control::BlockParamFloat _param_takeoff_alt;
	control::BlockParamFloat _param_dist_1wp;
	control::BlockParamInt _param_altmode;
	control::BlockParamInt _param_yawmode;
	control::BlockParamInt _param_force_vtol;

	struct mission_s _onboard_mission;
	struct mission_s _offboard_mission;

	int _current_onboard_mission_index;
	int _current_offboard_mission_index;
	bool _need_takeoff;					/**< if true, then takeoff must be performed before going to the first waypoint (if needed) */

	enum {
		MISSION_TYPE_NONE,
		MISSION_TYPE_ONBOARD,
		MISSION_TYPE_OFFBOARD
	} _mission_type;

	bool _inited;
	bool _home_inited;
	bool _need_mission_reset;

	MissionFeasibilityChecker _missionFeasibilityChecker; /**< class that checks if a mission is feasible */

	float _min_current_sp_distance_xy; /**< minimum distance which was achieved to the current waypoint  */
	float _mission_item_previous_alt; /**< holds the altitude of the previous mission item,
					    can be replaced by a full copy of the previous mission item if needed */
	float _on_arrival_yaw; /**< holds the yaw value that should be applied when the current waypoint is reached */
	float _distance_current_previous; /**< distance from previous to current sp in pos_sp_triplet,
					    only use if current and previous are valid */

	enum work_item_type {
		WORK_ITEM_TYPE_DEFAULT,		/**< default mission item */
		WORK_ITEM_TYPE_TAKEOFF,		/**< takeoff before moving to waypoint */
		WORK_ITEM_TYPE_MOVE_TO_LAND,	/**< move to land waypoint before descent */
		WORK_ITEM_TYPE_ALIGN,		/**< align for next waypoint */
		WORK_ITEM_TYPE_CMD_BEFORE_MOVE,	/**<  */
		WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF,	/**<  */
		WORK_ITEM_TYPE_TRANSITON_BEFORE_LAND,	/**<  */
		WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION	/**<  */
	} _work_item_type;	/**< current type of work to do (sub mission item) */

};

#endif
