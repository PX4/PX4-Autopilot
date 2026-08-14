/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * Mission mode class that handles everything related to executing a mission.
 * This class gets included as one of the 'modes' in the Navigator, along with other
 * modes like RTL, Loiter, etc.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include <cstdint>

#include "mission_base.h"
#include "navigation.h"

class Navigator;

class Mission : public MissionBase
{
public:
	Mission(Navigator *navigator);
	~Mission() = default;

	virtual void on_inactive() override;
	virtual void on_activation() override;

	/**
	 * Set the current mission item.
	 *
	 * @param index Mission sequence to set as current, or -1 to keep the current item unchanged
	 *              (e.g. to reset jump counters in place).
	 * @param reset_jump_counters Reset all DO_JUMP repeat counters and mission completion flag
	 * @return true if the current mission item was set successfully (or the reset-only, unchanged-index
	 *         request could be honored), false if index is out of range or there is no current mission item.
	 */
	bool set_current_mission_index(int32_t index, bool reset_jump_counters = false);

	uint16_t get_land_start_index() const { return _mission.land_start_index; }
	bool get_land_start_available() const { return hasMissionLandStart(); }


private:

	bool setNextMissionItem() override;

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	void onMissionUpdate(bool has_mission_items_changed) override;

	/**
	 * Plan a smart rejoin onto the uploaded route and arm the virtual branch-in waypoint.
	 *
	 * @param resume_mission_on_previous True when the camera-trigger resume logic will rewind the
	 *                                   mission index, in which case joining must not interfere.
	 * @return true if a join route was armed, false to fall back to the legacy direct-to-item behavior.
	 */
	bool trySetRouteJoinOnActivation(bool resume_mission_on_previous);
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE

	/**
	 * Returns true if we need to move to waypoint location after vtol takeoff
	 */
	bool do_need_move_to_takeoff();

	/**
	 * Calculate takeoff height for mission item considering ground clearance
	 */
	float calculate_takeoff_altitude(struct mission_item_s *mission_item);

	/**
	 * Save current mission state to dataman
	 */
	void save_mission_state();

	void setActiveMissionItems() override;

	void handleTakeoff(WorkItemType &new_work_item_type, mission_item_s next_mission_items[], size_t &num_found_items);

	void handleVtolTransition(WorkItemType &new_work_item_type, mission_item_s next_mission_items[],
				  size_t &num_found_items);

	bool _need_mission_save{false};

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
	/**
	 * The last DO_JUMP edge flown in normal mission execution. Nominal progress is already implied
	 * by current_seq; only an active loop must survive the jump so rejoin replans stay anchored on it.
	 */
	mission_route::Segment _last_flown_loop_segment{};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MIS_ROUTE_JOIN>)    _param_mis_route_join,
		(ParamFloat<px4::params::MIS_MC_SEG_DIST>) _param_mis_mc_seg_dist,
		(ParamFloat<px4::params::MIS_FW_SEG_DIST>) _param_mis_fw_seg_dist
	)
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
};
