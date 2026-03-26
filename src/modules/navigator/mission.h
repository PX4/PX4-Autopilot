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
	virtual void on_active() override;

	bool set_current_mission_index(uint16_t index);

	uint16_t get_land_start_index() const { return _mission.land_start_index; }
	bool get_land_start_available() const { return hasMissionLandStart(); }


private:

	bool setNextMissionItem() override;

	/**
	 * Returns true if we need to move to waypoint location after vtol takeoff
	 */
	bool do_need_move_to_takeoff();

	void updateMissionRouteCache();
	void syncMissionRouteState();

	/**
	 * Save current mission state to dataman
	 */
	void save_mission_state();

	void setActiveMissionItems() override;

	void handleTakeoff(WorkItemType &new_work_item_type, mission_item_s next_mission_items[], size_t &num_found_items);

	void handleVtolTransition(WorkItemType &new_work_item_type, mission_item_s next_mission_items[],
				  size_t &num_found_items);

protected:
	bool trySetRouteJoinOnActivation(bool resume_mission_on_previous);

private:
	bool _need_mission_save{false};
	MissionRoutePlanner::Segment _last_flown_loop_segment{};
	uint32_t _route_state_mission_id{0};
	int32_t _route_state_mission_count{0};
	uint8_t _route_state_mission_dataman_id{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MIS_ROUTE_JOIN>)     _param_mis_route_join,
		(ParamFloat<px4::params::MIS_MC_SEG_DIST>) _param_mis_mc_seg_dist,
		(ParamFloat<px4::params::MIS_FW_SEG_DIST>) _param_mis_fw_seg_dist
	)
};
