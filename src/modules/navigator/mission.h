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
 * modes like Return, Loiter, etc.
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

	/**
	 * Returns true if we need to move to waypoint location after vtol takeoff
	 */
	bool do_need_move_to_takeoff(const mission_item_s &takeoff_item);

	/**
	 * Calculate takeoff height for mission item considering ground clearance
	 */
	float calculate_takeoff_altitude(struct mission_item_s *mission_item);

	/**
	 * Save current mission state to dataman
	 */
	void save_mission_state();

	void setActiveMissionItems() override;

	bool isCurrentItemReached() override;

	void handleTakeoff(WorkItemType &new_work_item_type, mission_item_s next_mission_items[], size_t &num_found_items);

	/** Keep the previous setpoint, send the command of _mission_item if it has one, set the work item and publish. */
	void publishActiveItem(const position_setpoint_s &current_setpoint_copy, WorkItemType new_work_item_type);

	/** Start the transition in _vtol_transition.command, first aligning in hover towards alignment_target (copied). */
	void startVtolTransition(const mission_item_s *alignment_target, WorkItemType completion_work);
	/** Apply the transition rules for this cycle; return true when the item is done. */
	bool updateVtolTransition();
	/** Publish the alignment item (aligning) or send the transition command. */
	void setVtolTransitionStep(bool aligning);
	bool vtolTransitionActive() const { return _vtol_transition.command.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION; }
	void resetVtolTransition() { _vtol_transition.command.nav_cmd = NAV_CMD_IDLE; }

	struct VtolTransitionContext {
		mission_item_s command{}; // Transition command, kept while _mission_item is the alignment item.
		mission_item_s target{}; // Next position item or takeoff item to align towards, empty if there is none.
		WorkItemType completion_work{WorkItemType::WORK_ITEM_TYPE_DEFAULT}; // Work item once commanded.
		bool target_is_next{false}; // The target directly follows the item, so a front transition can complete early.
	} _vtol_transition{};

	// Item of a front transition that completed when it started, and the item it went on to, until fixed-wing.
	int32_t _front_transition_index{-1};
	int32_t _front_transition_next_index{-1};
	uint32_t _front_transition_mission_id{0};

	bool _need_mission_save{false};
};
