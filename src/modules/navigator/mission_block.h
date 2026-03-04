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
 * Base class for Mission class and special flight modes like
 * RTL, Land, Loiter, Takeoff, Geofence, etc.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include "navigator_mode.h"
#include "navigation.h"

#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/mission.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vtol_vehicle_status.h>

// cosine of maximal course error to exit loiter if exit course is enforced (fixed-wing only)
static constexpr float kCosineExitCourseThreshold = 0.99619f; // cos(5°)

class Navigator;

class MissionBlock : public NavigatorMode
{
public:
	/**
	 * Constructor
	 */
	MissionBlock(Navigator *navigator, uint8_t navigator_state_id);
	virtual ~MissionBlock() = default;

	MissionBlock(const MissionBlock &) = delete;
	MissionBlock &operator=(const MissionBlock &) = delete;

	void initialize() override;

	/**
	 * Check if the mission item contains a navigation position
	 *
	 * @return false if the mission item does not contain a valid position
	 */
	static bool item_contains_position(const mission_item_s &item);

	/**
	 * Returns true if the mission item is not an instant action, but has a delay / timeout
	 */
	bool item_has_timeout(const mission_item_s &item);

	/**
	 * Check if the mission item contains a gate condition
	 *
	 * @return true if mission item is a gate
	 */
	static bool item_contains_gate(const mission_item_s &item);

	/**
	 * Get the absolute altitude for mission item
	 *
	 * @param mission_item	the mission item of interest
	 * @param home_alt	the home altitude in [m AMSL].
	 * @return Mission item altitude in [m AMSL]
	 */
	static float get_absolute_altitude_for_item(const  mission_item_s &mission_item, float home_alt);

	/**
	 * Check if the mission item contains a marker
	 *
	 * @return true if mission item is a marker
	 */
	static bool item_contains_marker(const mission_item_s &item);

	/**
	 * Set the item_has_timeout() command timeout
	 *
	 * @param timeout Timeout in seconds
	 */
	void set_command_timeout(const float timeout) { _command_timeout = timeout; }

	/**
	 * Copies position from setpoint if valid, otherwise copies current position
	 */
	void copy_position_if_valid(struct mission_item_s *const mission_item,
				    const struct position_setpoint_s *const setpoint) const;

	/**
	 * Create mission item to align towards next waypoint
	 */
	void set_align_mission_item(struct mission_item_s *const mission_item,
				    const struct mission_item_s *const mission_item_next) const;

	void updateFailsafeChecks() override;

protected:
	/**
	 * @brief heading mode for setting navigation items
	 *
	 */
	enum class HeadingMode {
		NAVIGATION_HEADING = 0,
		DESTINATION_HEADING,
		CURRENT_HEADING,
	};
	/**
	 * Check if mission item has been reached (for Waypoint based mission items) or Completed (Action based mission items)
	 *
	 * Mission Item's 'nav_cmd' can be either Waypoint or Action based. In order to check whether current mission item's
	 * execution was successful, we need to check either the waypoint was 'reached' or the action was 'completed'.
	 *
	 * @return true if successfully reached or completed
	 */
	bool is_mission_item_reached_or_completed();

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
	bool mission_item_to_position_setpoint(const mission_item_s &item, position_setpoint_s *sp);

	void setLoiterItemFromCurrentPositionSetpoint(struct mission_item_s *item);

	void setLoiterItemFromCurrentPosition(struct mission_item_s *item);
	void setLoiterItemFromCurrentPositionWithBraking(struct mission_item_s *item);

	void setLoiterItemCommonFields(struct mission_item_s *item);

	/**
	 * Set a takeoff mission item
	 */
	void set_takeoff_item(struct mission_item_s *item, float abs_altitude);

	/**
	 * Set a land mission item
	 */
	void set_land_item(struct mission_item_s *item);

	/**
	 * Set idle mission item
	 */
	void set_idle_item(struct mission_item_s *item);

	/**
	 * Set vtol transition item
	 */
	void set_vtol_transition_item(struct mission_item_s *item, const uint8_t new_mode);

	void setLoiterToAltMissionItem(mission_item_s &item, const PositionYawSetpoint &pos_yaw_sp, float loiter_radius) const;

	void setLoiterHoldMissionItem(mission_item_s &item, const PositionYawSetpoint &pos_yaw_sp, float loiter_time,
				      float loiter_radius) const;

	void setMoveToPositionMissionItem(mission_item_s &item, const PositionYawSetpoint &pos_yaw_sp) const;

	void setLandMissionItem(mission_item_s &item, const PositionYawSetpoint &pos_yaw_sp) const;

	void startPrecLand(uint16_t land_precision);
	void updateAltToAvoidTerrainCollisionAndRepublishTriplet(mission_item_s mission_item);

	/**
	 * @brief Issue a command for mission items with a nav_cmd that specifies an action
	 *
	 * Execute the specified command inside the mission item. The action depends on the nav_cmd
	 * value (which correlates to the MAVLink's MAV_CMD enum values) and the params defined in the
	 * mission item.
	 *
	 * This is used for commands like MAV_CMD_DO* in MAVLink, where immediate actions are defined.
	 * For more information, refer to: https://mavlink.io/en/services/mission.html#mavlink_commands
	 *
	 * @param item Mission item to execute
	 */
	void issue_command(const mission_item_s &item);

	/**
	 * [s] Get the time to stay that's specified in the mission item
	 */
	float get_time_inside(const mission_item_s &item) const;

	float get_absolute_altitude_for_item(const mission_item_s &mission_item) const;

	mission_item_s _mission_item{}; // Current mission item that is being executed

	bool _waypoint_position_reached{false};
	bool _waypoint_yaw_reached{false};

	hrt_abstime _time_wp_reached{0};

	// Mission items that have a timeout to allow the payload e.g. gripper, winch, gimbal executing the command see item_has_timeout()
	hrt_abstime _timestamp_command_timeout{0}; ///< Timestamp when the current item_has_timeout() command was started
	float _command_timeout{0.f}; ///< Time in seconds any item_has_timeout() command should be waited for before continuing the mission

private:
	void updateMaxHaglFailsafe();
};
