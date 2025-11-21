/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file mission_base.h
 *
 * Mission base mode class that can be used for modes interacting with a mission.
 *
 */

#pragma once

#include <cstdint>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <dataman_client/DatamanClient.hpp>
#include <uORB/topics/geofence_status.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/navigator_mission_item.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>

#include "mission_block.h"
#include "navigation.h"

using namespace time_literals;

class Navigator;

class MissionBase : public MissionBlock, public ModuleParams
{
public:
	MissionBase(Navigator *navigator, int32_t dataman_cache_size_signed, uint8_t navigator_state_id);
	~MissionBase() override = default;

	virtual void on_inactive() override;
	virtual void on_inactivation() override;
	virtual void on_activation() override;
	virtual void on_active() override;

	virtual bool isLanding();

protected:

	/**
	 * @brief Maximum time to wait for dataman loading
	 *
	 */
	static constexpr hrt_abstime MAX_DATAMAN_LOAD_WAIT{500_ms};

	// Work Item corresponds to the sub-mode set on the "MAV_CMD_DO_SET_MODE" MAVLink message
	enum class WorkItemType {
		WORK_ITEM_TYPE_DEFAULT,		/**< default mission item */
		WORK_ITEM_TYPE_CLIMB,		/**< takeoff before moving to waypoint */
		WORK_ITEM_TYPE_MOVE_TO_LAND,	/**< move to land waypoint before descent */
		WORK_ITEM_TYPE_ALIGN_HEADING,		/**< align for next waypoint */
		WORK_ITEM_TYPE_TRANSITION_AFTER_TAKEOFF,
		WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION,
		WORK_ITEM_TYPE_PRECISION_LAND
	} _work_item_type{WorkItemType::WORK_ITEM_TYPE_DEFAULT};	/**< current type of work to do (sub mission item) */

	enum class MissionType {
		MISSION_TYPE_NONE,
		MISSION_TYPE_MISSION
	} _mission_type{MissionType::MISSION_TYPE_NONE};

	/**
	 * @brief Get the Previous Mission Position Items
	 *
	 * @param[in] start_index is the index from where to start searching the previous mission position items
	 * @param[out] items_index is an array of indexes indicating the previous mission position items found
	 * @param[out] num_found_items are the amount of previous position items found
	 * @param[in] max_num_items are the maximum amount of previous position items to be searched
	 */
	void getPreviousPositionItems(int32_t start_index, int32_t items_index[], size_t &num_found_items,
				      uint8_t max_num_items);
	/**
	 * @brief Get the next mission item containing a position setpoint
	 *
	 * @param[in] start_index is the index from where to start searching (first possible return index)
	 * @param[out] items_index is an array of indexes indicating the next mission position items found
	 * @param[out] num_found_items are the amount of next position items found
	 * @param[in] max_num_items are the maximum amount of next position items to be searched
	 */
	void getNextPositionItems(int32_t start_index, int32_t items_index[], size_t &num_found_items,
				  uint8_t max_num_items);
	/**
	 * @brief Mission has a land start, a land, and is valid
	 *
	 * @return true If mission has a land start and a land item and is valid
	 * @return false otherwise
	 */
	bool hasMissionLandStart() const { return _mission.land_start_index >= 0 && _mission.land_index >= 0 && isMissionValid();};
	/**
	 * @brief Go to next Mission Item
	 * Go to next non jump mission item
	 *
	 * @param[in] execute_jump Flag indicating if a jump should be executed or ignored
	 * @return PX4_OK if next mission item exists, PX4_ERR otherwise
	 */
	int goToNextItem(bool execute_jump);
	/**
	 * @brief Go to previous Mission Item
	 * Go to previous non jump mission item
	 * @param[in] execute_jump Flag indicating if a jump should be executed or ignored
	 * @return PX4_OK if previous mission item exists, PX4_ERR otherwise
	 */
	int goToPreviousItem(bool execute_jump);
	/**
	 * @brief Go to Mission Item
	 *
	 * @param[in] index Index of the mission item to go to
	 * @param[in] execute_jump Flag indicating if a jump should be executed of ignored
	 * @param[in] mission_direction_backward Flag indicating if a mission is flown backward
	 * @return PX4_OK if the mission item exists, PX4_ERR otherwise
	 */
	int goToItem(int32_t index, bool execute_jump, bool mission_direction_backward = false);
	/**
	 * @brief Go To Previous Mission Position Item
	 *
	 * @param[in] execute_jump Flag indicating if a jump should be executed or ignored
	 * @return PX4_OK if previous mission item exists, PX4_ERR otherwise
	 */
	int goToPreviousPositionItem(bool execute_jump);
	/**
	 * @brief Go To Next Mission Position Item
	 *
	 * @param[in] execute_jump Flag indicating if a jump should be executed or ignored
	 * @return PX4_OK if next mission item exists, PX4_ERR otherwise
	 */
	int goToNextPositionItem(bool execute_jump);
	/**
	 * @brief Go to Mission Land Start Item
	 *
	 * @return PX4_OK if land start item exists and is loaded, PX4_ERR otherwise
	 */
	int goToMissionLandStart();
	/**
	 * @brief Set the Mission to closest mission position item from current position
	 *
	 * @param[in] lat latitude of the current position
	 * @param[in] lon longitude of the current position
	 * @param[in] alt altitude of the current position
	 * @param[in] home_alt altitude of the home position
	 * @param[in] vehicle_status vehicle status struct
	 * @return PX4_OK if closest item is found and loaded, PX4_ERR otherwise
	 */
	int setMissionToClosestItem(double lat, double lon, float alt, float home_alt, const vehicle_status_s &vehicle_status);
	/**
	 * @brief Initialize Mission
	 *
	 * @return PX4_OK if mission could be loaded, PX4_ERR otherwise
	 */
	int initMission();
	/**
	 * @brief Reset Mission
	 *
	 */
	void resetMission();
	/**
	 * @brief Reset Mission Jump Counter of Mission Jump Items
	 *
	 */
	void resetMissionJumpCounter();
	/**
	 * @brief Get the Non Jump Mission Item
	 *
	 * @param[out] mission_index Index of the mission item
	 * @param[out] mission The return mission item
	 * @param execute_jump Flag indicating if a jump item should be executed or ignored
	 * @param write_jumps Flag indicating if the jump counter should be updated
	 * @param mission_direction_backward Flag indicating if the mission is flown backwards
	 * @return PX4_OK if mission item could be loaded, PX4_ERR otherwise
	 */
	int getNonJumpItem(int32_t &mission_index, mission_item_s &mission, bool execute_jump, bool write_jumps,
			   bool mission_direction_backward = false);
	/**
	 * @brief Is Mission Valid
	 *
	 * @return true is mission is valid
	 * @return false otherwise
	 */
	bool isMissionValid() const;

	/**
	 * @brief Check whether a mission is ready to go
	 * @param[in] forced flag if the check has to be run irregardles of any updates.
	 */
	void check_mission_valid(bool forced = false);

	/**
	 * On mission update
	 * Change behaviour after external mission update.
	 * @param[in] has_mission_items_changed flag if the mission items have been changed.
	 */
	void onMissionUpdate(bool has_mission_items_changed);

	/**
	 * Update mission topic
	 */
	void update_mission();

	/**
	 * Move on to next mission item or switch to loiter
	 */
	void advance_mission();

	/**
	 * @brief Configures mission items in current setting
	 *
	 * Configure the mission items depending on current mission item index and settings such
	 * as terrain following, etc.
	 */
	void set_mission_items();

	/**
	 * @brief Load current mission item
	 *
	 * Load current mission item from dataman cache.
	 * @return true, if the mission item could be loaded, false otherwise
	 */
	bool loadCurrentMissionItem();

	/**
	 * Set the mission result
	 */
	void set_mission_result();

	/**
	 * @brief Reset the item cache
	 */
	void resetItemCache();

	/**
	 * @brief Set the actions to be performed on Active Mission Item
	 *
	 */
	virtual void setActiveMissionItems() = 0;
	/**
	 * @brief Set the Next Mission Item after old mission item has been completed
	 *
	 * @return true if the next mission item could be set
	 * @return false otherwise
	 */
	virtual bool setNextMissionItem() = 0;
	/**
	 * @brief Set action at the end of the mission
	 *
	 */
	void setEndOfMissionItems();
	/**
	 * @brief Publish navigator mission item
	 *
	 */
	void publish_navigator_mission_item();

	/**
	 * @brief Do need move to item
	 *
	 * @return true if the item is horizontally further away than the mission item
	 * @return false otherwise
	 */
	bool do_need_move_to_item();

	/**
	 * @brief Handle landing
	 *
	 * @param new_work_item_type new work item type state machine to be set
	 * @param next_mission_items the next mission items after the current mission item
	 * @param num_found_items number of found next mission items
	 */
	void handleLanding(WorkItemType &new_work_item_type, mission_item_s next_mission_items[],
			   size_t &num_found_items);
	/**
	 * @brief I position setpoint equal
	 *
	 * @param p1 First position setpoint to compare
	 * @param p2 Second position setpoint to compare
	 * @return true if both setpoints are equal
	 * @return false otherwise
	 */
	bool position_setpoint_equal(const position_setpoint_s *p1, const position_setpoint_s *p2) const;

	/**
	 * @brief Set the Mission Index
	 *
	 * @param[in] index Index of the mission item
	 */
	void setMissionIndex(int32_t index);


	bool _is_current_planned_mission_item_valid{false};	/**< Flag indicating if the currently loaded mission item is valid*/
	bool _mission_has_been_activated{false};		/**< Flag indicating if the mission has been activated*/
	bool _mission_checked{false};				/**< Flag indicating if the mission has been checked by the mission validator*/
	bool _system_disarmed_while_inactive{false};		/**< Flag indicating if the system has been disarmed while mission is inactive*/
	mission_s _mission;					/**< Currently active mission*/
	float _mission_init_climb_altitude_amsl{NAN}; 		/**< altitude AMSL the vehicle will climb to when mission starts */
	int _inactivation_index{-1}; // index of mission item at which the mission was paused. Used to resume survey missions at previous waypoint to not lose images.
	int _mission_activation_index{-1};					/**< Index of the mission item that will bring the vehicle back to a mission waypoint */
	bool _speed_replayed_on_activation{false};			/**< Flag indicating if the speed change items have been replayed on activation */

	int32_t _load_mission_index{-1}; /**< Mission inted of loaded mission items in dataman cache*/
	int32_t _dataman_cache_size_signed; /**< Size of the dataman cache. A negativ value indicates that previous mission items should be loaded, a positiv value the next mission items*/

	DatamanCache _dataman_cache{"mission_dm_cache_miss", 10}; /**< Dataman cache of mission items*/
	DatamanClient	&_dataman_client = _dataman_cache.client(); /**< Dataman client*/

	uORB::Subscription _mission_sub{ORB_ID(mission)};	/**< mission subscription*/
	uORB::SubscriptionData<vehicle_land_detected_s> _land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};	/**< vehicle status subscription */
	uORB::SubscriptionData<vehicle_global_position_s> _global_pos_sub{ORB_ID(vehicle_global_position)};	/**< global position subscription */
	uORB::Publication<navigator_mission_item_s> _navigator_mission_item_pub{ORB_ID::navigator_mission_item}; /**< Navigator mission item publication*/
	uORB::Publication<mission_s> _mission_pub{ORB_ID(mission)}; /**< Mission publication*/
private:
	/**
	 * @brief Maximum number of jump mission items iterations
	 *
	 */
	static constexpr uint16_t MAX_JUMP_ITERATION{10u};
	/**
	 * @brief Update Dataman cache
	 *
	 */
	virtual void updateDatamanCache();
	/**
	 * @brief Update mission subscription
	 *
	 */
	void updateMavlinkMission();

	/**
	 * Reset mission
	 */
	void checkMissionRestart();

	/**
	 * Set a mission item as reached
	 */
	void set_mission_item_reached();

	/**
	 * Updates the heading of the vehicle. Rotary wings only.
	 */
	void heading_sp_update();

	/**
	 * Abort landing
	 */
	void do_abort_landing();

	/**
	 * Inform about a changed mission item after a DO_JUMP
	 */
	void report_do_jump_mission_changed(int index, int do_jumps_remaining);

	/**
	 * @brief Cache the mission items containing gimbal, camera mode and trigger commands
	 *
	 * @param mission_item The mission item to cache if applicable
	 */
	void cacheItem(const mission_item_s &mission_item);

	/**
	 * @brief Update the cached items up to the given index
	 *
	 * @param end_index The index to update up to
	 */
	void updateCachedItemsUpToIndex(int end_index);

	/**
	 * @brief Replay the cached gimbal items
	 */
	void replayCachedGimbalItems();

	/**
	 * @brief Replay the cached camera mode items
	 */
	void replayCachedCameraModeItems();

	/**
	 * @brief Replay the cached trigger items
	 *
	 */
	void replayCachedTriggerItems();

	/**
	 * @brief Replay the cached speed change items and delete them afterwards
	 *
	 */
	void replayCachedSpeedChangeItems();

	/**
	 * @brief Check if there are cached gimbal items to be replayed
	 *
	 * @return true if there are cached items
	 */
	bool haveCachedGimbalItems();

	/**
	 * @brief Check if there are cached camera mode items to be replayed
	 *
	 * @return true if there are cached items
	 */
	bool haveCachedCameraModeItems();

	/**
	 * @brief Check if the camera was triggering
	 *
	 * @return true if there was a camera trigger command in the cached items that didn't disable triggering
	 */
	bool cameraWasTriggering();

	/**
	 * @brief Parameters update
	 *
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update();

	/**
	 * @brief Check if a climb is necessary to align with mission altitude prior to starting the mission
	 *
	 * @param mission_item_index The index of the mission item to check if a climb is necessary
	 */
	void checkClimbRequired(int32_t mission_item_index);

	/**
	 * @brief check if relevant data in the new mission have changed.
	 * @param[in] new_mission new mission received over uorb
	 * @return true if the relevant mission data has changed, false otherwise
	 */
	bool checkMissionDataChanged(mission_s new_mission);

	/**
	 * @brief update current mission altitude after the home position has changed.
	 */

	void updateMissionAltAfterHomeChanged();

	bool canRunMissionFeasibility();

	uint32_t _home_update_counter = 0; /**< Variable to store the previous value for home change detection.*/

	bool _align_heading_necessary{false}; // if true, heading of vehicle needs to be aligned with heading of next waypoint. Used to create new mission items for heading alignment.

	mission_item_s _last_gimbal_configure_item {};
	mission_item_s _last_gimbal_control_item {};
	mission_item_s _last_camera_mode_item {};
	mission_item_s _last_camera_trigger_item {};
	mission_item_s _last_speed_change_item {};

	DEFINE_PARAMETERS_CUSTOM_PARENT(
		ModuleParams,
		(ParamInt<px4::params::MIS_MNT_YAW_CTL>) _param_mis_mnt_yaw_ctl
	)

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionData<geofence_status_s> _geofence_status_sub{ORB_ID(geofence_status)};
};
