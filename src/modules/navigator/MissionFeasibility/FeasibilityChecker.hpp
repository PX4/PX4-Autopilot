/****************************************************************************
 *
 *   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
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

#pragma once

#include "../navigation.h"
#include <mathlib/mathlib.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/Subscription.hpp>
#include <px4_platform_common/module_params.h>

class FeasibilityChecker : public ModuleParams
{
public:

	FeasibilityChecker();

	enum class VehicleType {
		RotaryWing,
		FixedWing,
		Vtol,
		Other
	};

	/**
	 * @brief Run validity checks for mission item
	 *
	 *
	 * @param mission_item The mission item to run the checks on
	 * @param current_index The index of the current mission item in the mission
	 * @param total_count The total number of mission items in the mission
	 * @return False if fatal error occured and no other checks should be run.
	*/
	bool processNextItem(mission_item_s &mission_item, const int current_index, const int total_count);

	void setMavlinkLogPub(orb_advert_t *mavlink_log_pub)
	{
		_mavlink_log_pub = mavlink_log_pub;
	}

	/**
	 * @return True At least one check failed.
	*/
	bool someCheckFailed()
	{
		return _takeoff_failed ||
		       _distance_first_waypoint_failed ||
		       _distance_between_waypoints_failed ||
		       _land_pattern_validity_failed ||
		       _fixed_wing_land_approach_failed ||
		       _below_home_alt_failed ||
		       _mission_validity_failed ||
		       _takeoff_land_available_failed;
	}

	/**
	 * @brief Reset all data
	*/
	void reset();

private:
	orb_advert_t *_mavlink_log_pub{nullptr};

	uORB::Subscription _home_pos_sub{ORB_ID(home_position)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_detector_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};

	// parameters
	float _param_fw_lnd_ang{0.f};
	float _param_mis_dist_1wp{0.f};
	float _param_nav_acc_rad{0.f};
	int32_t _param_mis_takeoff_land_req{0};

	bool _is_landed{false};
	float _home_alt_msl{NAN};
	matrix::Vector2d _home_lat_lon = matrix::Vector2d((double)NAN, (double)NAN);
	matrix::Vector2d _current_position_lat_lon = matrix::Vector2d((double)NAN, (double)NAN);
	VehicleType _vehicle_type{VehicleType::RotaryWing};

	// internal flags to keep track of which checks failed
	bool _mission_validity_failed{false};
	bool _takeoff_failed{false};
	bool _land_pattern_validity_failed{false};
	bool _distance_first_waypoint_failed{false};
	bool _distance_between_waypoints_failed{false};
	bool _below_home_alt_failed{false};
	bool _fixed_wing_land_approach_failed{false};
	bool _takeoff_land_available_failed{false};
	bool _items_fit_to_vehicle_type_failed{false};

	// internal checkTakeoff related variables
	bool _found_item_with_position{false};
	bool _has_vtol_takeoff{false};
	bool _has_takeoff{false};

	// internal checkFixedWingLanding related variables
	bool _landing_valid{false};
	int _do_land_start_index{-1};
	int _landing_approach_index{-1};
	mission_item_s _mission_item_previous = {};

	// internal checkHorizontalDistanceToFirstWaypoint variables
	bool _first_waypoint_found{false};

	// internal checkDistancesBetweenWaypoints variables
	double _last_lat{(double)NAN};
	double _last_lon{(double)NAN};
	int _last_cmd{-1};

	/**
	 * @brief Update data from external topics, e.g home position
	*/
	void updateData();

	// methods which are called for each mission item

	/**
	 * @brief Check general mission item validity, e.g. supported commands.
	 *
	 * @param mission_item The current mission item
	 * @return False if the check failed.
	*/
	bool checkMissionItemValidity(mission_item_s &mission_item, const int current_index);

	/**
	 * @brief Check if takeoff is valid
	 *
	 * @param mission_item The current mission item
	 * @return False if the check failed.
	*/
	bool checkTakeoff(mission_item_s &mission_item);

	/**
	 * @brief Check if the mission items fit to the vehicle type
	 *
	 * @param mission_item The current mission item
	 * @return False if the check failed.
	*/
	bool checkItemsFitToVehicleType(const mission_item_s &mission_item);

	/**
	 * @brief Check validity of landing pattern (fixed wing & vtol)
	 *
	 * @param mission_item The current mission item
	 * @param current_index The current mission index
	 * @param last_index The last index of the mission
	 * @return False if the check failed.
	*/
	bool checkLandPatternValidity(mission_item_s &mission_item, const int current_index, const int last_index);

	/**
	 * @brief Check distance to first waypoint from current vehicle position (if available).
	 *
	 * @param mission_item The current mission item
	 * @return False if the check failed.
	*/
	bool checkHorizontalDistanceToFirstWaypoint(mission_item_s &mission_item);

	/**
	 * @brief Check distances between waypoints
	 *
	 * @param mission_item The current mission item
	 * @return False if the check failed.
	*/
	bool checkDistancesBetweenWaypoints(const mission_item_s &mission_item);

	/**
	 * @brief Check if any waypoint is below the home altitude. Issues warning only.
	 *
	 * @param mission_item The current mission item
	 * @param current_index The current mission index
	 * @return Always returns true, only issues warning.
	*/
	bool checkIfBelowHomeAltitude(const mission_item_s &mission_item, const int current_index);

	/**
	 * @brief Check fixed wing land approach (fixed wing only)
	 *
	 * @param mission_item The current mission item
	 * @param current_index The current mission index
	 * @return False if the check failed.
	*/
	bool checkFixedWindLandApproach(mission_item_s &mission_item, const int current_index);

	// methods which are called once at the end
	/**
	 * @brief Check if takeoff/landing are available according to MIS_TKO_LAND_REQ parameter
	 *
	 * @return False if the check failed.
	*/
	bool checkTakeoffLandAvailable();

	/**
	 * @brief Run checks which are common for all vehicle types
	 *
	 * @param mission_item The current mission item
	 * @param current_index The current mission index
	*/
	void doCommonChecks(mission_item_s &mission_item, const int current_index);

	/**
	 * @brief Run checks which are only related to VTOL vehicles.
	 *
	 * @param mission_item The current mission item
	 * @param current_index The current mission index
	*/
	void doVtolChecks(mission_item_s &mission_item, const int current_index, const int last_index);

	/**
	 * @brief Run checks which are only related to fixed wing vehicles.
	 *
	 * @param mission_item The current mission item
	 * @param current_index The current mission index
	 * @param last_index The last mission index
	 * @return False if the check failed.
	*/
	void doFixedWingChecks(mission_item_s &mission_item, const int current_index, const int last_index);

	/**
	 * @brief Run checks which are only related to multirotor vehicles.
	 *
	 * @param mission_item The current mission item
	 * @param current_index The current mission index
	 * @return False if the check failed.
	*/
	void doMulticopterChecks(mission_item_s &mission_item, const int current_index);
};
