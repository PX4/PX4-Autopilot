/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

	void processNextItem(mission_item_s &mission_item, const int current_index, const int total_count);

	void setMavlinkLogPub(orb_advert_t *mavlink_log_pub)
	{
		_mavlink_log_pub = mavlink_log_pub;
	}

	bool someCheckFailed()
	{
		return _takeoff_failed ||
		       _distance_first_waypoint_failed ||
		       _distance_between_waypoints_failed ||
		       _fixed_wing_landing_failed ||
		       _fixed_wing_land_approach_failed ||
		       _below_home_alt_failed ||
		       _mission_validity_failed ||
		       _takeoff_land_available_failed;
	}

	void reset();

private:
	orb_advert_t *_mavlink_log_pub{nullptr};

	uORB::Subscription _home_pos_sub{ORB_ID(home_position)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_detector_sub{ORB_ID(vehicle_land_detected)};

	// parameters
	float _param_fw_lnd_ang{0.f};
	float _param_mis_dist_1wp{0.f};
	float _param_mis_dist_wps{0.f};
	float _param_nav_acc_rad{0.f};
	int _param_mis_takeoff_land_req{0};

	bool _is_landed{false};
	float _home_alt_msl{NAN};
	matrix::Vector2d _home_lat_lon = matrix::Vector2d(NAN, NAN);
	bool _is_vtol{false};
	VehicleType _vehicle_type{VehicleType::RotaryWing};

	// internal flags to keep track of which checks failed
	bool _mission_validity_failed{false};
	bool _takeoff_failed{false};
	bool _fixed_wing_landing_failed{false};
	bool _distance_first_waypoint_failed{false};
	bool _distance_between_waypoints_failed{false};
	bool _below_home_alt_failed{false};
	bool _fixed_wing_land_approach_failed{false};
	bool _takeoff_land_available_failed{false};

	// internal checkTakeoff related variables
	bool _found_item_with_position{false};
	bool _has_vtol_takeoff{false};
	bool _has_takeoff{false};

	// internal checkFixedWingLanding related variables
	bool _landing_valid{false};
	int _do_land_start_index{-1};
	int _landing_approach_index{-1};
	mission_item_s _mission_item_previous = {};

	// internal checkDistanceToFirstWaypoint variables
	bool _first_waypoint_found{false};

	// internal checkDistancesBetweenWaypoints variables
	double _last_lat{NAN};
	double _last_lon{NAN};
	int _last_cmd{-1};

	void updateData();

	// methods which are called for each mission item
	bool checkMissionItemValidity(mission_item_s &mission_item, const int current_index);
	bool checkTakeoff(mission_item_s &mission_item);
	bool checkFixedWingLanding(mission_item_s &mission_item, const int current_index);
	bool checkDistanceToFirstWaypoint(mission_item_s &mission_item);
	bool checkDistancesBetweenWaypoints(const mission_item_s &mission_item);
	bool checkIfBelowHomeAltitude(const mission_item_s &mission_item, const int current_index);
	bool checkFixedWindLandApproach(mission_item_s &mission_item, const int current_index);

	// methods which are called once at the end
	bool checkTakeoffLandAvailable();

	void doCommonChecks(mission_item_s &mission_item, const int current_index);
	void doVtolChecks(mission_item_s &mission_item, const int current_index);
	void doFixedWingChecks(mission_item_s &mission_item, const int current_index);
	void doMulticopterChecks(mission_item_s &mission_item, const int current_index);
};
