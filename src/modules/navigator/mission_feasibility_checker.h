/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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
 * @file mission_feasibility_checker.h
 * Provides checks if mission is feasible given the navigation capabilities
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Sander Smeets <sander@droneslab.com>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#pragma once

#include <dataman/dataman.h>
#include <uORB/topics/mission.h>
#include <px4_platform_common/module_params.h>

class Geofence;
class Navigator;

class MissionFeasibilityChecker: public ModuleParams
{
private:
	Navigator *_navigator{nullptr};

	/**
	 * @brief Check geofence validity (if available)
	 *
	 * Check includes:
	 * - home position is valid if required
	 * - home position is valid if geofence is set
	 * - all mission items don't violate set geofence
	 *
	 * @param mission Mission struct
	 * @param home_alt Home altitude [m AMSL]
	 * @param home_valid Home valid
	 * @return True if checks passed, False if not
	 */
	bool checkGeofence(const mission_s &mission, float home_alt, bool home_valid);

	/**
	 * @brief Check if Home altitude is valid if waypoints have relative altitude, and that waypoints are above it
	 *
	 * @param mission Mission struct
	 * @param home_alt Home altitude [m AMSL]
	 * @param home_alt_valid Home altitude valid
	 * @return True if checks passed, False if not
	 */
	bool checkHomePositionAltitude(const mission_s &mission, float home_alt, bool home_alt_valid);

	/**
	 * @brief Check if all mission items are supported
	 *
	 * @param mission Mission struct
	 * @return True if all set mission items are supported, False if not
	 */
	bool checkMissionItemValidity(const mission_s &mission);

	/**
	 * @brief Check if distance to first waypoint is below threshold
	 *
	 * @param mission Mission struct
	 * @param max_distance Maximally allowed distance to first waypoint [m]
	 * @return True if distance is below threshold, False if not
	 */
	bool checkDistanceToFirstWaypoint(const mission_s &mission, float max_distance);

	/**
	 * @brief Check if distance between consecutive waypoints is below threshold
	 *
	 * @param mission Mission struct
	 * @param max_distance Maximally allowed distance between consecutive waypoints [m]
	 * @return True if all distances between waypoints are below threshold, False if not
	 */
	bool checkDistancesBetweenWaypoints(const mission_s &mission, float max_distance);

	/**
	 * @brief Check if mission contains takeoff (safe in _has_takeoff), and if that takeoff is valid
	 *
	 * @param mission Mission struct
	 * @param home_alt Home altitude [m AMSL]
	 * @return True if the planned takeoff is valid, or no takeoff planned, False otherwise
	 */
	bool checkTakeoff(const mission_s &mission, float home_alt);

	/**
	 * @brief Check if requirement for availability of planned takeoff and/or landing is fullfilled
	 *
	 * The requirement is controlled through parameter MIS_TKO_LAND_REQ.
	 * Only checks for availability of takeoff/landing, not their validity
	 *
	 * @return True if check passes, False if not
	 */
	bool checkTakeoffLandAvailable();

	/**
	 * @brief Check if mission contains landing
	 *
	 * @param mission Mission struct
	 * @return True if mission contains a landing, False if otherwise
	 */
	bool hasMissionLanding(const mission_s &mission);

	/**
	 * @brief Fixed-wing vehicle: Check if mission contains landing (safe in _has_landing), and if that landing is valid
	 *
	 * @param mission Mission struct
	 * @return True if the planned landing is valid, or no landing planned, False otherwise
	 */
	bool checkFixedWingLanding(const mission_s &mission);

	/**
	 * @brief VTOL vehicle: Check if mission contains landing (safe in _has_landing), and if that landing is valid
	 *
	 * @param mission Mission struct
	 * @return True if the planned landing is valid, or no landing planned, False otherwise
	 */
	bool checkVTOLLanding(const mission_s &mission);

	bool _has_takeoff{false};
	bool _has_landing{false};

public:
	MissionFeasibilityChecker(Navigator *navigator) : ModuleParams(nullptr), _navigator(navigator) {}
	~MissionFeasibilityChecker() = default;

	MissionFeasibilityChecker(const MissionFeasibilityChecker &) = delete;
	MissionFeasibilityChecker &operator=(const MissionFeasibilityChecker &) = delete;

	/*
	 * Returns true if mission is feasible and false otherwise
	 */
	bool checkMissionFeasible(const mission_s &mission,
				  float max_distance_to_1st_waypoint, float max_distance_between_waypoints);
};
