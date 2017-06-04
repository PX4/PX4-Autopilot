/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 */

#ifndef MISSION_FEASIBILITY_CHECKER_H_
#define MISSION_FEASIBILITY_CHECKER_H_

#include <dataman/dataman.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/fw_pos_ctrl_status.h>

class Geofence;
class Navigator;

class MissionFeasibilityChecker
{
private:
	Navigator *_navigator{nullptr};

	/* Checks for all airframes */
	bool checkGeofence(dm_item_t dm_current, size_t nMissionItems, Geofence &geofence, float home_alt, bool home_valid);

	bool checkHomePositionAltitude(dm_item_t dm_current, size_t nMissionItems, float home_alt, bool home_valid,
				       bool &warning_issued, bool throw_error = false);

	bool checkMissionItemValidity(dm_item_t dm_current, size_t nMissionItems, bool condition_landed);

	bool check_dist_1wp(dm_item_t dm_current, size_t nMissionItems, double curr_lat, double curr_lon,
			    float dist_first_wp, bool &warning_issued);

	/* Checks specific to fixedwing airframes */
	bool checkFixedwing(dm_item_t dm_current, size_t nMissionItems, fw_pos_ctrl_status_s *fw_pos_ctrl_status,
			    float home_alt, bool home_valid, float default_acceptance_rad, bool land_start_req);

	bool checkFixedWingTakeoff(dm_item_t dm_current, size_t nMissionItems, float home_alt, bool home_valid,
				   float default_acceptance_rad);
	bool checkFixedWingLanding(dm_item_t dm_current, size_t nMissionItems, fw_pos_ctrl_status_s *fw_pos_ctrl_status,
				   bool land_start_req);

	/* Checks specific to rotarywing airframes */
	bool checkRotarywing(dm_item_t dm_current, size_t nMissionItems,
			     float home_alt, bool home_valid, float default_acceptance_rad);

public:
	MissionFeasibilityChecker(Navigator *navigator) : _navigator(navigator) {}
	~MissionFeasibilityChecker() = default;

	MissionFeasibilityChecker(const MissionFeasibilityChecker &) = delete;
	MissionFeasibilityChecker &operator=(const MissionFeasibilityChecker &) = delete;

	/*
	 * Returns true if mission is feasible and false otherwise
	 */
	bool checkMissionFeasible(const mission_s &mission, float max_waypoint_distance, bool land_start_req);

};

#endif /* MISSION_FEASIBILITY_CHECKER_H_ */
