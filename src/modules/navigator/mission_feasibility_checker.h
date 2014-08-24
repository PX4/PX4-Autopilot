/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 */

#ifndef MISSION_FEASIBILITY_CHECKER_H_
#define MISSION_FEASIBILITY_CHECKER_H_

#include <unistd.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/navigation_capabilities.h>
#include <dataman/dataman.h>
#include "geofence.h"


class MissionFeasibilityChecker
{
private:
	int		_mavlink_fd;

	int _capabilities_sub;
	struct navigation_capabilities_s _nav_caps;

	bool _initDone;
	void init();

	/* Checks for all airframes */
	bool checkGeofence(dm_item_t dm_current, size_t nMissionItems, Geofence &geofence);
	bool checkHomePositionAltitude(dm_item_t dm_current, size_t nMissionItems, float home_alt, bool throw_error = false);

	/* Checks specific to fixedwing airframes */
	bool checkMissionFeasibleFixedwing(dm_item_t dm_current, size_t nMissionItems, Geofence &geofence, float home_alt);
	bool checkFixedWingLanding(dm_item_t dm_current, size_t nMissionItems);
	void updateNavigationCapabilities();

	/* Checks specific to rotarywing airframes */
	bool checkMissionFeasibleRotarywing(dm_item_t dm_current, size_t nMissionItems, Geofence &geofence, float home_alt);
public:

	MissionFeasibilityChecker();
	~MissionFeasibilityChecker() {}

	/*
	 * Returns true if mission is feasible and false otherwise
	 */
	bool checkMissionFeasible(bool isRotarywing, dm_item_t dm_current, size_t nMissionItems, Geofence &geofence, float home_alt);

};


#endif /* MISSION_FEASIBILITY_CHECKER_H_ */
