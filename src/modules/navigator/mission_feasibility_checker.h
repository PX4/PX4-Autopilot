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

#include <dataman_client/DatamanClient.hpp>
#include <uORB/topics/mission.h>
#include <px4_platform_common/module_params.h>
#include "MissionFeasibility/FeasibilityChecker.hpp"

class Navigator;

class MissionFeasibilityChecker: public ModuleParams
{
private:
	Navigator *_navigator{nullptr};
	DatamanClient &_dataman_client;
	FeasibilityChecker _feasibility_checker;

	bool checkMissionAgainstGeofence(const mission_s &mission, float home_alt, bool home_valid);

public:
	MissionFeasibilityChecker(Navigator *navigator, DatamanClient &dataman_client) :
		ModuleParams(nullptr),
		_navigator(navigator),
		_dataman_client(dataman_client),
		_feasibility_checker()
	{

	}
	~MissionFeasibilityChecker() = default;

	MissionFeasibilityChecker(const MissionFeasibilityChecker &) = delete;
	MissionFeasibilityChecker &operator=(const MissionFeasibilityChecker &) = delete;

	/*
	 * Returns true if mission is feasible and false otherwise
	 */
	bool checkMissionFeasible(const mission_s &mission);
};
