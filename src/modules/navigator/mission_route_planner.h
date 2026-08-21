/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file mission_route_planner.h
 *
 * Mission-route planner interface for Navigator features that need the uploaded
 * mission as route geometry. The planner projects positions onto the mission path,
 * ranks candidate paths, and returns plain data for the caller to execute.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "mission_route_types.h"

#include <stdint.h>

namespace mission_route
{
class Provider;
}

class MissionRoutePlanner
{
public:
	/**
	 * @brief Construct a planner over one read-only data provider.
	 *
	 * The provider must outlive the planner.
	 *
	 * Planning calls use shared fixed-memory scratch and must run serially on the Navigator task.
	 */
	explicit MissionRoutePlanner(const mission_route::Provider &provider);
	MissionRoutePlanner(const MissionRoutePlanner &) = delete;
	MissionRoutePlanner &operator=(const MissionRoutePlanner &) = delete;
	MissionRoutePlanner(MissionRoutePlanner &&) = delete;
	MissionRoutePlanner &operator=(MissionRoutePlanner &&) = delete;

	/** @brief Build the Mission-mode smart-rejoin plan back to the mission end using the nominal route direction. */
	mission_route::FailureReason planMissionResumeJoin(const mission_route::MissionResumeRequest &request,
			mission_route::MissionResumePlan &plan) const;
	/** @brief Build the execution-oriented route-following return plan to the selected goal. */
	mission_route::FailureReason planRouteToGoal(const mission_route::RouteToGoalRequest &request,
			mission_route::RouteToGoalPlan &plan) const;

private:
	const mission_route::Provider &_provider;
};
