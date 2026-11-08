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
 * @file mission_route_vehicle.h
 *
 * Which airframes use the mission route features, and the vehicle fields of a planner request.
 */

#pragma once

#include "mission_route_types.h"

#include <stdint.h>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

namespace mission_route
{

enum class RouteAirframe : uint8_t {
	kMulticopter,
	kFixedWing,
	kVtol,
	kOther
};

/** The airframe, not the flight mode: a VTOL reports ROTARY_WING in hover and in both transitions. */
RouteAirframe routeAirframe(const vehicle_status_s &status);

/** Airframes that join the mission route when MIS_ROUTE_JOIN is set. */
bool routeJoinSupported(RouteAirframe airframe);

/** Airframes that follow the mission route with RTL_TYPE 7; the others stop without a route. */
bool routeReturnSupported(RouteAirframe airframe);

/** Set the vehicle fields of a planner request, for the Mission join and route Return. */
void setRequestVehicle(MissionResumeRequest &request, const vehicle_status_s &status,
		       const vehicle_local_position_s &local_position);

} // namespace mission_route
