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

#include "mission_route_vehicle.h"

namespace mission_route
{

RouteAirframe routeAirframe(const vehicle_status_s &status)
{
	if (status.is_vtol) {
		return RouteAirframe::kVtol;
	}

	switch (status.vehicle_type) {
	case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
		return RouteAirframe::kMulticopter;

	case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
		return RouteAirframe::kFixedWing;

	default:
		return RouteAirframe::kOther;
	}
}

bool routeJoinSupported(RouteAirframe airframe)
{
	switch (airframe) {
	case RouteAirframe::kMulticopter:
	case RouteAirframe::kFixedWing:
	case RouteAirframe::kVtol:
		return true;

	case RouteAirframe::kOther:
		return false;
	}

	return false;
}

bool routeReturnSupported(RouteAirframe airframe)
{
	switch (airframe) {
	case RouteAirframe::kMulticopter:
	case RouteAirframe::kFixedWing:
	case RouteAirframe::kVtol:
		return true;

	case RouteAirframe::kOther:
		return false;
	}

	return false;
}

void setRequestVehicle(MissionResumeRequest &request, const vehicle_status_s &status,
		       const vehicle_local_position_s &local_position)
{
	request.is_fixed_wing = (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
	request.in_transition_to_fw = status.in_transition_to_fw;
	request.is_vtol = status.is_vtol;

	if (local_position.v_xy_valid) {
		request.velocity_north_m_s = local_position.vx;
		request.velocity_east_m_s = local_position.vy;
	}
}

} // namespace mission_route
