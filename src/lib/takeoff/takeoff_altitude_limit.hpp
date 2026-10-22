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
 * @file takeoff_altitude_limit.hpp
 *
 * Limit on the altitude a takeoff from the ground may be commanded to. Shared by the commander, which rejects a
 * takeoff command above the limit, and the navigator, which must not install the target of a rejected command.
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

namespace takeoff
{

/**
 * @brief Resolve the altitude a takeoff command will climb to
 *
 * A takeoff command does not have to carry an altitude. In that case the navigator takes off to MIS_TAKEOFF_ALT
 * above the altitude the vehicle is at when the mode is activated, see Takeoff::set_takeoff_position(). Does not
 * apply to a VTOL takeoff, which has no such default.
 *
 * @param commanded_altitude_amsl altitude of the takeoff command, NAN if not set [m AMSL]
 * @param global_position current global position estimate
 * @param mis_takeoff_alt default takeoff height (MIS_TAKEOFF_ALT), NAN if unknown [m]
 * @return altitude the vehicle will climb to, NAN if it cannot be determined [m AMSL]
 */
inline float resolveAltitudeAmsl(const float commanded_altitude_amsl,
				 const vehicle_global_position_s &global_position, const float mis_takeoff_alt)
{
	if (PX4_ISFINITE(commanded_altitude_amsl)) {
		return commanded_altitude_amsl;
	}

	if (!global_position.alt_valid) {
		return NAN;
	}

	return global_position.alt + mis_takeoff_alt;
}

/**
 * @brief Highest altitude a takeoff can be commanded to
 *
 * Derived from the maximum height above the ground the estimator can support with the currently available aiding
 * sources, so that a takeoff is rejected before the vehicle leaves the ground instead of running into the limit
 * during the climb.
 *
 * @param landed true if the vehicle is landed
 * @param global_position current global position estimate
 * @param local_position current local position estimate
 * @return maximum takeoff altitude [m AMSL], NAN if unknown, INFINITY if there is no limit
 */
inline float maxAltitudeAmsl(const bool landed, const vehicle_global_position_s &global_position,
			     const vehicle_local_position_s &local_position)
{
	// The limit is derived from the terrain altitude, which is the altitude the vehicle climbs away from, so it
	// only applies to a takeoff from the ground. There is nothing to reject once the vehicle is in air, the
	// navigator limits the altitude setpoint during the climb instead.
	if (!landed || !global_position.terrain_alt_valid) {
		return NAN;
	}

	// Same limit as the navigator's max HAGL failsafe, so that an accepted takeoff does not trigger it. An axis
	// that is not limited is reported as INFINITY, which results in no limit.
	const float hagl_max = math::min(local_position.hagl_max_z, local_position.hagl_max_xy);

	return global_position.terrain_alt + hagl_max;
}

/**
 * @brief Check a takeoff altitude against the limit
 *
 * @param altitude_amsl takeoff altitude [m AMSL]
 * @param max_altitude_amsl limit from maxAltitudeAmsl() [m AMSL]
 * @return true if the takeoff altitude exceeds the limit
 */
inline bool isAboveLimit(const float altitude_amsl, const float max_altitude_amsl)
{
	// Either side is NAN when it is unknown, and the limit is INFINITY when unlimited. In both cases the comparison
	// is false and the takeoff is not rejected.
	return altitude_amsl > max_altitude_amsl;
}

} // namespace takeoff
