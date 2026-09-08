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
 * @file hagl_limits.hpp
 *
 * The estimator publishes the maximum height above the ground it can support with the currently available
 * aiding sources as a hard limit in vehicle_local_position. Consumers keep a margin to that limit so that
 * normal tracking errors do not push the vehicle past the point where the aiding source drops out.
 *
 * These helpers exist so that all consumers use the same margin: the commander checks commanded takeoff
 * altitudes, the navigator limits altitude setpoints and the flight tasks limit their own setpoints. If those
 * disagreed, an altitude accepted by one would immediately be limited by the other.
 */

#pragma once

#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

namespace hagl_limits
{

static constexpr float kMarginFraction = 0.1f; ///< fraction of the reported maximum height above ground kept as margin
static constexpr float kMarginMaxXY = 1.f; ///< upper bound of the margin applied for xy-control [m]

/**
 * Apply the margin to the maximum height above the ground for z-control reported by the estimator.
 *
 * @param hagl_max maximum height above the ground for z-control reported by the estimator [m]
 * @return height above the ground to limit to [m]
 */
static inline float withMarginZ(const float hagl_max)
{
	return (1.f - kMarginFraction) * hagl_max;
}

/**
 * Apply the margin to the maximum height above the ground for xy-control reported by the estimator.
 *
 * @param hagl_max_xy maximum height above the ground for xy-control reported by the estimator [m]
 * @return height above the ground to limit to [m]
 */
static inline float withMarginXY(const float hagl_max_xy)
{
	return hagl_max_xy - math::min(kMarginFraction * hagl_max_xy, kMarginMaxXY);
}

/**
 * Combine the maximum height above the ground reported for z- and xy-control into a single limit.
 *
 * An axis that is not limited is reported as INFINITY in vehicle_local_position, but as NAN by
 * Ekf::get_ekf_ctrl_limits() before EKF2 converts it. Neither must mask a finite limit on the other axis, and
 * math::min() cannot be used directly for the NAN case: it returns its second argument whenever either one is
 * NAN, so a limit would be dropped depending on the argument order.
 *
 * @param hagl_max_z maximum height above the ground for z-control reported by the estimator [m]
 * @param hagl_max_xy maximum height above the ground for xy-control reported by the estimator [m]
 * @return the tighter of the two limits [m], NAN if neither axis is limited
 */
static inline float combineMax(const float hagl_max_z, const float hagl_max_xy)
{
	if (!PX4_ISFINITE(hagl_max_z)) {
		return hagl_max_xy;
	}

	if (!PX4_ISFINITE(hagl_max_xy)) {
		return hagl_max_z;
	}

	return math::min(hagl_max_z, hagl_max_xy);
}

/**
 * Lowest height above the ground the vehicle has to stay below to keep all aiding sources.
 *
 * Each axis keeps its own margin before the two are combined, so that the tighter limit is the one that
 * applies and neither axis is limited by the other's margin.
 *
 * @param hagl_max_z maximum height above the ground for z-control reported by the estimator [m]
 * @param hagl_max_xy maximum height above the ground for xy-control reported by the estimator [m]
 * @return height above the ground to limit to [m], NAN if neither axis is limited
 */
static inline float getLowestLimit(const float hagl_max_z, const float hagl_max_xy)
{
	return combineMax(withMarginZ(hagl_max_z), withMarginXY(hagl_max_xy));
}

/**
 * Highest altitude a takeoff can be commanded to without exceeding the maximum height above the ground the
 * estimator can support with the currently available aiding sources.
 *
 * Only meaningful while the vehicle is on the ground, where the terrain altitude is the altitude the vehicle
 * will climb away from.
 *
 * @param hagl_max_z maximum height above the ground for z-control reported by the estimator [m]
 * @param hagl_max_xy maximum height above the ground for xy-control reported by the estimator [m]
 * @param terrain_alt terrain altitude below the vehicle [m AMSL]
 * @return maximum takeoff altitude [m AMSL], NAN if there is no limit
 */
static inline float maxTakeoffAltitudeAmsl(const float hagl_max_z, const float hagl_max_xy, const float terrain_alt)
{
	const float hagl_max = getLowestLimit(hagl_max_z, hagl_max_xy);

	if (!PX4_ISFINITE(hagl_max) || !PX4_ISFINITE(terrain_alt)) {
		return NAN;
	}

	return terrain_alt + hagl_max;
}

} // namespace hagl_limits
