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
 * @file mission_route_land_approaches.h
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "mission_route_provider.h"
#include "safe_point_land.hpp"

namespace mission_route
{

/**
 * @brief Read the landing-approach block associated with the first valid rally point near rtl_position.
 *
 * A block starts at the associated rally point and contains the consecutive NAV_CMD_LOITER_TO_ALT
 * items that follow it. The next rally point starts a new block.
 * Invalid rally points are skipped so a later nearby valid rally point can still be considered.
 */
land_approaches_s getVtolLandApproachesNearLocation(const Provider &provider,
		const PositionYawSetpoint &rtl_position, float home_altitude_amsl);

/** @brief Read the landing-approach block attached to one exact rally-point item. */
land_approaches_s getVtolLandApproachesAtSafePointIndex(const Provider &provider, int safe_point_index,
		float home_altitude_amsl);

bool hasVtolLandApproachesNearLocation(const Provider &provider, const PositionYawSetpoint &rtl_position,
				       float home_altitude_amsl);

bool hasVtolLandApproachesAtSafePointIndex(const Provider &provider, int safe_point_index,
		float home_altitude_amsl);

bool anySafePointHasVtolLandApproach(const Provider &provider, float home_altitude_amsl);

} // namespace mission_route
