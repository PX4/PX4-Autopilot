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
 * @file mission_route_types.h
 *
 * Mission-route planner data types.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "navigation.h"
#include "safe_point_land.hpp"

#include <math.h>
#include <stdint.h>

#include <px4_platform_common/defines.h>

namespace mission_route
{

static constexpr double kNullIslandThresholdDeg{1e-7};
static constexpr float kLandApproachAssociationDistanceM{10.f};

/** @brief A global geographic coordinate. */
struct Position {
	double lat{static_cast<double>(NAN)};
	double lon{static_cast<double>(NAN)};
	float alt{NAN};

	bool valid() const;
};

float getAbsoluteAltitudeForMissionItem(const mission_item_s &mission_item, float home_altitude_amsl);

/** @brief Extract the valid position from a rally-point safe-point item. */
bool extractSafePointPosition(const mission_item_s &safe_point_item, float home_altitude_amsl, Position &position);

/** @brief Copy a valid Position into a PositionYawSetpoint (yaw left unset). Returns false for an invalid position. */
bool copyPositionToYawSetpoint(const Position &position, PositionYawSetpoint &setpoint);

/** @brief Convert one LOITER_TO_ALT safe-point item into a concrete landing-approach point. */
loiter_point_s makeVtolLandApproachPoint(const mission_item_s &mission_item, float home_altitude_amsl);

} // namespace mission_route
