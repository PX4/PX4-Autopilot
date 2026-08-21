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
 * @file ParachuteRelease.hpp
 * Release point prediction for the fixed-wing parachute landing.
 */

#pragma once

#include <matrix/matrix/math.hpp>

// [s] nominal time from the release command until the canopy decelerates the vehicle, used to
// predict the forward carry of the touchdown point
static constexpr float kParachuteDeploymentTime = 1.0f;

// [s] conservative time for the canopy to fully open: the release is never triggered below
// FW_LND_PARA_SINK times this, so that the canopy has room to open before touchdown
static constexpr float kParachuteCanopyOpenTime = 3.0f;

// [m/s] guard against division by a zero or negative sink rate
static constexpr float kParachuteMinSinkRate = 0.1f;

/**
 * @brief Minimum release altitude, below which the canopy cannot fully open before touchdown.
 *
 * @param sink_rate Expected sink rate under canopy [m/s]
 * @return Release floor above ground [m]
 */
float parachuteReleaseFloor(float sink_rate);

/**
 * @brief Release altitude: the configured altitude, floored for the canopy opening.
 *
 * @param release_alt_param Configured release altitude above ground (FW_LND_PARA_ALT) [m]
 * @param sink_rate Expected sink rate under canopy [m/s]
 * @return Release altitude above ground [m]
 */
float parachuteReleaseAltitude(float release_alt_param, float sink_rate);

/**
 * @brief Aim point shift compensating the crosswind drift under canopy.
 *
 * The release timing can only correct the drift along the approach; the crosswind component is
 * corrected by aiming upwind of the landing point (subtract the returned shift from it). The
 * drift is predicted from the altitude the release is expected to happen at: the release
 * altitude, or lower if the vehicle cannot hold it (e.g. a motor-less glider).
 *
 * @param wind_vel Wind velocity estimate, NE [m/s]
 * @param approach_direction Unit vector along the landing approach, NE
 * @param altitude_above_ground Current altitude above the landing point [m]
 * @param release_alt Release altitude above ground [m]
 * @param release_floor Minimum release altitude above ground [m]
 * @param sink_rate Expected sink rate under canopy [m/s]
 * @return Crosswind drift during the descent, NE [m]
 */
matrix::Vector2f parachuteCrosswindAimShift(const matrix::Vector2f &wind_vel,
		const matrix::Vector2f &approach_direction, float altitude_above_ground, float release_alt,
		float release_floor, float sink_rate);

/**
 * @brief Along-track distance to the touchdown point at which to release the parachute.
 *
 * Signed: negative in a headwind stronger than the deployment carry, placing the release past
 * the landing point so that the canopy drifts back onto it.
 *
 * @param ground_speed Ground velocity, NE [m/s]
 * @param wind_vel Wind velocity estimate, NE [m/s]
 * @param wind_valid Whether the wind estimate is valid (drift term is zero otherwise)
 * @param approach_direction Unit vector along the landing approach, NE
 * @param altitude_above_ground Current altitude above the landing point [m]
 * @param sink_rate Expected sink rate under canopy [m/s]
 * @return Release distance before the landing point [m]
 */
float parachuteReleaseDistance(const matrix::Vector2f &ground_speed, const matrix::Vector2f &wind_vel,
			       bool wind_valid, const matrix::Vector2f &approach_direction, float altitude_above_ground,
			       float sink_rate);
