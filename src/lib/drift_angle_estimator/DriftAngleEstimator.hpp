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
 * @file DriftAngleEstimator.hpp
 *
 * Estimates the drift angle of a surface vehicle from its ground velocity.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <uORB/topics/vehicle_local_position.h>

/**
 * Estimates the drift angle, the angle from the expected direction of travel to the course over ground.
 * The expected direction of travel is given in body frame, forwards by default. An omnidirectional vehicle passes
 * the direction it pushes in, e.g. its thrust direction.
 * Course control subtracts the drift angle from the course to get the heading setpoint, like the fixed wing corrects
 * its course setpoint for the wind, so the vehicle moves over ground along the course despite wheel slip or current.
 */
class DriftAngleEstimator
{
public:
	/**
	 * Update the estimate from the angle between the ground velocity and the expected direction of travel.
	 * Only updated while moving along the expected direction fast enough for the velocity direction to be meaningful,
	 * kept otherwise.
	 * @param vehicle_local_position Local position with the velocity and heading estimate
	 * @param expected_direction [rad] [@frame Body] Expected direction of travel, 0 forwards, pi/2 right, pi backwards
	 */
	void update(const vehicle_local_position_s &vehicle_local_position, float expected_direction = 0.f);

	/**
	 * @return Drift angle [rad], 0 until estimated since the last reset
	 */
	float getDriftAngle() const { return _valid ? _filter.getState() : 0.f; }

	void reset() { _valid = false; }

	static constexpr float MIN_SPEED{.5f}; ///< [m/s] Speed along the expected direction below which the velocity direction is noise
	static constexpr float EVH_FACTOR{3.f}; ///< Factor of the velocity standard deviation the speed has to exceed
	static constexpr float MAX_DRIFT_ANGLE{M_PI_F / 4.f}; ///< [rad] Limits the heading offset from the course

private:
	static constexpr uint64_t TIME_CONSTANT_US{1000000}; ///< Filters the yaw oscillations of the vehicle
	static constexpr uint64_t MAX_DT_US{100000}; ///< Sample interval limit, as the estimate pauses at low speed

	AlphaFilter<float> _filter{TIME_CONSTANT_US};
	bool _valid{false};
	hrt_abstime _timestamp{0};
};
