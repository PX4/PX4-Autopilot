/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#pragma once

#include "autopilot_tester.h"

#include <chrono>

#include <mavsdk/geometry.h>

// Test helper for fixed-wing loiter / reposition / Hold behaviour.
//
// All positions are expressed as home-relative NED local coordinates and
// converted to global setpoints internally. Loiter geometry can only be
// observed indirectly (there is no MAVLink message that exposes the internal
// position setpoint), so the checks are trajectory based: they assert where the
// vehicle actually flies over a time window.
class AutopilotTesterLoiter : public AutopilotTester
{
public:
	using LocalCoordinate = mavsdk::geometry::CoordinateTransformation::LocalCoordinate;

	AutopilotTesterLoiter() = default;
	~AutopilotTesterLoiter() = default;

	// Switch to Hold (AUTO_LOITER) and wait until the flight mode is reported.
	void enter_hold_and_wait(std::chrono::seconds timeout = std::chrono::seconds(10));

	// Command a fixed-wing orbit (MAV_CMD_DO_ORBIT) around a home-relative center
	// with the given radius. Must be in Hold for the setpoint to be picked up.
	void command_orbit(LocalCoordinate center, float radius_m, float rel_alt_m);

	// Command a figure-eight (MAV_CMD_DO_FIGURE_EIGHT) around a home-relative center.
	void command_figure_eight(LocalCoordinate center, float major_axis_m, float minor_axis_m, float rel_alt_m);

	// Command a plain reposition to a home-relative target (fly straight toward it).
	void command_reposition(LocalCoordinate target, float rel_alt_m);

	// Send a pause/Hold reposition: lat/lon/alt all NaN, CHANGE_MODE flag set.
	void command_hold_here();

	// Send a reposition that only changes altitude: lat/lon NaN, alt finite.
	void command_reposition_altitude(float rel_alt_m);

	// Wait until the vehicle is circling `center` at ~radius (within tol) and has
	// stayed in that radial band continuously for `settle`.
	void wait_until_on_loiter(LocalCoordinate center, float radius_m, float radius_tol_m,
				  std::chrono::seconds settle, std::chrono::seconds timeout);

	// Wait until the relative (to home) altitude climbs above `threshold_m`. Uses a monotonic
	// threshold (not a band) so it cannot be skipped between samples at a high speed factor.
	void wait_until_relative_altitude_above(float threshold_m, std::chrono::seconds timeout);

	// Assert that for the whole `duration` the vehicle stays in the radial band
	// [radius - tol, radius + tol] around `center`, i.e. keeps circling that loiter.
	void check_stays_on_loiter(LocalCoordinate center, float radius_m, float radius_tol_m,
				   std::chrono::seconds duration);

	// Assert that for the whole `duration` the vehicle stays within `max_radius_m`
	// of `center` (loose "circles near here" check for a freshly created loiter).
	void check_stays_within(LocalCoordinate center, float max_radius_m, std::chrono::seconds duration);

	// Assert the horizontal distance to `point` never drops below `min_dist_m`
	// during `duration` (the vehicle must not reach the given point).
	void check_never_reaches(LocalCoordinate point, float min_dist_m, std::chrono::seconds duration);

	// Assert relative altitude stays within `tol_m` of `rel_alt_m` for `duration`.
	void check_holds_altitude(float rel_alt_m, float tol_m, std::chrono::seconds duration);

	// Assert the vehicle climbs to `target_rel_alt_m` (within tol) while never
	// leaving the radial band around `center`.
	void check_climbs_on_loiter(LocalCoordinate center, float radius_m, float radius_tol_m,
				    float target_rel_alt_m, float alt_tol_m, std::chrono::seconds timeout);

	// Current relative (to home) altitude in meters.
	float current_relative_altitude();

	// Current horizontal position as a home-relative NED coordinate.
	LocalCoordinate current_local_position();

	// NAV_LOITER_RAD default loiter radius in meters.
	float default_loiter_radius();

private:
	// Horizontal distance in meters between a NED sample and a local coordinate.
	static double horizontal_distance(const mavsdk::Telemetry::PositionVelocityNed &sample, const LocalCoordinate &c);
};
