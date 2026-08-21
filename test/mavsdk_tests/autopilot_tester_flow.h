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
#include <mutex>
#include <utility>
#include <vector>

#include <mavsdk/geometry.h>

// Test helper for optical flow aided navigation.
//
// The flow models (sihsim_quadx_flow) fly without GNSS fusion, so the estimator
// never picks a global origin on its own and everything that needs a global
// position (arming checks, home position, orbit center) stays unavailable until
// a GCS provides one.
//
// Flow quality can only be judged indirectly: the velocity the estimator
// reports is compared against the SIH ground truth velocity, which is streamed
// out as HIL_STATE_QUATERNION.
class AutopilotTesterFlow : public AutopilotTester
{
public:
	using LocalCoordinate = mavsdk::geometry::CoordinateTransformation::LocalCoordinate;

	AutopilotTesterFlow() = default;
	~AutopilotTesterFlow() = default;

	// Set the EKF origin to the stored (ground truth) home position with SET_GPS_GLOBAL_ORIGIN and
	// wait until the vehicle reports a global position. Requires store_home() to have run.
	void set_ekf_origin_to_home(std::chrono::seconds timeout = std::chrono::seconds(20));

	// Wait until the autopilot itself reports that it can be armed.
	//
	// wait_until_ready() cannot be used on a flow only vehicle: MAVSDK derives both the global and
	// the local position health from the GPS / optical flow / vision flags in SYS_STATUS, and PX4
	// only ever fills in the GPS one, so health_all_ok() stays false no matter how good the flow
	// aided estimate is.
	void wait_until_armable(std::chrono::seconds timeout = std::chrono::seconds(60));

	// Command an orbit (MAV_CMD_DO_ORBIT) around a home-relative center. On a multicopter it is
	// flown with the nose pointing at the center so that the heading turns with the vehicle, on a
	// fixed wing it turns into a loiter of that radius. Pass NAN as the velocity to leave the
	// tangential velocity to the vehicle.
	void command_orbit(LocalCoordinate center, float radius_m, float velocity_m_s, float rel_alt_m);

	// Wait until the vehicle is on the circle around `center`, i.e. its distance to the center is
	// within `tolerance_m` of `radius_m`.
	void wait_until_on_circle(LocalCoordinate center, float radius_m, float tolerance_m,
				  std::chrono::seconds timeout);

	// Start comparing the estimated NED velocity against the ground truth velocity, and start
	// tracking how far the (ground truth) heading turns. Both streams are requested at `rate_hz` so
	// that the two samples being compared are close in time.
	void start_ground_truth_comparison(double rate_hz = 30.0);
	void stop_ground_truth_comparison();

	// Wait until the heading has turned by `min_sweep_deg` in total since the comparison started.
	void wait_until_heading_swept(float min_sweep_deg, std::chrono::seconds timeout);

	// Assert the ground truth trajectory stayed on the commanded circle: every recorded sample was
	// within `tolerance_m` of `radius_m` around `center`. Together with the heading sweep this is
	// what tells a flown orbit apart from a vehicle that only turned on the spot.
	void check_ground_truth_on_circle(LocalCoordinate center, float radius_m, float tolerance_m);

	// Assert on the collected velocity error: the mean absolute error of every axis has to stay
	// below `mean_tol_m_s` and no single sample may be off by more than `max_tol_m_s`.
	void check_velocity_error_below(float mean_tol_m_s, float max_tol_m_s);

private:
	static constexpr unsigned kAxes = 3; // north, east, down

	// Total heading change since the comparison started, in degrees.
	float heading_swept_deg();

	// Latest estimated velocity, kept here so that the ground truth callback pairs each ground truth
	// sample with an estimate instead of reading back into MAVSDK from inside a callback.
	std::mutex _velocity_mutex;
	bool _estimate_valid{false};
	float _estimated_velocity_m_s[kAxes] {};
	double _error_sum_m_s[kAxes] {};
	float _error_max_m_s[kAxes] {};
	unsigned _error_samples{0};

	// Ground truth positions (lat/lon in degrees) recorded during the comparison.
	std::vector<std::pair<double, double>> _ground_truth_lla;

	bool _heading_valid{false};
	float _previous_heading_rad{0.f};
	double _heading_swept_rad{0.0};

	Telemetry::PositionVelocityNedHandle _position_velocity_handle{};
	MavlinkPassthrough::MessageHandle _ground_truth_handle{};
};
