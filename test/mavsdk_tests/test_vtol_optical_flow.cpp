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


// Integration test for optical flow aided navigation on a VTOL (sihsim_standard_vtol_flow).
//
// Like the multicopter flow test this model flies without GNSS fusion, so the EKF origin has to be
// provided the way a GCS would. The vehicle takes off in multicopter mode, transitions to fixed
// wing on command and flies a loiter, which keeps it banked and turning while it translates much
// faster than a multicopter would. The whole loiter is flown low enough for the flow to stay in
// range (SIH_OF_H_MAX / SIH_DISTSNSR_MAX are 30 m on this model), and the estimated NED velocity is
// compared against the SIH ground truth velocity throughout.

#include "autopilot_tester_flow.h"

#include <chrono>

using namespace std::chrono_literals;

namespace
{

constexpr float kFlightAltitude = 10.f;

// A wide loiter keeps the bank angle small, which keeps the downward facing flow sensor looking at
// the ground and the distance it measures close to the altitude.
constexpr float kLoiterRadius = 100.f;

// Band the vehicle has to enter to count as established on the loiter, and the band the flown
// (ground truth) circle is checked against afterwards. The latter has to swallow how tightly the
// fixed wing tracks the circle plus the drift of the flow only position estimate, observed is
// ~1.6 m.
constexpr float kLoiterCaptureTolerance = 20.f;
constexpr float kLoiterRadiusTolerance = 10.f;

// The estimate has to track the truth: the two streams are paired by arrival, so a short
// acceleration between the two samples being compared already shows up as an error. Observed is
// ~0.1 m/s mean and ~0.3 m/s worst case, the tolerances leave room for the sampling jitter.
constexpr float kMeanVelocityErrorTolerance = 0.3f;
constexpr float kMaxVelocityErrorTolerance = 1.f;

// A full turn of the heading, which the vehicle only reaches by flying a whole loiter.
constexpr float kHeadingSweep = 360.f;

} // namespace

TEST_CASE("Optical flow - VTOL loiter tracks the ground truth velocity", "[optical_flow_vtol]")
{
	AutopilotTesterFlow tester;
	tester.connect(connection_url);

	// The ground truth is streamed independently of the estimator, so home can be stored before
	// there is any global position, and it is the position the EKF origin gets set to.
	tester.store_home();
	tester.set_ekf_origin_to_home();

	tester.wait_until_armable();
	tester.set_takeoff_altitude(kFlightAltitude);
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.wait_until_altitude(kFlightAltitude, 60s);

	tester.transition_to_fixedwing();
	tester.wait_until_fixedwing(10s);
	tester.sleep_for(5s); // let the vehicle accelerate to its cruise speed

	// Loiter around wherever the transition ended up, so that the circle does not depend on how far
	// the vehicle travelled while accelerating.
	const auto position = tester.get_current_position_ned();
	const AutopilotTesterFlow::LocalCoordinate loiter_center{position[0], position[1]};

	tester.command_orbit(loiter_center, kLoiterRadius, NAN /* velocity: up to the vehicle */, kFlightAltitude);
	tester.wait_until_on_circle(loiter_center, kLoiterRadius, kLoiterCaptureTolerance, 180s);

	tester.start_ground_truth_comparison();

	// Fly a full circle: the heading of a fixed wing follows the circle, so the sweep doubles as
	// the "the loiter is really being flown" check.
	tester.wait_until_heading_swept(kHeadingSweep, 180s);
	tester.stop_ground_truth_comparison();

	tester.check_ground_truth_on_circle(loiter_center, kLoiterRadius, kLoiterRadiusTolerance);
	tester.check_velocity_error_below(kMeanVelocityErrorTolerance, kMaxVelocityErrorTolerance);
}
