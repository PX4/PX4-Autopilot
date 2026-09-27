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


// Integration test for optical flow aided navigation on a multicopter (sihsim_quadx_flow).
//
// The model has no GNSS fusion: the horizontal velocity is observed by the simulated downward
// facing optical flow sensor together with the distance sensor, and the height comes from the
// barometer. The test provides the EKF origin the way a GCS would (SET_GPS_GLOBAL_ORIGIN, the
// estimator never picks one on its own without GNSS), takes off and flies an orbit with the nose
// pointing at the circle center. Circling that way keeps the vehicle translating in every
// direction while it also keeps turning, so the flow rate the sensor reports is a mix of ground
// relative velocity and body rates. Throughout the manoeuvre the estimated NED velocity is
// compared against the SIH ground truth velocity.

#include "autopilot_tester_flow.h"

#include <chrono>

using namespace std::chrono_literals;

namespace
{

// Well below SIH_OF_H_MAX / SIH_DISTSNSR_MAX (30 m) so the flow stays valid throughout.
constexpr float kFlightAltitude = 10.f;

// The center is one radius north of home, so the takeoff position is already on the circle.
constexpr float kOrbitRadius = 20.f;
constexpr float kOrbitVelocity = 5.f; // ~25 s and, with the nose on the center, ~14 deg/s per turn
const AutopilotTesterFlow::LocalCoordinate kOrbitCenter{kOrbitRadius, 0.0};

// The estimate has to track the truth: the two streams are paired by arrival, so a short
// acceleration between the two samples being compared already shows up as an error. Observed is
// ~0.1 m/s mean and ~0.3 m/s worst case, the tolerances leave room for the sampling jitter.
constexpr float kMeanVelocityErrorTolerance = 0.3f;
constexpr float kMaxVelocityErrorTolerance = 1.f;

// The circle the vehicle actually flew (ground truth) is checked against the commanded one. The
// band has to swallow the initial acceleration onto the circle and whatever the flow only position
// estimate drifts over one orbit, observed is ~2.5 m.
constexpr float kOrbitRadiusTolerance = 8.f;

// A full turn of the heading, which the vehicle only reaches by flying a whole orbit.
constexpr float kHeadingSweep = 360.f;

} // namespace

TEST_CASE("Optical flow: orbit tracks the ground truth velocity", "[optical_flow]")
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
	tester.sleep_for(3s); // let the position controller settle into the hover

	tester.command_orbit(kOrbitCenter, kOrbitRadius, kOrbitVelocity, kFlightAltitude);
	tester.start_ground_truth_comparison();

	// Fly a full circle. The heading turns with it because the nose follows the circle center, so
	// the sweep doubles as the "the orbit is really being flown" check.
	tester.wait_until_heading_swept(kHeadingSweep, 120s);
	tester.stop_ground_truth_comparison();

	tester.check_ground_truth_on_circle(kOrbitCenter, kOrbitRadius, kOrbitRadiusTolerance);
	tester.check_velocity_error_below(kMeanVelocityErrorTolerance, kMaxVelocityErrorTolerance);

	tester.land();
	tester.wait_until_disarmed();
}
