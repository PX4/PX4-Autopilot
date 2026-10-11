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

// The SIH ground is hills (SIH_TERR_EN 1) for this test, set through the environment in
// the config. The vehicle flies a low, slow leg over rising and falling ground with the
// distance sensor on, which is the regime EKF2's conditional range aid runs in. The aid
// starts below 0.7 times EKF2_RNG_A_HMAX and EKF2_RNG_A_VMAX, 3.5 m and 0.7 m/s at the
// defaults, and holds until 5 m and 1 m/s, so the leg flies at 5 m and 0.6 m/s. The
// checks use only the sensor, the estimator, and the ground truth, so the test does not
// depend on the terrain library.

#include "autopilot_tester.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{

// Seed 80, amplitude 5 m, base frequency 0.005 /m, bearing 30 deg, evaluated with the
// terrain library on the host. Along the 100 m leg the ground runs from 2.9 m below home to
// 2.3 m above. At 5 m of flight altitude the range is under 3.5 m for a third of the leg
// and under 5 m for half of it, with 2.7 m of clearance at the least. The highest point
// past 40 m out is 57 m along the leg, 2.3 m above home.
constexpr float kFlightAltitudeM = 5.f;
constexpr float kLegLengthM = 100.f;
constexpr float kBearingRad = 30.f * M_PI / 180.f;
constexpr float kLandingDistanceM = 57.f;
constexpr float kLandingRiseM = 2.3f;

// The orbit sits on the same leg, centered 45 m out with a 20 m radius. Around the ring the
// ground runs from 3.0 m below home to 2.2 m above, and at 5 m the range is under 5 m for half
// the ring. The ring starts due north of the center, on ground 1.2 m above home.
constexpr float kOrbitCenterDistanceM = 45.f;
constexpr float kOrbitRadiusM = 20.f;
constexpr float kOrbitStartRiseM = 1.2f;
constexpr float kOrbitSpeedMS = 0.6f;          // under 0.7 x EKF2_RNG_A_VMAX, where the range aid engages

constexpr float kRangeAidHeightM = 5.f;        // EKF2_RNG_A_HMAX default

// the ground has to show through the reading as both a rise and a dip
constexpr float kMinTerrainSeenM = 2.f;
// the fused clearance follows the raw range, the estimator's own noise and lag stay well under this
constexpr float kClearanceToleranceM = 0.5f;
// a landed vehicle sits on the rise, not at home altitude, with margin for the contact model
constexpr float kMinLandedRiseM = 1.5f;

struct RangeSample {
	float range_m;
	float flat_plane_m;      // true altitude above home over the cosine of tilt, the flat ground reading
	float bottom_clearance_m;
	float true_altitude_m;
	float est_altitude_m;    // EKF2's relative altitude, which follows the ground once range aid engages
	float cos_tilt;          // at the moment of the sample, the flat plane reading is scaled by it
};

class AutopilotTesterTerrain : public AutopilotTester
{
public:
	void start_sampling()
	{
		// home is reset to the landing spot after touchdown, so keep the takeoff home altitude
		_takeoff_home_alt_m = getTelemetry()->home().absolute_altitude_m;
		CHECK(std::isfinite(_takeoff_home_alt_m));
		CHECK(getTelemetry()->set_rate_distance_sensor(20) == Telemetry::Result::Success);
		CHECK(getTelemetry()->set_rate_attitude_euler(20) == Telemetry::Result::Success);
		CHECK(getTelemetry()->set_rate_altitude(20) == Telemetry::Result::Success);

		_attitude_handle = getTelemetry()->subscribe_attitude_euler([this](Telemetry::EulerAngle a) {
			std::lock_guard<std::mutex> lock(_mutex);
			_cos_tilt = std::cos(a.roll_deg * M_PI / 180.f) * std::cos(a.pitch_deg * M_PI / 180.f);
		});

		// the true altitude, since the estimated one follows the ground once range aid engages
		_ground_truth_handle = getTelemetry()->subscribe_ground_truth([this](Telemetry::GroundTruth g) {
			std::lock_guard<std::mutex> lock(_mutex);
			_true_altitude_m = g.absolute_altitude_m - _takeoff_home_alt_m;
		});

		_altitude_handle = getTelemetry()->subscribe_altitude([this](Telemetry::Altitude a) {
			std::lock_guard<std::mutex> lock(_mutex);
			_bottom_clearance_m = a.bottom_clearance_m;
			_est_altitude_m = a.altitude_relative_m;
		});

		_distance_handle = getTelemetry()->subscribe_distance_sensor([this](Telemetry::DistanceSensor d) {
			std::lock_guard<std::mutex> lock(_mutex);

			if (_sampling && _cos_tilt > 0.5f && std::isfinite(d.current_distance_m) && std::isfinite(_true_altitude_m)) {
				_samples.push_back({d.current_distance_m, _true_altitude_m / _cos_tilt, _bottom_clearance_m, _true_altitude_m, _est_altitude_m, _cos_tilt});
			}
		});
	}

	// the callbacks hold this, so they go before the tester does
	void stop_sampling()
	{
		getTelemetry()->unsubscribe_attitude_euler(_attitude_handle);
		getTelemetry()->unsubscribe_ground_truth(_ground_truth_handle);
		getTelemetry()->unsubscribe_altitude(_altitude_handle);
		getTelemetry()->unsubscribe_distance_sensor(_distance_handle);
	}

	void set_sampling(bool on)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		_sampling = on;
	}

	// check_band compares the range with the range aid height, which only reads on a leg
	// flown at a fixed altitude, and check_following asks for the ground being held instead
	void check_leg_over_hills(bool check_band, bool check_following)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		REQUIRE(_samples.size() > 100);

		float terrain_max = -INFINITY;
		float terrain_min = INFINITY;
		float range_min = INFINITY, range_max = -INFINITY;
		float true_min = INFINITY, true_max = -INFINITY;
		float est_min = INFINITY, est_max = -INFINITY;
		unsigned in_aid_band = 0;
		unsigned clearance_valid = 0;
		unsigned clearance_close = 0;

		for (const auto &s : _samples) {
			// the ground height under the beam is the flat reading minus the real one
			const float terrain_m = (s.flat_plane_m - s.range_m) * s.cos_tilt;
			terrain_max = std::max(terrain_max, terrain_m);
			terrain_min = std::min(terrain_min, terrain_m);
			range_min = std::min(range_min, s.range_m); range_max = std::max(range_max, s.range_m);
			true_min = std::min(true_min, s.true_altitude_m); true_max = std::max(true_max, s.true_altitude_m);
			est_min = std::min(est_min, s.est_altitude_m); est_max = std::max(est_max, s.est_altitude_m);

			if (s.range_m < kRangeAidHeightM) {
				in_aid_band++;
			}

			if (std::isfinite(s.bottom_clearance_m)) {
				clearance_valid++;

				if (std::fabs(s.bottom_clearance_m - s.range_m) < kClearanceToleranceM) {
					clearance_close++;
				}
			}
		}

		const float aid_fraction = static_cast<float>(in_aid_band) / _samples.size();
		const float aid_percent = 100.f * aid_fraction;
		std::cout << "terrain seen by the range: " << terrain_min << " to " << terrain_max << " m over " << _samples.size() << " samples\n"
			  << "range " << range_min << " to " << range_max << " m, under " << kRangeAidHeightM << " m for " << aid_percent << "% of the samples\n"
			  << "true altitude " << true_min << " to " << true_max << " m, estimated " << est_min << " to " << est_max << " m\n"
			  << "fused clearance within " << kClearanceToleranceM << " m of the range on " << clearance_close << " of " << clearance_valid << " samples"
			  << std::endl;

		CHECK(terrain_max > kMinTerrainSeenM);
		CHECK(terrain_min < -kMinTerrainSeenM);

		if (check_band) {
			CHECK(aid_fraction > 0.3f);
			CHECK(aid_fraction < 0.7f);
		}

		if (check_following) {
			// with the range as the height source the estimate holds while the true altitude rides the ground
			CHECK(true_max - true_min > kMinTerrainSeenM);
			CHECK(est_max - est_min < 1.f);
		}

		REQUIRE(clearance_valid > 100);
		CHECK(clearance_close > 0.9f * clearance_valid);
	}

	void check_landed_on_rise(float rise_m, float min_rise_m)
	{
		const float landed_alt_m = getTelemetry()->ground_truth().absolute_altitude_m;
		std::cout << "landed " << landed_alt_m - _takeoff_home_alt_m << " m above home, the rise is " << rise_m << " m" << std::endl;
		CHECK(landed_alt_m - _takeoff_home_alt_m > min_rise_m);
	}

	// One full turn around center at altitude through PX4's orbit mode, the DO_ORBIT command a
	// ground station sends. The turn is counted from telemetry so the test knows when the ring is
	// closed. Time is the vehicle's own, so the loop and its timeout scale with the speed factor.
	void fly_orbit(float center_n, float center_e, float radius_m, float altitude_m, std::chrono::seconds timeout)
	{
		const auto center = get_coordinate_transformation().global_from_local({center_n, center_e});
		const double altitude_amsl_m = getHome().absolute_altitude_m + altitude_m;
		REQUIRE(getAction()->do_orbit(radius_m, kOrbitSpeedMS, Action::OrbitYawBehavior::HoldFrontTangentToCircle,
					      center.latitude_deg, center.longitude_deg, altitude_amsl_m) == Action::Result::Success);

		const std::chrono::milliseconds step = 50ms;
		float swept_rad = 0.f;
		float last_angle = NAN;
		std::chrono::milliseconds elapsed = 0ms;

		while (std::fabs(swept_rad) < 2.f * M_PI) {
			REQUIRE(elapsed < timeout);
			const auto pos = getTelemetry()->position_velocity_ned().position;
			const float angle = std::atan2(pos.east_m - center_e, pos.north_m - center_n);

			if (std::isfinite(last_angle)) {
				float d = angle - last_angle;

				if (d > M_PI) { d -= 2.f * M_PI; }

				if (d < -M_PI) { d += 2.f * M_PI; }

				swept_rad += d;
			}

			last_angle = angle;
			sleep_for(step);
			elapsed += step;
		}

		std::cout << "orbit swept " << swept_rad * 180.f / M_PI << " deg" << std::endl;
	}

private:
	std::mutex _mutex;
	Telemetry::AttitudeEulerHandle _attitude_handle{};
	Telemetry::GroundTruthHandle _ground_truth_handle{};
	Telemetry::AltitudeHandle _altitude_handle{};
	Telemetry::DistanceSensorHandle _distance_handle{};
	float _takeoff_home_alt_m{NAN};
	bool _sampling{false};
	float _cos_tilt{1.f};
	float _true_altitude_m{NAN};
	float _bottom_clearance_m{NAN};
	float _est_altitude_m{NAN};
	std::vector<RangeSample> _samples;
};

} // namespace

TEST_CASE("Terrain - rangefinder reads the hills and the vehicle lands on one", "[terrain]")
{
	AutopilotTesterTerrain tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.request_ground_truth();
	tester.start_sampling();
	tester.set_rc_loss_exception(AutopilotTester::RcLossException::Offboard);
	tester.arm();

	const float acceptance_radius_m = 1.f;
	const std::chrono::seconds goto_timeout = 240s;

	const float north_per_m = std::cos(kBearingRad);
	const float east_per_m = std::sin(kBearingRad);
	// the nose points along the leg on the way out and back at it on the return
	const float leg_yaw_deg = 30.f;
	const Offboard::PositionNedYaw above_home {0.f, 0.f, -kFlightAltitudeM, leg_yaw_deg};
	const Offboard::PositionNedYaw leg_end {kLegLengthM * north_per_m, kLegLengthM * east_per_m, -kFlightAltitudeM, leg_yaw_deg};
	const Offboard::PositionNedYaw above_rise {kLandingDistanceM * north_per_m, kLandingDistanceM * east_per_m, -kFlightAltitudeM, leg_yaw_deg + 180.f};

	tester.offboard_goto(above_home, acceptance_radius_m, goto_timeout);

	tester.set_sampling(true);
	tester.offboard_goto(leg_end, acceptance_radius_m, goto_timeout);
	tester.set_sampling(false);
	tester.check_leg_over_hills(true, false);

	tester.offboard_goto(above_rise, acceptance_radius_m, goto_timeout);
	tester.offboard_land();
	tester.wait_until_disarmed(120s);
	tester.check_landed_on_rise(kLandingRiseM, kMinLandedRiseM);
	tester.stop_sampling();
}

TEST_CASE("Terrain - rangefinder reads the hills around an orbit", "[terrain]")
{
	AutopilotTesterTerrain tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.store_home();
	tester.request_ground_truth();
	tester.start_sampling();
	tester.set_rc_loss_exception(AutopilotTester::RcLossException::Offboard);
	tester.arm();

	const float acceptance_radius_m = 1.f;
	const std::chrono::seconds goto_timeout = 240s;
	const float center_n = kOrbitCenterDistanceM * std::cos(kBearingRad);
	const float center_e = kOrbitCenterDistanceM * std::sin(kBearingRad);

	const Offboard::PositionNedYaw above_home {0.f, 0.f, -kFlightAltitudeM, 0.f};
	const Offboard::PositionNedYaw ring_start {center_n + kOrbitRadiusM, center_e, -kFlightAltitudeM, 90.f};

	tester.offboard_goto(above_home, acceptance_radius_m, goto_timeout);
	tester.offboard_goto(ring_start, acceptance_radius_m, goto_timeout);

	tester.set_sampling(true);
	tester.fly_orbit(center_n, center_e, kOrbitRadiusM, kFlightAltitudeM, 420s);
	tester.set_sampling(false);
	tester.check_leg_over_hills(false, true);

	tester.land();
	tester.wait_until_disarmed(120s);
	tester.check_landed_on_rise(kOrbitStartRiseM, 0.5f);
	tester.stop_sampling();
}
