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
 * EKF2 driven by noise-free sensors generated from a closed-form trajectory over the
 * rotating WGS-84 Earth (Savage's Gen Nav simulator). The sensors are exact, so the
 * remaining estimation error is produced by the filter's own approximations.
 */

#include <gtest/gtest.h>
#include <deque>

#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include <lib/gen_nav/gen_nav.hpp>
#include <lib/world_magnetic_model/geo_mag_declination.h>

struct Errors {
	float attitude_deg{0.f};
	float velocity{0.f};
	float horizontal_position{0.f};
	float vertical_position{0.f};

	void keepMax(const Errors &other)
	{
		attitude_deg = math::max(attitude_deg, other.attitude_deg);
		velocity = math::max(velocity, other.velocity);
		horizontal_position = math::max(horizontal_position, other.horizontal_position);
		vertical_position = math::max(vertical_position, other.vertical_position);
	}

	void print() const
	{
		printf("max errors: attitude %.4f deg, velocity %.4f m/s, horizontal %.4f m, vertical %.4f m\n",
		       (double)attitude_deg, (double)velocity, (double)horizontal_position, (double)vertical_position);
	}

	void expectBelow(const Errors &limits) const
	{
		EXPECT_LT(attitude_deg, limits.attitude_deg);
		EXPECT_LT(velocity, limits.velocity);
		EXPECT_LT(horizontal_position, limits.horizontal_position);
		EXPECT_LT(vertical_position, limits.vertical_position);
	}
};

struct FlightCondition {
	const char *name;
	double latitude_deg;
	double longitude_deg;
	float cruise_vel_north;
	float cruise_vel_east;
	Errors aided_limits;
	Errors dead_reckoning_limits;
};

class EkfGenNavTest : public ::testing::TestWithParam<FlightCondition>
{
public:
	EkfGenNavTest(): ::testing::TestWithParam<FlightCondition>(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf),
		_gen_nav(profile(), Eulerf(0.f, 0.f, kInitialYaw)) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;
	GenNav _gen_nav;

	static constexpr float kInitialYaw = 1.f;
	// The zero gyro update averages all its samples, so the gyro bias learned before the
	// heading is aligned takes this long to fade
	static constexpr double kMotionStart = 60.0;

	// mirrors the delay Gps::send() subtracts from the sample timestamp
	static constexpr double kGpsDelay = 0.11;

	GenNav::Profile profile() const
	{
		GenNav::Profile profile{};
		profile.latitude_deg = GetParam().latitude_deg;
		profile.longitude_deg = GetParam().longitude_deg;
		profile.cruise_vel_north = GetParam().cruise_vel_north;
		profile.cruise_vel_east = GetParam().cruise_vel_east;
		profile.t_start_s = static_cast<float>(kMotionStart);
		return profile;
	}

	void SetUp() override
	{
		// EKF2_DELAY_MAX default. The fusion horizon is one prediction period shorter than this,
		// so the 110 ms library default would fuse the 110 ms delayed GNSS one period late.
		_ekf->getParamHandle()->ekf2_delay_max = 200.f;
		_ekf->init(0);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_sensor_simulator._imu.setRateHz(1000);
		_ekf_wrapper.enableGpsFusion();
		_sensor_simulator.startGps();
	}

	void setSensorData(double t)
	{
		Vector3f delta_angle;
		Vector3f delta_velocity;
		const double dt = t - _gen_nav.time();
		_gen_nav.step(t, delta_angle, delta_velocity);

		if (dt > 0.0) {
			_sensor_simulator._imu.setData(delta_velocity / static_cast<float>(dt), delta_angle / static_cast<float>(dt));
		}

		const LatLonAlt position = _gen_nav.position(t);
		_sensor_simulator._baro.setData(position.altitude());

		const float latitude_deg = static_cast<float>(position.latitude_deg());
		const float longitude_deg = static_cast<float>(position.longitude_deg());
		const float declination = math::radians(get_mag_declination_degrees(latitude_deg, longitude_deg));
		const float inclination = math::radians(get_mag_inclination_degrees(latitude_deg, longitude_deg));
		const Vector3f mag_field_ned = Dcmf(Eulerf(0.f, -inclination, declination))
					       * Vector3f(get_mag_strength_gauss(latitude_deg, longitude_deg), 0.f, 0.f);
		_sensor_simulator._mag.setData(_gen_nav.attitude().rotateVectorInverse(mag_field_ned));

		const double t_gps = math::max(t - kGpsDelay, 0.0);
		const LatLonAlt gps_position = _gen_nav.position(t_gps);
		_sensor_simulator._gps.setLatitude(gps_position.latitude_deg());
		_sensor_simulator._gps.setLongitude(gps_position.longitude_deg());
		_sensor_simulator._gps.setAltitude(gps_position.altitude());
		_sensor_simulator._gps.setVelocity(Vector3f(_gen_nav.velocityNed(t_gps)));
	}

	// Body rate script relative to the local NED frame, during cruise
	static Vector3f maneuverRate(double t)
	{
		const double tau = t - kMotionStart;

		if (tau > 30.0 && tau <= 35.0) { return Vector3f(0.f, 0.f, 0.3f); }

		if (tau > 40.0 && tau <= 41.0) { return Vector3f(0.5f, 0.f, 0.f); }

		if (tau > 41.0 && tau <= 42.0) { return Vector3f(-0.5f, 0.f, 0.f); }

		if (tau > 50.0 && tau <= 51.0) { return Vector3f(0.f, 0.3f, -0.2f); }

		if (tau > 51.0 && tau <= 52.0) { return Vector3f(0.f, -0.3f, 0.2f); }

		return Vector3f();
	}

	Errors outputErrors(double t) const
	{
		Errors errors{};
		errors.attitude_deg = math::degrees(AxisAnglef(_gen_nav.attitude().inversed() * _ekf->getQuaternion()).angle());
		errors.velocity = (_ekf->getVelocity() - Vector3f(_gen_nav.velocityNed(t))).norm();

		const Vector3f position_error = _ekf->getLatLonAlt() - _gen_nav.position(t);
		errors.horizontal_position = Vector2f(position_error).norm();
		errors.vertical_position = fabsf(position_error(2));
		return errors;
	}

	Errors run(double duration_s, bool verbose = false)
	{
		Errors max_errors{};
		const uint64_t end_us = _sensor_simulator.getTime() + static_cast<uint64_t>(duration_s * 1e6);

		while (_sensor_simulator.getTime() < end_us) {
			const double t = static_cast<double>(_sensor_simulator.getTime()) * 1e-6;

			if (t >= kMotionStart && !_ekf->control_status_flags().in_air) {
				_ekf->set_in_air_status(true);
				_ekf->set_vehicle_at_rest(false);
			}

			_gen_nav.setBodyRate(maneuverRate(t));
			setSensorData(t);
			_sensor_simulator.runMicroseconds(1000);

			const Errors errors = outputErrors(t);
			max_errors.keepMax(errors);

			if (verbose && (_sensor_simulator.getTime() % 1'000'000 == 0)) {
				printf("t=%6.1f att=%7.4f deg vel=%7.4f m/s hpos=%7.4f m vpos=%7.4f m\n", t,
				       (double)errors.attitude_deg, (double)errors.velocity,
				       (double)errors.horizontal_position, (double)errors.vertical_position);
			}
		}

		return max_errors;
	}
};

TEST_P(EkfGenNavTest, perfectSensorsTrackTruth)
{
	// GIVEN: the vehicle at rest while GNSS becomes healthy and the filter aligns
	run(kMotionStart);
	ASSERT_TRUE(_ekf_wrapper.isIntendingGpsFusion());

	// WHEN: it accelerates, climbs, cruises and maneuvers in roll, pitch and yaw
	const Errors max_errors = run(70.0);
	max_errors.print();

	// THEN: with exact sensors only the filter's own approximations remain. The horizontal
	// floor comes from GNSS validity times falling 1 ms off the 10 ms fusion grid.
	max_errors.expectBelow(GetParam().aided_limits);
}

TEST_P(EkfGenNavTest, inertialDeadReckoning)
{
	// GIVEN: the position is known before the first prediction so that the Earth rate is
	// compensated during the whole alignment, which is done at rest with all aiding sources
	const LatLonAlt start = _gen_nav.position(0.0);
	_ekf->resetGlobalPosToExternalObservation(start.latitude_deg(), start.longitude_deg(), start.altitude(), 0.1f, 0.1f, 0);

	// keeps the fake height fallback from resetting the dead-reckoned vertical states
	_ekf->getParamHandle()->ekf2_noaid_tout = 1'000'000'000;

	run(kMotionStart);
	ASSERT_TRUE(_ekf_wrapper.isIntendingGpsFusion());

	// WHEN: all aiding stops as the vehicle starts moving
	_sensor_simulator.stopGps();
	_sensor_simulator.stopBaro();
	_sensor_simulator._mag.stop();

	const Errors max_errors = run(70.0);
	EXPECT_TRUE(_ekf->control_status_flags().inertial_dead_reckoning);

	max_errors.print();

	// THEN: with exact sensors the drift comes from the alignment residuals and the filter's
	// own approximations
	max_errors.expectBelow(GetParam().dead_reckoning_limits);
}

static std::string conditionName(const ::testing::TestParamInfo<FlightCondition> &info)
{
	return info.param.name;
}

// Errors limits: {attitude deg, velocity m/s, horizontal m, vertical m}
static const FlightCondition kMidLatitude {
	.name = "MidLatitude",
	.latitude_deg = 47.3566094, .longitude_deg = 8.5190237,
	.cruise_vel_north = 15.f, .cruise_vel_east = -10.f,
	.aided_limits = {0.02f, 0.008f, 0.04f, 0.012f},
	.dead_reckoning_limits = {0.005f, 0.016f, 0.35f, 0.014f}
};

static const FlightCondition kHighLatitude {
	.name = "HighLatitude",
	.latitude_deg = 80.0, .longitude_deg = 16.0,
	.cruise_vel_north = 15.f, .cruise_vel_east = -10.f,
	.aided_limits = {0.1f, 0.02f, 0.05f, 0.04f},
	.dead_reckoning_limits = {0.005f, 0.025f, 0.55f, 0.02f}
};

static const FlightCondition kEquatorCrossing {
	.name = "EquatorCrossing",
	.latitude_deg = -0.005, .longitude_deg = -78.5,
	.cruise_vel_north = 15.f, .cruise_vel_east = -10.f,
	.aided_limits = {0.1f, 0.02f, 0.05f, 0.04f},
	.dead_reckoning_limits = {0.005f, 0.005f, 0.12f, 0.01f}
};

static const FlightCondition kFastCruise {
	.name = "FastCruise",
	.latitude_deg = 47.3566094, .longitude_deg = 8.5190237,
	.cruise_vel_north = 32.f, .cruise_vel_east = -24.f,
	.aided_limits = {0.03f, 0.015f, 0.09f, 0.012f},
	.dead_reckoning_limits = {0.005f, 0.018f, 0.37f, 0.02f}
};

INSTANTIATE_TEST_SUITE_P(EkfGenNav, EkfGenNavTest,
			 ::testing::Values(kMidLatitude, kHighLatitude, kEquatorCrossing, kFastCruise),
			 conditionName);

// EKF2 does not handle the pole yet: its latitude runs past 90 deg while the navigation frame
// flips by 180 deg in yaw. Run with --gtest_also_run_disabled_tests.
static const FlightCondition kNorthPoleCrossing {
	.name = "NorthPoleCrossing",
	.latitude_deg = 89.997, .longitude_deg = 8.5190237,
	.cruise_vel_north = 15.f, .cruise_vel_east = 0.f,
	.aided_limits = {0.1f, 0.02f, 0.05f, 0.04f},
	.dead_reckoning_limits = {0.005f, 0.025f, 0.55f, 0.02f}
};

INSTANTIATE_TEST_SUITE_P(DISABLED_EkfGenNav, EkfGenNavTest, ::testing::Values(kNorthPoleCrossing), conditionName);
