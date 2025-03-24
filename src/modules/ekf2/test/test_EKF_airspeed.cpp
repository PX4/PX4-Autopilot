/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * Test the fusion of airspeed data
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"


class EkfAirspeedTest : public ::testing::Test
{
public:

	EkfAirspeedTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf),
		_quat_sim(Eulerf(0.0f, 0.0f, math::radians(45.0f))) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;
	const Quatf _quat_sim;

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// run briefly to init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_sensor_simulator.simulateOrientation(_quat_sim);
		_sensor_simulator.runSeconds(7);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}
};

TEST_F(EkfAirspeedTest, testWindVelocityEstimation)
{
	const Vector3f simulated_velocity_earth(0.0f, 1.5f, 0.0f);
	const Vector2f airspeed_body(2.4f, 0.0f);
	_ekf_wrapper.enableExternalVisionVelocityFusion();
	_sensor_simulator._vio.setVelocity(simulated_velocity_earth);
	_sensor_simulator._vio.setVelocityFrameToLocalNED();
	_sensor_simulator.startExternalVision();

	// Let the EV fusion start first to reset the velocity estimate
	_sensor_simulator.runSeconds(0.5);

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_ekf->set_is_fixed_wing(true);
	_sensor_simulator.startAirspeedSensor();
	_sensor_simulator._airspeed.setData(airspeed_body(0), airspeed_body(0));

	// Wind estimation is rather slow
	_sensor_simulator.runSeconds(15);

	EXPECT_TRUE(_ekf_wrapper.isWindVelocityEstimated());

	const Vector3f vel = _ekf->getVelocity();
	const Vector2f vel_wind_earth = _ekf->getWindVelocity();
	const float height_before_pressure_correction = _ekf->getPosition()(2);

	const Dcmf R_to_earth_sim(_quat_sim);
	EXPECT_TRUE(matrix::isEqual(vel, simulated_velocity_earth));
	const Vector3f vel_wind_expected = simulated_velocity_earth - R_to_earth_sim * (Vector3f(airspeed_body(0),
					   airspeed_body(1), 0.0f));

	EXPECT_NEAR(vel_wind_earth(0), vel_wind_expected(0), 1e-1f);
	EXPECT_NEAR(vel_wind_earth(1), vel_wind_expected(1), 1e-1f);

	EXPECT_NEAR(height_before_pressure_correction, 0.0f, 1e-5f);

	// Apply height correction
	const float static_pressure_coef_xp = 1.0f;
	const float static_pressure_coef_yp = -1.0f;	// not used as wind direction is along x axis
	parameters *_params = _ekf->getParamHandle();
	_params->static_pressure_coef_xp = static_pressure_coef_xp;
	_params->static_pressure_coef_yp = static_pressure_coef_yp;
	float expected_height_difference = 0.5f * static_pressure_coef_xp * airspeed_body(0) * airspeed_body(
			0) / CONSTANTS_ONE_G;

	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.runSeconds(20);

	const float height_after_pressure_correction = _ekf->getPosition()(2);
	// height increase means that state z decrease due to z axis pointing down
	const float expected_height_after_pressure_correction = height_before_pressure_correction -
			expected_height_difference;

	EXPECT_NEAR(height_after_pressure_correction, expected_height_after_pressure_correction, 1e-2f);

}

TEST_F(EkfAirspeedTest, testResetWindUsingAirspeed)
{
	const Vector3f simulated_velocity_earth(-3.6f, 8.f, 0.0f);
	const Vector2f airspeed_body(15.f, 0.0f);
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator._gps.setVelocity(simulated_velocity_earth);
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(11);

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_ekf->set_is_fixed_wing(true);

	// Simulate the fact that the sideslip can start immediately, without
	// waiting for a measurement sample.
	_ekf_wrapper.enableBetaFusion();
	_sensor_simulator.runSeconds(0.1);
	EXPECT_TRUE(_ekf_wrapper.isIntendingBetaFusion());

	_sensor_simulator.startAirspeedSensor();
	_sensor_simulator._airspeed.setData(airspeed_body(0), airspeed_body(0));
	_sensor_simulator.runSeconds(0.1);

	EXPECT_TRUE(_ekf_wrapper.isWindVelocityEstimated());

	const Vector2f vel_wind_earth = _ekf->getWindVelocity();

	const Dcmf R_to_earth_sim(_quat_sim);
	const Vector3f vel_wind_expected = simulated_velocity_earth - R_to_earth_sim * (Vector3f(airspeed_body(0),
					   airspeed_body(1), 0.0f));

	EXPECT_NEAR(vel_wind_earth(0), vel_wind_expected(0), 1.f);
	EXPECT_NEAR(vel_wind_earth(1), vel_wind_expected(1), 1.f);
}

TEST_F(EkfAirspeedTest, testAirspeedDeadReckoning)
{
	const Vector3f simulated_velocity_earth(-3.6f, 8.f, 0.0f);
	const Vector2f airspeed_body(15.f, 0.0f);
	_sensor_simulator.runSeconds(10);

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_ekf->set_is_fixed_wing(true);

	const Vector3f pos_prev = _ekf->getPosition();

	const double latitude_new  = -15.0000005;
	const double longitude_new = -115.0000005;
	const float altitude_new  = 1500.0;
	const float eph = 50.f;
	const float epv = 10.f;

	_ekf->resetGlobalPosToExternalObservation(latitude_new, longitude_new, altitude_new, eph, epv, 0);

	const Vector3f pos = _ekf->getPosition();

	// lat/lon is initialized so the local horizontal position remains constant
	EXPECT_NEAR(pos(0), pos_prev(0), 1e-3f);
	EXPECT_NEAR(pos(1), pos_prev(1), 1e-3f);

	// alt is updated as the local altitude origin was already set
	EXPECT_NEAR(pos(2), pos_prev(2) - (altitude_new - _ekf->getEkfGlobalOriginAltitude()), 1e-3f);

	const LatLonAlt lla = _ekf->getLatLonAlt();
	EXPECT_NEAR(lla.latitude_deg(), latitude_new, 1e-6f);
	EXPECT_NEAR(lla.longitude_deg(), longitude_new, 1e-6f);
	EXPECT_NEAR(lla.altitude(), altitude_new, 1e-3f);


	// Simulate the fact that the sideslip can start immediately, without
	// waiting for a measurement sample.
	_ekf_wrapper.enableBetaFusion();
	_sensor_simulator.runSeconds(1.f);
	EXPECT_TRUE(_ekf_wrapper.isIntendingBetaFusion());

	_sensor_simulator.startAirspeedSensor();
	_sensor_simulator._airspeed.setData(airspeed_body(0), airspeed_body(0));
	_sensor_simulator.runSeconds(1.f);
	EXPECT_TRUE(_ekf_wrapper.isIntendingAirspeedFusion());

	EXPECT_TRUE(_ekf_wrapper.isWindVelocityEstimated());
	const Vector3f vel = _ekf->getVelocity();
	EXPECT_NEAR(vel.norm(), airspeed_body.norm(), 1e-2f);
	const Vector2f vel_wind_earth = _ekf->getWindVelocity();
	EXPECT_NEAR(vel_wind_earth(0), 0.f, .1f);
	EXPECT_NEAR(vel_wind_earth(1), 0.f, .1f);

	EXPECT_TRUE(_ekf->isGlobalHorizontalPositionValid());
}

TEST_F(EkfAirspeedTest, testAirspeedDeadReckoningLatLonAltReset)
{
	// GIVEN: a flying fixed-wing dead-reckoning with airspeed and sideslip fusion
	const Vector3f simulated_velocity_earth(-3.6f, 8.f, 0.0f);
	const Vector2f airspeed_body(15.f, 0.0f);
	_sensor_simulator.runSeconds(10);

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_ekf->set_is_fixed_wing(true);

	double latitude  = -15.0000005;
	double longitude = -115.0000005;
	float altitude  = 1500.0;
	const float eph = 50.f;
	const float epv = 1.f;

	_ekf->resetGlobalPosToExternalObservation(latitude, longitude, altitude, eph, epv, 0);

	_ekf_wrapper.enableBetaFusion();
	_sensor_simulator.runSeconds(1.f);
	EXPECT_TRUE(_ekf_wrapper.isIntendingBetaFusion());

	_sensor_simulator.startAirspeedSensor();
	_sensor_simulator._airspeed.setData(airspeed_body(0), airspeed_body(0));
	_sensor_simulator.runSeconds(10.f);
	EXPECT_TRUE(_ekf_wrapper.isIntendingAirspeedFusion());

	EXPECT_TRUE(_ekf->isGlobalHorizontalPositionValid());

	// WHEN: an external position reset is sent
	ResetLoggingChecker reset_logging_checker(_ekf);
	reset_logging_checker.capturePreResetState();

	double latitude_new = -16.0000005;
	double longitude_new = -116.0000005;
	float altitude_new = 1602.0;
	_ekf->resetGlobalPosToExternalObservation(latitude_new, longitude_new, altitude_new, eph, epv, 0);

	const Vector3f pos_new = _ekf->getPosition();
	const float altitude_est = -pos_new(2) + _ekf->getEkfGlobalOriginAltitude();

	double latitude_est, longitude_est;
	_ekf->global_origin().reproject(pos_new(0), pos_new(1), latitude_est, longitude_est);

	// THEN: the global position is adjusted accordingly
	EXPECT_NEAR(altitude_est, altitude_new, 0.01f);
	EXPECT_NEAR(latitude_est, latitude_new, 1e-3f);
	EXPECT_NEAR(longitude_est, longitude_new, 1e-3f);
	EXPECT_TRUE(_ekf->isGlobalHorizontalPositionValid());

	reset_logging_checker.capturePostResetState();
	EXPECT_TRUE(reset_logging_checker.isVerticalVelocityResetCounterIncreasedBy(0));
	EXPECT_TRUE(reset_logging_checker.isVerticalPositionResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isHorizontalVelocityResetCounterIncreasedBy(0));
	EXPECT_TRUE(reset_logging_checker.isHorizontalPositionResetCounterIncreasedBy(1));

	// AND WHEN: only the lat/lon is valid
	latitude_new = -16.0000005;
	longitude_new = -116.0000005;
	altitude_new = NAN;
	_ekf->resetGlobalPosToExternalObservation(latitude_new, longitude_new, altitude_new, eph, epv, 0);

	// THEN: lat/lon are reset but not the altitude
	reset_logging_checker.capturePostResetState();
	EXPECT_TRUE(reset_logging_checker.isVerticalVelocityResetCounterIncreasedBy(0));
	EXPECT_TRUE(reset_logging_checker.isVerticalPositionResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isHorizontalVelocityResetCounterIncreasedBy(0));
	EXPECT_TRUE(reset_logging_checker.isHorizontalPositionResetCounterIncreasedBy(2));
}
