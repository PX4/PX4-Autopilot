/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * Test the mag fusion
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"

class EkfMagTest : public ::testing::Test
{
public:

	EkfMagTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// Init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);
	}

	const uint32_t _init_duration_s{6};
};

TEST_F(EkfMagTest, fusionStartWithReset)
{
	_ekf->set_min_required_gps_health_time(5e6);
	// GIVEN: some meaningful mag data
	const float mag_heading = M_PI_F / 3.f;
	const float incl = 63.1f;
	const Vector3f mag_data(0.2f * cosf(mag_heading), -0.2f * sinf(mag_heading),
				0.4f * sinf(incl) * sqrtf(0.2f * 0.2f + 0.4f * 0.4f));
	_sensor_simulator._mag.setData(mag_data);

	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.runSeconds(_init_duration_s);

	// THEN: the fusion initializes using the mag data and runs normally
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), mag_heading, radians(1.f));
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());

	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
	// AND WHEN: GNSS fusion starts
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(6);

	// THEN: the earth mag field is reset to the WMM
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 2);

	Vector3f mag_earth = _ekf->getMagEarthField();
	float mag_decl = atan2f(mag_earth(1), mag_earth(0));
	float mag_decl_wmm_deg = 0.f;
	_ekf->get_mag_decl_deg(mag_decl_wmm_deg);
	EXPECT_NEAR(degrees(mag_decl), mag_decl_wmm_deg, 1e-5f);

	float mag_incl = asinf(mag_earth(2) / fmaxf(mag_earth.norm(), 1e-4f));
	float mag_incl_wmm_deg = 0.f;
	_ekf->get_mag_inc_deg(mag_incl_wmm_deg);
	EXPECT_NEAR(degrees(mag_incl), mag_incl_wmm_deg, 1e-5f);
}

TEST_F(EkfMagTest, noInitLargeStrength)
{
	// GIVEN: a really large magnetic field
	_ekf_wrapper.enableMagStrengthCheck();
	const Vector3f mag_data(1.f, 1.f, 1.f);
	_sensor_simulator._mag.setData(mag_data);

	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.runSeconds(_init_duration_s);

	// THEN: the fusion shouldn't start
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());
	EXPECT_EQ(0, (int) _ekf->control_status_flags().yaw_align);
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter);
}

TEST_F(EkfMagTest, suddenLargeStrength)
{
	_ekf_wrapper.enableMagStrengthCheck();

	// GIVEN: some meaningful mag data
	const float mag_heading = -M_PI_F / 7.f;
	Vector3f mag_data(0.2f * cosf(mag_heading), -0.2f * sinf(mag_heading), 0.4f);
	_sensor_simulator._mag.setData(mag_data);

	_sensor_simulator.runSeconds(_init_duration_s);

	// THEN: the fusion initializes using the mag data and runs normally
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), mag_heading, radians(1.f));
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());

	// BUT WHEN: the mag field norm is suddenly too large
	mag_data *= 5.f;
	_sensor_simulator._mag.setData(mag_data);
	_sensor_simulator.runSeconds(6.f);

	// THEN: the mag fusion should stop after some time
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());
}

TEST_F(EkfMagTest, noInitLargeInclination)
{
	// GIVEN: a really large magnetic field
	_ekf_wrapper.enableMagInclinationCheck();
	// To prevent an early pass of the inclination check, "force WMM" must be set
	_ekf_wrapper.enableMagCheckForceWMM();
	_sensor_simulator.startGps();
	Vector3f mag_data(0.4f, 0.f, 0.f);
	_sensor_simulator._mag.setData(mag_data);

	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.runSeconds(_init_duration_s + 10.f); // live some extra time fo GNSS checks to pass

	// THEN: the fusion shouldn't start
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());
	EXPECT_EQ(0, (int) _ekf->control_status_flags().yaw_align);
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter);

	// BUT then: as soon as there is some meaningful data
	const float mag_heading = -M_PI_F / 7.f;
	mag_data = Vector3f(0.2f * cosf(mag_heading), -0.2f * sinf(mag_heading), 0.4f);
	_sensor_simulator._mag.setData(mag_data);

	_sensor_simulator.runSeconds(2.f);

	float decl_deg = 0.f;
	_ekf->get_mag_decl_deg(decl_deg);

	// THEN: the fusion initializes using the mag data and runs normally
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), mag_heading + radians(decl_deg), radians(1.f));
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_EQ(1, (int) _ekf->control_status_flags().yaw_align);
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
}

TEST_F(EkfMagTest, suddenInclinationChange)
{
	_ekf_wrapper.enableMagInclinationCheck();
	_ekf_wrapper.enableMagCheckForceWMM();
	_sensor_simulator.startGps();

	// GIVEN: some meaningful mag data
	const float mag_heading = -M_PI_F / 7.f;
	Vector3f mag_data(0.2f * cosf(mag_heading), -0.2f * sinf(mag_heading), 0.4f);
	_sensor_simulator._mag.setData(mag_data);

	_sensor_simulator.runSeconds(_init_duration_s + 10.f);

	float decl_deg = 0.f;
	_ekf->get_mag_decl_deg(decl_deg);

	// THEN: the fusion initializes using the mag data and runs normally
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), mag_heading + radians(decl_deg), radians(1.f));
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());

	// BUT WHEN: the mag field inclination suddenly changes
	mag_data(2) = -mag_data(2);
	_sensor_simulator._mag.setData(mag_data);
	_sensor_simulator.runSeconds(6.f);

	// THEN: the mag fusion should stop after some time
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());
}

TEST_F(EkfMagTest, velocityRotationOnYawReset)
{
	// GIVEN: Mag fusion is active and vehicle is flying with airspeed
	const float initial_mag_heading = M_PI_F / 4.f; // 45 degrees
	Vector3f mag_data(0.2f * cosf(initial_mag_heading), -0.2f * sinf(initial_mag_heading), 0.4f);
	_sensor_simulator._mag.setData(mag_data);
	_sensor_simulator.runSeconds(_init_duration_s);

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_ekf->set_is_fixed_wing(true);

	const float airspeed_body = 15.0f; // 15 m/s airspeed in body X direction
	_sensor_simulator.startAirspeedSensor();
	_sensor_simulator._airspeed.setData(airspeed_body, airspeed_body);

	_ekf_wrapper.enableBetaFusion();
	_sensor_simulator.runSeconds(3);

	// initial state
	const Vector3f vel_before = _ekf->getVelocity();
	const float yaw_before = _ekf_wrapper.getYawAngle();
	const matrix::Dcm2f R_ned_to_body_before(-yaw_before);
	const Vector2f vel_body_before = R_ned_to_body_before * Vector2f(vel_before);

	// WHEN: Mag heading suddenly changes by more than 0.3 rad (90 degrees)
	const float new_mag_heading = yaw_before + M_PI_F / 2.f;
	mag_data = Vector3f(0.2f * cosf(new_mag_heading), -0.2f * sinf(new_mag_heading), 0.4f);
	_sensor_simulator._mag.setData(mag_data);
	_sensor_simulator.runSeconds(8.f);

	// THEN: the yaw should be reset to the new mag heading
	const float yaw_after = _ekf_wrapper.getYawAngle();
	EXPECT_NEAR(yaw_after, new_mag_heading, radians(5.0f))
			<< "Yaw after: " << degrees(yaw_after)
			<< " Expected: " << degrees(new_mag_heading);

	// AND: the NED velocity should be rotated to maintain consistent body-frame velocity
	const Vector3f vel_after = _ekf->getVelocity();

	// Calculate body-frame velocity after reset
	const matrix::Dcm2f R_ned_to_body_after(-yaw_after);
	const Vector2f vel_body_after = R_ned_to_body_after * Vector2f(vel_after);

	// Body-frame velocity should remain approximately the same
	EXPECT_NEAR(vel_body_before(0), vel_body_after(0), 1.0f)
			<< "Body-frame velocity X before: " << vel_body_before(0)
			<< " after: " << vel_body_after(0);
	EXPECT_NEAR(vel_body_before(1), vel_body_after(1), 1.0f)
			<< "Body-frame velocity Y before: " << vel_body_before(1)
			<< " after: " << vel_body_after(1);

	// Verify that the yaw change was sufficient to trigger velocity rotation (> 0.3 rad)
	const float yaw_change = fabsf(wrap_pi(yaw_after - yaw_before));
	EXPECT_GT(yaw_change, 0.3f) << "Yaw change: " << degrees(yaw_change) << " deg";
}
