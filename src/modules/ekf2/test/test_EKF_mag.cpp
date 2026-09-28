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

TEST_F(EkfMagTest, manualYawExpiresWhenHeadingIsFused)
{
	// GIVEN: mag fusion is aligned in flight, so the mag remains the heading source
	// even after a manual heading reset
	const float mag_heading = M_PI_F / 4.f;
	_sensor_simulator._mag.setData(Vector3f(0.2f * cosf(mag_heading), -0.2f * sinf(mag_heading), 0.4f));
	_sensor_simulator.runSeconds(_init_duration_s);

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator._rng.setData(5.f, 100);
	_sensor_simulator.startRangeFinder();
	_sensor_simulator.runSeconds(5.f);

	ASSERT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion() || _ekf_wrapper.isIntendingMag3DFusion());
	ASSERT_FALSE(_ekf->control_status_flags().yaw_manual);

	// WHEN: the heading is set manually
	_ekf->resetHeadingToExternalObservation(mag_heading + radians(30.f), radians(2.f));
	_sensor_simulator.runSeconds(1.f);

	// THEN: the manual heading is protected from being overridden by the mag
	EXPECT_TRUE(_ekf->control_status_flags().yaw_manual);

	// AND WHEN: the mag keeps being fused as heading source for a long time
	_sensor_simulator.runSeconds(20.f);
	EXPECT_TRUE(_ekf->control_status_flags().yaw_manual);

	_sensor_simulator.runSeconds(15.f);

	// THEN: the heading is mag derived again and the manual reset no longer protected
	EXPECT_FALSE(_ekf->control_status_flags().yaw_manual);
}

TEST_F(EkfMagTest, manualYawSurvivesWithGnssAidingOnly)
{
	// GIVEN: the vehicle sits on the ground with GNSS velocity and position fused
	const float mag_heading = M_PI_F / 4.f;
	_sensor_simulator._mag.setData(Vector3f(0.2f * cosf(mag_heading), -0.2f * sinf(mag_heading), 0.4f));
	_sensor_simulator.runSeconds(_init_duration_s);

	_ekf->set_min_required_gps_health_time(1e6);
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(10.f);

	ASSERT_TRUE(_ekf->control_status_flags().gnss_vel);
	ASSERT_TRUE(_ekf->control_status_flags().gnss_pos);

	// WHEN: the heading is set manually, which stops the mag from being used as heading source
	_ekf->resetHeadingToExternalObservation(mag_heading + radians(30.f), radians(2.f));
	_sensor_simulator.runSeconds(2.f);

	ASSERT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	ASSERT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());

	// THEN: GNSS aiding alone does not observe the heading of a vehicle that is not
	// accelerating laterally, so the manual heading is kept indefinitely
	_sensor_simulator.runSeconds(60.f);
	EXPECT_TRUE(_ekf->control_status_flags().yaw_manual);
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), mag_heading + radians(30.f), radians(2.f));
}

TEST_F(EkfMagTest, manualYawOnGroundBlocksMagHeadingFusionIndefinitely)
{
	// GIVEN: the mag is the heading source of a vehicle that has not taken off yet
	const float mag_heading = M_PI_F / 4.f;
	_sensor_simulator._mag.setData(Vector3f(0.2f * cosf(mag_heading), -0.2f * sinf(mag_heading), 0.4f));
	_sensor_simulator.runSeconds(_init_duration_s);

	ASSERT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());

	// WHEN: the heading is set manually while still on the ground
	const float manual_heading = mag_heading + radians(30.f);
	_ekf->resetHeadingToExternalObservation(manual_heading, radians(2.f));
	_sensor_simulator.runSeconds(2.f);

	// THEN: the mag is no longer used as heading source, because in-flight mag alignment is the
	// only condition that lets mag heading fusion resume after a manual heading reset
	EXPECT_TRUE(_ekf->control_status_flags().yaw_manual);
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());

	// AND: this does not resolve itself while the vehicle stays on the ground. No heading
	// observation is fused, so the manual heading never expires, which in turn keeps mag
	// heading fusion disabled.
	_sensor_simulator.runSeconds(120.f);

	EXPECT_TRUE(_ekf->control_status_flags().yaw_manual);
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());

	// AND: the manual heading is never overridden by the mag
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), manual_heading, radians(3.f));
	// AND: the missing in-flight alignment is the only thing holding heading fusion off; the mag
	// itself is still healthy and being fused into the field states
	EXPECT_FALSE(_ekf->control_status_flags().mag_aligned_in_flight);
	EXPECT_TRUE(_ekf->control_status_flags().mag);
	EXPECT_TRUE(_ekf->control_status_flags().yaw_align);
	EXPECT_FALSE(_ekf->control_status_flags().mag_fault);
	EXPECT_FALSE(_ekf->control_status_flags().mag_field_disturbed);
}

TEST_F(EkfMagTest, manualYawSurvivesTakeoffUntilHeadingFused)
{
	// GIVEN: the heading was set manually on the ground, before any in-flight mag alignment
	const float mag_heading = M_PI_F / 4.f;
	_sensor_simulator._mag.setData(Vector3f(0.2f * cosf(mag_heading), -0.2f * sinf(mag_heading), 0.4f));
	_sensor_simulator.runSeconds(_init_duration_s);

	const float manual_heading = mag_heading + radians(30.f);
	_ekf->resetHeadingToExternalObservation(manual_heading, radians(2.f));
	_sensor_simulator.runSeconds(2.f);

	ASSERT_TRUE(_ekf->control_status_flags().yaw_manual);
	ASSERT_FALSE(_ekf->control_status_flags().mag_aligned_in_flight);

	// WHEN: the vehicle takes off and climbs above the height of the ground mag anomalies,
	// which triggers the in-flight mag alignment
	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator._rng.setData(5.f, 100);
	_sensor_simulator.startRangeFinder();
	_sensor_simulator.runSeconds(5.f);

	// THEN: the mag becomes the heading source again, but the manual heading is neither
	// overridden by the alignment nor immediately declared stale by it
	EXPECT_TRUE(_ekf->control_status_flags().mag_aligned_in_flight);
	EXPECT_TRUE(_ekf_wrapper.isIntendingMag3DFusion());
	EXPECT_TRUE(_ekf->control_status_flags().yaw_manual);
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), manual_heading, radians(1.f));

	// AND WHEN: the mag has been driving the heading for long enough
	_sensor_simulator.runSeconds(20.f);
	EXPECT_TRUE(_ekf->control_status_flags().yaw_manual);

	_sensor_simulator.runSeconds(15.f);

	// THEN: the manual heading expires
	EXPECT_FALSE(_ekf->control_status_flags().yaw_manual);
}

TEST_F(EkfMagTest, magHeadingChangeRateLimited)
{
	// GIVEN: EKF just had its initial yaw alignment from mag, setting heading variance to
	// sq(ekf2_head_noise). At this point, a wrong mag measurement would cause a heading
	// jump without the rate limit.
	const Vector3f correct_mag(0.2f, 0.f, 0.4f);
	_sensor_simulator._mag.setData(correct_mag);

	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.runSeconds(1.f);

	ASSERT_GT(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter)
			<< "Initial yaw reset should have occurred";
	ASSERT_TRUE(_ekf_wrapper.isIntendingMagFusion());

	// WHEN: mag suddenly changes direction (within innovation gate, but enough to trigger the rate limit)
	const float wrong_angle = radians(15.f);
	const Vector3f wrong_mag(0.2f * cosf(wrong_angle), -0.2f * sinf(wrong_angle), 0.4f);
	_sensor_simulator._mag.setData(wrong_mag);

	// allow the EKF delay buffer to flush so wrong mag data starts being processed
	const float step_dt = 0.1f;
	_sensor_simulator.runSeconds(3.f * step_dt);

	// THEN: per-step heading change is bounded
	const float max_rate_rad_per_s = radians(1.f);
	float yaw_prev = _ekf_wrapper.getYawAngle();

	for (int i = 0; i < 10; i++) {
		_sensor_simulator.runSeconds(step_dt);
		const float yaw_curr = _ekf_wrapper.getYawAngle();
		const float rate = fabsf(wrap_pi(yaw_curr - yaw_prev)) / step_dt;
		EXPECT_NEAR(rate, max_rate_rad_per_s, radians(0.1f)) << "step " << i << ": heading rate "
				<< degrees(rate) << " deg/s";
		yaw_prev = yaw_curr;
	}

	// AND: heading eventually converges to the wrong heading despite being rate-limited
	_sensor_simulator.runSeconds(30.f);
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), wrong_angle, radians(4.f));
}

TEST_F(EkfMagTest, magFaultCleared)
{
	// GIVEN: biased mag data
	_sensor_simulator._mag.setBias(Vector3f(-0.3f, 0.2f, 0.f));
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(11);

	// THEN: the initial heading is incorrect
	EXPECT_NEAR(degrees(_ekf_wrapper.getYawAngle()), -110.f, 5.f);
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());

	// WHEN: motion allows the yaw estimator to converge
	_sensor_simulator.setTrajectoryTargetVelocity(Vector3f(2.f, -2.f, -1.f));
	_ekf->set_in_air_status(true);

	_sensor_simulator.runTrajectorySeconds(3.f);

	// THEN: the heading error is detected, solved and mag is declared faulty
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());
	EXPECT_TRUE(_ekf_wrapper.isMagFaultDetected());

	// BUT when: the mag disturbance is gone
	_sensor_simulator._mag.setBias(Vector3f());
	_sensor_simulator.setTrajectoryTargetVelocity(Vector3f(0.f, 0.f, 0.f));

	_sensor_simulator.runTrajectorySeconds(7.f);

	// THEN: the fault is cleared and mag fusion restarts
	EXPECT_FALSE(_ekf_wrapper.isMagFaultDetected());
	EXPECT_TRUE(_ekf_wrapper.isMagHeadingConsistent());
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion()); // because in-flight alignment is required
}
