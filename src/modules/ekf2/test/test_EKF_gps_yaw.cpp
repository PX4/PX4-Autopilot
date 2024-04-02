/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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
 * Test the gps yaw fusion
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"

class EkfGpsHeadingTest : public ::testing::Test
{
public:

	EkfGpsHeadingTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	void runConvergenceScenario(float yaw_offset_rad = 0.f, float antenna_offset_rad = 0.f);
	void checkConvergence(float truth, float tolerance = FLT_EPSILON);

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// Init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_sensor_simulator.runSeconds(_init_duration_s);
		_sensor_simulator._gps.setYaw(NAN);
		_sensor_simulator.runSeconds(2);
		_ekf_wrapper.enableGpsFusion();
		_ekf_wrapper.enableGpsHeadingFusion();
		_sensor_simulator.startGps();
		_sensor_simulator.runSeconds(11);
	}

	const uint32_t _init_duration_s{4};
};

void EkfGpsHeadingTest::runConvergenceScenario(float yaw_offset_rad, float antenna_offset_rad)
{
	// GIVEN: an initial GPS yaw, not aligned with the current one
	// The yaw antenna offset has already been corrected in the driver
	float gps_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle());

	_sensor_simulator._gps.setYaw(gps_heading); // used to remove the correction to fuse the real measurement
	_sensor_simulator._gps.setYawOffset(antenna_offset_rad);

	// WHEN: the GPS yaw fusion is activated
	_ekf_wrapper.enableGpsHeadingFusion();
	_sensor_simulator.runSeconds(5);

	// THEN: the estimate is reset and stays close to the measurement
	checkConvergence(gps_heading, 0.01f);
}

void EkfGpsHeadingTest::checkConvergence(float truth, float tolerance_deg)
{
	const float yaw_est = _ekf_wrapper.getYawAngle();
	EXPECT_LT(fabsf(matrix::wrap_pi(yaw_est - truth)), math::radians(tolerance_deg))
			<< "yaw est: " << math::degrees(yaw_est) << "gps yaw: " << math::degrees(truth);
}

TEST_F(EkfGpsHeadingTest, fusionStartWithReset)
{
	// GIVEN:EKF that fuses GPS

	// WHEN: enabling GPS heading fusion and heading difference is bigger than 15 degrees
	const float gps_heading = _ekf_wrapper.getYawAngle() + math::radians(20.f);
	_sensor_simulator._gps.setYaw(gps_heading);
	_ekf_wrapper.enableGpsHeadingFusion();
	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.runSeconds(0.4);

	// THEN: GPS heading fusion should have started;
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeadingFusion());

	// AND: a reset to GPS heading is performed
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), gps_heading, 0.001);

	// WHEN: GPS heading is disabled
	_sensor_simulator._gps.stop();
	_sensor_simulator.runSeconds(11);

	// THEN: after a while the fusion should be stopped
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsHeadingFusion());
}

TEST_F(EkfGpsHeadingTest, yawConvergence)
{
	// GIVEN: an initial GPS yaw, not aligned with the current one
	const float initial_yaw = math::radians(10.f);
	float gps_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle() + initial_yaw);

	_sensor_simulator._gps.setYaw(gps_heading);

	// WHEN: the GPS yaw fusion is activated
	_ekf_wrapper.enableGpsHeadingFusion();
	_sensor_simulator.runSeconds(5);

	// THEN: the estimate is reset and stays close to the measurement
	checkConvergence(gps_heading, 0.05f);

	// AND WHEN: the the measurement changes
	gps_heading += math::radians(2.f);
	_sensor_simulator._gps.setYaw(gps_heading);
	_sensor_simulator.runSeconds(20);

	// THEN: the estimate slowly converges to the new measurement
	// Note that the process is slow, because the gyro did not detect any motion
	checkConvergence(gps_heading, 0.5f);
}

TEST_F(EkfGpsHeadingTest, yaw0)
{
	runConvergenceScenario();
}

TEST_F(EkfGpsHeadingTest, yaw60)
{
	const float yaw_offset_rad = math::radians(60.f);
	const float antenna_offset_rad = math::radians(80.f);
	runConvergenceScenario(yaw_offset_rad, antenna_offset_rad);
}

TEST_F(EkfGpsHeadingTest, yaw180)
{
	const float yaw_offset_rad = math::radians(180.f);
	const float antenna_offset_rad = math::radians(-20.f);
	runConvergenceScenario(yaw_offset_rad, antenna_offset_rad);
}

TEST_F(EkfGpsHeadingTest, yawMinus120)
{
	const float yaw_offset_rad = math::radians(120.f);
	const float antenna_offset_rad = math::radians(-42.f);
	runConvergenceScenario(yaw_offset_rad, antenna_offset_rad);
}

TEST_F(EkfGpsHeadingTest, yawMinus30)
{
	const float yaw_offset_rad = math::radians(-30.f);
	const float antenna_offset_rad = math::radians(10.f);
	runConvergenceScenario(yaw_offset_rad, antenna_offset_rad);
}

TEST_F(EkfGpsHeadingTest, fallBackToMag)
{
	// GIVEN: an initial GPS yaw, not aligned with the current one
	// GPS yaw is expected to arrive a bit later, first feed some NANs
	// to the filter
	_sensor_simulator.runSeconds(6);
	float gps_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle() + math::radians(10.f));
	_sensor_simulator._gps.setYaw(gps_heading);

	// WHEN: the GPS yaw fusion is activated
	_sensor_simulator.runSeconds(1);

	// THEN: GPS heading fusion should have started, and mag
	// fusion should be disabled
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());

	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();

	// BUT WHEN: the GPS yaw is suddenly invalid
	gps_heading = NAN;
	_sensor_simulator._gps.setYaw(gps_heading);
	_sensor_simulator.runSeconds(7.5);

	// THEN: after a few seconds, the fusion should stop and
	// the estimator should fall back to mag fusion
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsHeadingFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
}

TEST_F(EkfGpsHeadingTest, fallBackToYawEmergencyEstimator)
{
	// GIVEN: an initial GPS yaw, not aligned with the current one (e.g.: wrong orientation of the antenna array) and no mag.
	_ekf_wrapper.setMagFuseTypeNone();
	_sensor_simulator.runSeconds(6);

	float gps_heading = math::radians(90.f);
	const float true_heading = math::radians(-20.f);

	_sensor_simulator._gps.setYaw(gps_heading);
	_sensor_simulator.runSeconds(10);

	const Vector3f accel_frd{-1.0, -1.5f, 0.f};
	_sensor_simulator._imu.setAccelData(accel_frd + Vector3f(0.f, 0.f, -CONSTANTS_ONE_G));
	const float dt = 0.5f;
	const Dcmf R_to_earth{Eulerf(0.f, 0.f, true_heading)};

	// needed to record takeoff time
	_ekf->set_in_air_status(false);
	_ekf->set_in_air_status(true);

	// WHEN: The drone starts to accelerate
	Vector3f simulated_velocity{};

	for (int i = 0; i < 10; i++) {
		_sensor_simulator.runSeconds(dt);

		const Vector3f accel_ned = R_to_earth * accel_frd;

		simulated_velocity += accel_ned * dt;
		_sensor_simulator._gps.setVelocity(simulated_velocity);
	}

	// THEN: the yaw emergency detects the yaw issue,
	// the GNSS yaw aiding is stopped and the heading
	// is reset to the emergency yaw estimate
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());

	checkConvergence(true_heading, 5.f);
}

TEST_F(EkfGpsHeadingTest, yawJmpOnGround)
{
	// GIVEN: the GPS yaw fusion activated
	float gps_heading = _ekf_wrapper.getYawAngle();
	_sensor_simulator._gps.setYaw(gps_heading);
	_sensor_simulator.runSeconds(1);
	_ekf->set_in_air_status(false);

	// WHEN: the measurement suddenly changes
	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	gps_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle() + math::radians(45.f));
	_sensor_simulator._gps.setYaw(gps_heading);
	_sensor_simulator.runSeconds(8);

	// THEN: the fusion should reset
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeadingFusion());
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
	EXPECT_LT(fabsf(matrix::wrap_pi(_ekf_wrapper.getYawAngle() - gps_heading)), math::radians(1.f));
}

TEST_F(EkfGpsHeadingTest, yawJumpInAir)
{
	// GIVEN: the GPS yaw fusion activated
	float gps_heading = _ekf_wrapper.getYawAngle();
	_sensor_simulator._gps.setYaw(gps_heading);
	_sensor_simulator.runSeconds(5);
	_ekf->set_in_air_status(true);

	// WHEN: the measurement suddenly changes
	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	gps_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle() + math::radians(180.f));
	_sensor_simulator._gps.setYaw(gps_heading);
	_sensor_simulator.runSeconds(7.5);

	// THEN: the fusion should reset
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);

	// BUT WHEN: the measurement jumps a 2nd time
	gps_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle() + math::radians(180.f));
	_sensor_simulator._gps.setYaw(gps_heading);
	_sensor_simulator.runSeconds(7.5);

	// THEN: after a few seconds, the fusion should stop and
	// the estimator doesn't fall back to mag fusion because it has
	// been declared inconsistent with the filter states
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isMagHeadingConsistent());
	//TODO: should we force a reset to mag if the GNSS yaw fusion was forced to stop?
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
}

TEST_F(EkfGpsHeadingTest, stopOnGround)
{
	// GIVEN: the GPS yaw fusion activated and there is no mag data
	_sensor_simulator._mag.stop();
	float gps_heading = _ekf_wrapper.getYawAngle();
	_sensor_simulator._gps.setYaw(gps_heading);
	_sensor_simulator.runSeconds(5);

	// WHEN: the measurement stops
	gps_heading = NAN;
	_sensor_simulator._gps.setYaw(gps_heading);
	_sensor_simulator.runSeconds(7.5);

	// THEN: the fusion should stop and the GPS pos/vel aiding
	// should stop as well because the yaw is not aligned anymore
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsFusion());

	// AND IF: the mag fusion type is set to NONE
	_ekf_wrapper.setMagFuseTypeNone();

	// WHEN: running without yaw aiding
	const float yaw_variance_before = _ekf->getYawVar();
	_sensor_simulator.runSeconds(20.0);

	// THEN: the yaw variance increases
	EXPECT_GT(_ekf->getYawVar(), yaw_variance_before);
}
