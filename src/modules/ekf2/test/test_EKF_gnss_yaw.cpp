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
 * Test the gnss yaw fusion
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"

class EkfGnssHeadingTest : public ::testing::Test
{
public:

	EkfGnssHeadingTest(): ::testing::Test(),
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
		_sensor_simulator._gnss_yaw.setYaw(NAN);
		_sensor_simulator.runSeconds(2);
		_ekf_wrapper.enableGnssFusion();
		_ekf_wrapper.enableGnssHeadingFusion();
		_sensor_simulator.startGnss();
		_sensor_simulator.startGnssYaw();
		_sensor_simulator.runSeconds(11);
	}

	const uint32_t _init_duration_s{4};
};

void EkfGnssHeadingTest::runConvergenceScenario(float yaw_offset_rad, float antenna_offset_rad)
{
	// GIVEN: an initial GNSS yaw, not aligned with the current one
	// The yaw antenna offset has already been corrected in the driver
	float gnss_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle());

	_sensor_simulator._gnss_yaw.setYaw(gnss_heading); // used to remove the correction to fuse the real measurement
	_sensor_simulator._gnss_yaw.setYawOffset(antenna_offset_rad);

	// WHEN: the GNSS yaw fusion is activated
	_ekf_wrapper.enableGnssHeadingFusion();
	_sensor_simulator.runSeconds(5);

	// THEN: the estimate is reset and stays close to the measurement
	checkConvergence(gnss_heading, 0.01f);
}

void EkfGnssHeadingTest::checkConvergence(float truth, float tolerance_deg)
{
	const float yaw_est = _ekf_wrapper.getYawAngle();
	EXPECT_LT(fabsf(matrix::wrap_pi(yaw_est - truth)), math::radians(tolerance_deg))
			<< "yaw est: " << math::degrees(yaw_est) << "gps yaw: " << math::degrees(truth);
}

TEST_F(EkfGnssHeadingTest, fusionStartWithReset)
{
	// GIVEN:EKF that fuses GNSS

	// WHEN: enabling GNSS heading fusion and heading difference is bigger than 15 degrees
	const float gnss_heading = _ekf_wrapper.getYawAngle() + math::radians(20.f);
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
	_ekf_wrapper.enableGnssHeadingFusion();
	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.runSeconds(0.4);

	// THEN: GNSS heading fusion should have started;
	EXPECT_TRUE(_ekf_wrapper.isIntendingGnssHeadingFusion());

	// AND: a reset to GNSS heading is performed
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
	EXPECT_NEAR(_ekf_wrapper.getYawAngle(), gnss_heading, 0.001);

	// WHEN: GNSS heading is disabled
	_sensor_simulator.stopGnssYaw();
	_sensor_simulator.runSeconds(11);

	// THEN: after a while the fusion should be stopped
	EXPECT_FALSE(_ekf_wrapper.isIntendingGnssHeadingFusion());
}

TEST_F(EkfGnssHeadingTest, yawConvergence)
{
	// GIVEN: an initial GNSS yaw, not aligned with the current one
	const float initial_yaw = math::radians(10.f);
	float gnss_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle() + initial_yaw);

	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);

	// WHEN: the GNSS yaw fusion is activated
	_ekf_wrapper.enableGnssHeadingFusion();
	_sensor_simulator.runSeconds(5);

	// THEN: the estimate is reset and stays close to the measurement
	checkConvergence(gnss_heading, 0.05f);

	// AND WHEN: the the measurement changes
	gnss_heading += math::radians(2.f);
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
	_sensor_simulator.runSeconds(20);

	// THEN: the estimate slowly converges to the new measurement
	// Note that the process is slow, because the gyro did not detect any motion
	checkConvergence(gnss_heading, 0.5f);
}

TEST_F(EkfGnssHeadingTest, yaw0)
{
	runConvergenceScenario();
}

TEST_F(EkfGnssHeadingTest, yaw60)
{
	const float yaw_offset_rad = math::radians(60.f);
	const float antenna_offset_rad = math::radians(80.f);
	runConvergenceScenario(yaw_offset_rad, antenna_offset_rad);
}

TEST_F(EkfGnssHeadingTest, yaw180)
{
	const float yaw_offset_rad = math::radians(180.f);
	const float antenna_offset_rad = math::radians(-20.f);
	runConvergenceScenario(yaw_offset_rad, antenna_offset_rad);
}

TEST_F(EkfGnssHeadingTest, yawMinus120)
{
	const float yaw_offset_rad = math::radians(120.f);
	const float antenna_offset_rad = math::radians(-42.f);
	runConvergenceScenario(yaw_offset_rad, antenna_offset_rad);
}

TEST_F(EkfGnssHeadingTest, yawMinus30)
{
	const float yaw_offset_rad = math::radians(-30.f);
	const float antenna_offset_rad = math::radians(10.f);
	runConvergenceScenario(yaw_offset_rad, antenna_offset_rad);
}

TEST_F(EkfGnssHeadingTest, fallBackToMag)
{
	// GIVEN: an initial GNSS yaw, not aligned with the current one
	// GNSS yaw is expected to arrive a bit later, first feed some NANs
	// to the filter
	_sensor_simulator.runSeconds(6);
	float gnss_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle() + math::radians(10.f));
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);

	// WHEN: the GNSS yaw fusion is activated
	_sensor_simulator.runSeconds(1);

	// THEN: GNSS heading fusion should have started, and mag
	// fusion should be disabled
	EXPECT_TRUE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMag3DFusion());

	//const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();

	// BUT WHEN: the GNSS yaw is suddenly invalid
	gnss_heading = NAN;
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
	_sensor_simulator.runSeconds(7.5);

	// THEN: after a few seconds, the fusion should stop and
	// the estimator should fall back to mag fusion
	EXPECT_FALSE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	//EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
}

TEST_F(EkfGnssHeadingTest, fallBackToYawEmergencyEstimator)
{
	// GIVEN: an initial GNSS yaw, not aligned with the current one (e.g.: wrong orientation of the antenna array) and no mag.
	_ekf_wrapper.setMagFuseTypeNone();
	_sensor_simulator.runSeconds(6);

	float gnss_heading = math::radians(90.f);
	const float true_heading = math::radians(-20.f);

	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
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
		_sensor_simulator._gnss.setVelocity(simulated_velocity);
	}

	// THEN: the yaw emergency detects the yaw issue,
	// the GNSS yaw aiding is stopped and the heading
	// is reset to the emergency yaw estimate
	EXPECT_FALSE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGnssFusion());

	checkConvergence(true_heading, 5.f);
}

TEST_F(EkfGnssHeadingTest, yawJmpOnGround)
{
	// GIVEN: the GNSS yaw fusion activated
	float gnss_heading = _ekf_wrapper.getYawAngle();
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
	_sensor_simulator.runSeconds(1);
	_ekf->set_in_air_status(false);

	// WHEN: the measurement suddenly changes
	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	gnss_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle() + math::radians(45.f));
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
	_sensor_simulator.runSeconds(8);

	// THEN: the fusion should stop, reset to mag
	EXPECT_FALSE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);

	// AND WHEN: less than the GNSS health time (10s) has passed since the fusion failed
	_sensor_simulator.runSeconds(5);

	// THEN: the heading is not trusted for a reset yet
	EXPECT_FALSE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);

	// AND WHEN: the health time has passed
	_sensor_simulator.runSeconds(6);

	// THEN: GNSS yaw fusion restarts with a reset to the heading
	EXPECT_TRUE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 2);
	EXPECT_LT(fabsf(matrix::wrap_pi(_ekf_wrapper.getYawAngle() - gnss_heading)), math::radians(1.f));
}

TEST_F(EkfGnssHeadingTest, yawJumpInAir)
{
	// GIVEN: the GNSS yaw fusion activated
	float gnss_heading = _ekf_wrapper.getYawAngle();
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading + math::radians(90.f));
	_sensor_simulator.runSeconds(5);
	_ekf->set_in_air_status(true);

	// WHEN: the measurement suddenly changes
	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	gnss_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle() + math::radians(180.f));
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
	_sensor_simulator.runSeconds(7.5);

	// THEN: the fusion should not reset as heading is still observable through GNSS vel/pos fusion
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter);

	// THEN: after a few seconds, the fusion should stop and
	// the estimator doesn't fall back to mag fusion because it has
	// been declared inconsistent with the filter states
	EXPECT_FALSE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	EXPECT_FALSE(_ekf_wrapper.isMagHeadingConsistent());
	EXPECT_FALSE(_ekf_wrapper.isIntendingMagHeadingFusion());
}

TEST_F(EkfGnssHeadingTest, stopOnGround)
{
	// GIVEN: the GNSS yaw fusion activated and there is no mag data
	_sensor_simulator._mag.stop();
	float gnss_heading = _ekf_wrapper.getYawAngle();
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
	_sensor_simulator.runSeconds(5);

	// WHEN: the measurement stops
	gnss_heading = NAN;
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
	_sensor_simulator.runSeconds(7.5);

	// THEN: the fusion should stop and the GNSS pos/vel aiding
	// should stop as well because the yaw is not aligned anymore
	EXPECT_FALSE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	//EXPECT_FALSE(_ekf_wrapper.isIntendingGnssFusion());

	// AND IF: the mag fusion type is set to NONE
	_ekf_wrapper.setMagFuseTypeNone();

	// WHEN: running without yaw aiding
	const float yaw_variance_before = _ekf->getYawVar();
	_sensor_simulator.runSeconds(20.0);

	// THEN: the yaw variance increases
	EXPECT_GT(_ekf->getYawVar(), yaw_variance_before);
}

TEST_F(EkfGnssHeadingTest, continuesWithoutPosition)
{
	// GIVEN: GNSS yaw fusion is active
	const float gnss_heading = _ekf_wrapper.getYawAngle();
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
	_sensor_simulator.runSeconds(2);
	EXPECT_TRUE(_ekf_wrapper.isIntendingGnssHeadingFusion());

	// WHEN: position/velocity samples stop but heading keeps arriving
	_sensor_simulator.stopGnss();
	const uint64_t time_gnss_stopped = _sensor_simulator.getTime();
	_sensor_simulator.runSeconds(4);

	// THEN: the heading is still fused, it does not wait for a position sample
	EXPECT_TRUE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	EXPECT_GT(_ekf->aid_src_gnss_yaw().time_last_fuse, time_gnss_stopped);

	// AND WHEN: the position data timeout stops position and velocity fusion
	_sensor_simulator.runSeconds(4);
	EXPECT_FALSE(_ekf_wrapper.isIntendingGnssFusion());

	// THEN: the heading is still fused
	EXPECT_TRUE(_ekf_wrapper.isIntendingGnssHeadingFusion());
}

TEST_F(EkfGnssHeadingTest, startsWhilePositionChecksFail)
{
	// GIVEN: a position solution failing the checks; on the ground they need 10s without a failure to pass again
	_sensor_simulator._gnss.setNumberOfSatellites(3);
	_sensor_simulator.runSeconds(1);
	EXPECT_FALSE(_ekf->gnss_checks_passed());

	// AND: a good heading
	const float gnss_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle() + math::radians(20.f));
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();

	// WHEN: running on it
	_sensor_simulator.runSeconds(1);

	// THEN: the heading is used without waiting for the position checks
	EXPECT_FALSE(_ekf->gnss_checks_passed());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
	checkConvergence(gnss_heading, 0.5f);
}

TEST_F(EkfGnssHeadingTest, continuesWhenPositionQualityPoor)
{
	// GIVEN: GNSS yaw, position and velocity fusion active
	const float gnss_heading = _ekf_wrapper.getYawAngle();
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
	_sensor_simulator.runSeconds(2);
	EXPECT_TRUE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGnssFusion());

	// WHEN: the position solution fails the checks for longer than the fusion timeout
	_sensor_simulator._gnss.setNumberOfSatellites(3);
	_sensor_simulator.runSeconds(8);

	// THEN: position and velocity fusion stop but the heading is still fused
	EXPECT_FALSE(_ekf_wrapper.isIntendingGnssFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGnssHeadingFusion());
}

TEST_F(EkfGnssHeadingTest, unusableHeading)
{
	// GIVEN: a good heading that the sensors module marks unusable, as its receiver reports spoofing
	const float gnss_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle() + math::radians(20.f));
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
	_sensor_simulator._gnss_yaw.setUsable(false);
	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.runSeconds(4);

	// THEN: the heading is not used
	EXPECT_FALSE(_ekf_wrapper.isIntendingGnssHeadingFusion());

	// WHEN: the heading becomes usable again
	_sensor_simulator._gnss_yaw.setUsable(true);
	_sensor_simulator.runSeconds(5);

	// THEN: the heading is trusted for a reset only after EKF2_REQ_GPS_H (10 s)
	EXPECT_FALSE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	_sensor_simulator.runSeconds(6);
	EXPECT_TRUE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
	checkConvergence(gnss_heading, 0.5f);

	// WHEN: the heading becomes unusable while it is fused
	_sensor_simulator._gnss_yaw.setUsable(false);
	_sensor_simulator.runSeconds(8);

	// THEN: like position fusion, the fusion stops after the reset timeout
	EXPECT_FALSE(_ekf_wrapper.isIntendingGnssHeadingFusion());
}

TEST_F(EkfGnssHeadingTest, inaccurateHeadingDoesNotReset)
{
	// GIVEN: a receiver still resolving its baseline, reporting a heading 30 deg off with a 1 rad accuracy
	const float gnss_heading = matrix::wrap_pi(_ekf_wrapper.getYawAngle() + math::radians(30.f));
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
	_sensor_simulator._gnss_yaw.setYawAccuracy(1.f);
	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_sensor_simulator.runSeconds(4);

	// THEN: GNSS yaw fusion doesn't start, so yaw isn't reset to it
	EXPECT_FALSE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter);

	// WHEN: the reported accuracy drops below 15 deg
	const float yaw_acc = math::radians(10.f);
	_sensor_simulator._gnss_yaw.setYawAccuracy(yaw_acc);

	for (int i = 0; (i < 100) && (_ekf_wrapper.getQuaternionResetCounter() == initial_quat_reset_counter); i++) {
		_sensor_simulator.runMicroseconds(10000);
	}

	// THEN: fusion starts with a reset to the heading, as uncertain as the reported accuracy
	EXPECT_TRUE(_ekf_wrapper.isIntendingGnssHeadingFusion());
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);
	checkConvergence(gnss_heading, 0.5f);
	EXPECT_NEAR(_ekf->getYawVar(), yaw_acc * yaw_acc, 0.1f * yaw_acc * yaw_acc);
}

TEST_F(EkfGnssHeadingTest, fusesAtOwnRate)
{
	// GIVEN: GNSS yaw fusion active with heading arriving faster than position
	_sensor_simulator._gnss_yaw.setRateHz(10);
	const float gnss_heading = _ekf_wrapper.getYawAngle();
	_sensor_simulator._gnss_yaw.setYaw(gnss_heading);
	_sensor_simulator.runSeconds(2);
	EXPECT_TRUE(_ekf_wrapper.isIntendingGnssHeadingFusion());

	// WHEN: running for one second
	int fusion_count = 0;
	uint64_t time_last_fuse = _ekf->aid_src_gnss_yaw().time_last_fuse;

	for (int i = 0; i < 100; i++) {
		_sensor_simulator.runMicroseconds(10000);

		if (_ekf->aid_src_gnss_yaw().time_last_fuse != time_last_fuse) {
			time_last_fuse = _ekf->aid_src_gnss_yaw().time_last_fuse;
			fusion_count++;
		}
	}

	// THEN: every heading sample is fused, not only the ones coinciding with a 5 Hz position sample
	EXPECT_GE(fusion_count, 9);
}
