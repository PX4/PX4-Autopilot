/****************************************************************************
 *
 *   Copyright (c) 2019 ECL Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <math.h>
#include <memory>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"

class EkfInitializationTest : public ::testing::Test {
 public:
	EkfInitializationTest(): ::testing::Test(),
	_ekf{std::make_shared<Ekf>()},
	_sensor_simulator(_ekf),
	_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	const float _init_tilt_period = 0.5; // seconds

	// GTests is calling this
	void SetUp() override
	{
		_ekf->init(0);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}

	void initializedOrienationIsMatchingGroundTruth(Quatf true_quaternion)
	{
		Quatf quat_est = _ekf_wrapper.getQuaternion();
		EXPECT_TRUE(matrix::isEqual(quat_est, true_quaternion));
	}

	void validStateAfterOrientationInitialization()
	{
		quaternionVarianceBigEnoughAfterOrientationInitialization();
		velocityAndPositionCloseToZero();
		velocityAndPositionVarianceBigEnoughAfterOrientationInitialization();
	}

	void quaternionVarianceBigEnoughAfterOrientationInitialization()
	{
		const matrix::Vector<float, 4> quat_variance = _ekf_wrapper.getQuaternionVariance();
		const float quat_variance_limit = 0.0001f;
		EXPECT_TRUE(quat_variance(1) > quat_variance_limit) << "quat_variance(1)" << quat_variance(1);
		EXPECT_TRUE(quat_variance(2) > quat_variance_limit) << "quat_variance(2)" << quat_variance(2);
		EXPECT_TRUE(quat_variance(3) > quat_variance_limit) << "quat_variance(3)" << quat_variance(3);
	}

	void velocityAndPositionCloseToZero()
	{
		Vector3f pos = _ekf_wrapper.getPosition();
		Vector3f vel = _ekf_wrapper.getVelocity();

		EXPECT_TRUE(matrix::isEqual(pos, Vector3f{}, 0.001f));
		EXPECT_TRUE(matrix::isEqual(vel, Vector3f{}, 0.001f));
	}

	void velocityAndPositionVarianceBigEnoughAfterOrientationInitialization()
	{
		Vector3f pos_var = _ekf_wrapper.getPositionVariance();
		Vector3f vel_var = _ekf_wrapper.getVelocityVariance();

		const float pos_variance_limit = 0.2f;
		EXPECT_TRUE(pos_var(0) > pos_variance_limit) << "pos_var(1)" << pos_var(0);
		EXPECT_TRUE(pos_var(1) > pos_variance_limit) << "pos_var(2)" << pos_var(1);
		EXPECT_TRUE(pos_var(2) > pos_variance_limit) << "pos_var(3)" << pos_var(2);

		const float vel_variance_limit = 0.4f;
		EXPECT_TRUE(vel_var(0) > vel_variance_limit) << "vel_var(1)" << vel_var(0);
		EXPECT_TRUE(vel_var(1) > vel_variance_limit) << "vel_var(2)" << vel_var(1);
		EXPECT_TRUE(vel_var(2) > vel_variance_limit) << "vel_var(3)" << vel_var(2);
	}
};

TEST_F(EkfInitializationTest, initializeWithZeroTilt)
{
	const float pitch = math::radians(0.0f);
	const float roll = math::radians(0.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_sensor_simulator.simulateOrientation(quat_sim);
	_sensor_simulator.runSeconds(_init_tilt_period);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	validStateAfterOrientationInitialization();
}

TEST_F(EkfInitializationTest, initializeHeadingWithZeroTilt)
{
	const float pitch = math::radians(0.0f);
	const float roll = math::radians(0.0f);
	const float yaw = math::radians(90.0f);
	const Eulerf euler_angles_sim(roll, pitch, yaw);
	const Quatf quat_sim(euler_angles_sim);

	_sensor_simulator.simulateOrientation(quat_sim);
	_sensor_simulator.runSeconds(_init_tilt_period);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	validStateAfterOrientationInitialization();
}

TEST_F(EkfInitializationTest, initializeWithTilt)
{
	const float pitch = math::radians(30.0f);
	const float roll = math::radians(60.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_sensor_simulator.simulateOrientation(quat_sim);
	_sensor_simulator.runSeconds(_init_tilt_period);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	validStateAfterOrientationInitialization();
}

TEST_F(EkfInitializationTest, initializeWithPitch90)
{
	const Quatf quat_sim(0.0f, 0.7071068f, 0.0f, 0.7071068f);

	_sensor_simulator.simulateOrientation(quat_sim);
	_sensor_simulator.runSeconds(_init_tilt_period);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	// TODO: Quaternion Variance is smaller in this case than in the other cases
	validStateAfterOrientationInitialization();
}

TEST_F(EkfInitializationTest, initializeWithRoll90)
{
	const Quatf quat_sim(0.7071068f, 0.7071068f, 0.0f, 0.0f);

	_sensor_simulator.simulateOrientation(quat_sim);
	_sensor_simulator.runSeconds(_init_tilt_period);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	validStateAfterOrientationInitialization();
}
