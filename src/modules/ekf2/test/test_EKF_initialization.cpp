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

#include <gtest/gtest.h>
#include <math.h>
#include <memory>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"

class EkfInitializationTest : public ::testing::Test
{
public:
	EkfInitializationTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	const float _init_tilt_period = 0.7f; // seconds

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// first init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}

	void initializedOrienationIsMatchingGroundTruth(Quatf true_quaternion)
	{
		const Quatf quat_est = _ekf->getQuaternion();
		EXPECT_TRUE(matrix::isEqual(quat_est, true_quaternion))
				<< "quat est = " << quat_est(0) << ", " << quat_est(1) << ", "
				<< quat_est(2) << ", " << quat_est(3)
				<< "\nquat true = " << true_quaternion(0) << ", " << true_quaternion(1) << ", "
				<< true_quaternion(2) << ", " << true_quaternion(3);
	}

	void quaternionVarianceBigEnoughAfterOrientationInitialization(float quat_variance_limit = 0.00001f)
	{
		const matrix::Vector3f quat_variance = _ekf->getRotVarBody();
		EXPECT_TRUE(quat_variance(0) > quat_variance_limit) << "quat_variance(3): " << quat_variance(0);
		EXPECT_TRUE(quat_variance(1) > quat_variance_limit) << "quat_variance(1): " << quat_variance(1);
		EXPECT_TRUE(quat_variance(2) > quat_variance_limit) << "quat_variance(2): " << quat_variance(2);
	}

	void yawVarianceBigEnoughAfterHeadingReset()
	{
		// The yaw variance is smaller than its reset value as we do not probe instantly after the reset
		EXPECT_GT(sqrtf(_ekf->getYawVar()), _ekf_wrapper.getMagHeadingNoise() / 5.f);
	}

	void velocityAndPositionCloseToZero()
	{
		const Vector3f pos = _ekf->getPosition();
		const Vector3f vel = _ekf->getVelocity();

		EXPECT_TRUE(matrix::isEqual(pos, Vector3f{}, 0.002f))
				<< "pos = " << pos(0) << ", " << pos(1) << ", " << pos(2);
		EXPECT_TRUE(matrix::isEqual(vel, Vector3f{}, 0.003f))
				<< "vel = " << vel(0) << ", " << vel(1) << ", " << vel(2);
	}

	void velocityVarianceBigEnoughAfterOrientationInitialization(float vel_variance_limit)
	{
		const Vector3f vel_var = _ekf->getVelocityVariance();

		EXPECT_TRUE(vel_var(0) > vel_variance_limit) << "vel_var(0): " << vel_var(0);
		EXPECT_TRUE(vel_var(1) > vel_variance_limit) << "vel_var(1): " << vel_var(1);
		EXPECT_TRUE(vel_var(2) > vel_variance_limit) << "vel_var(2): " << vel_var(2);
	}

	void positionVarianceBigEnoughAfterOrientationInitialization(float pos_variance_limit)
	{
		const Vector3f pos_var = _ekf->getPositionVariance();

		EXPECT_TRUE(pos_var(0) > pos_variance_limit) << "pos_var(0): " << pos_var(0);
		EXPECT_TRUE(pos_var(1) > pos_variance_limit) << "pos_var(1): " << pos_var(1);
		EXPECT_TRUE(pos_var(2) > pos_variance_limit) << "pos_var(2): " << pos_var(2);
	}

	void learningCorrectAccelBias()
	{
		const Dcmf R_to_earth = Dcmf(_ekf->getQuaternion());
		const Vector3f accel_bias_var = _ekf->getAccelBiasVariance();
		const Vector3f accel_bias = _ekf->getAccelBias();

		for (int i = 0; i < 3; i++) {
			if (fabsf(R_to_earth(2, i)) > 0.8f) {
				// Highly observable, the variance decreases
				EXPECT_LT(accel_bias_var(i), 4.0e-2f) << "axis " << i;
			}

			EXPECT_LT(accel_bias(i), 4.0e-6f) << "axis " << i;
		}
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

	EXPECT_TRUE(_ekf->control_status_flags().vehicle_at_rest);
	EXPECT_FALSE(_ekf->control_status_flags().in_air);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);

	velocityAndPositionCloseToZero();

	// Fake position fusion obs var when at rest sq(0.01f)
	positionVarianceBigEnoughAfterOrientationInitialization(0.00001f);
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, initializeWithZeroTiltNotAtRest)
{
	const float pitch = math::radians(0.0f);
	const float roll = math::radians(0.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.simulateOrientation(quat_sim);
	//_sensor_simulator.runSeconds(_init_tilt_period);
	_sensor_simulator.runSeconds(10);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);

	velocityAndPositionCloseToZero();

	positionVarianceBigEnoughAfterOrientationInitialization(0.00001f); // Fake position fusion obs var when at rest sq(0.5f)
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, initializeWithTiltNoGyroBiasEstimate)
{
	const float pitch = math::radians(30.0f);
	const float roll = math::radians(-20.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_ekf_wrapper.disableGyroBiasEstimation();
	_sensor_simulator.simulateOrientation(quat_sim);

	_sensor_simulator.runSeconds(_init_tilt_period);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);

	velocityAndPositionCloseToZero();

	positionVarianceBigEnoughAfterOrientationInitialization(0.00001f); // Fake position fusion obs var when at rest sq(0.5f)
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, gyroBias)
{
	// GIVEN: a healthy filter
	_sensor_simulator.runSeconds(20);

	// WHEN: there is a yaw gyro bias after initial convergence of the filter
	_sensor_simulator._imu.setGyroData(Vector3f(0.f, 0.f, 0.1f));

	// THEN: the vertical accel bias should not be affected
	Vector3f accel_bias;

	for (int ii = 0; ii < 100; ii++) {
		_sensor_simulator.runSeconds(0.5);
		accel_bias = _ekf->getAccelBias();

		if (fabsf(accel_bias(2)) > 0.3f) {

			// Print state covariance and correlation matrices for debugging
			const auto P = _ekf->covariances();

			printf("State covariance:\n");

			for (int i = 0; i <= State::size; i++) {
				for (int j = 0; j <= State::size; j++) {
					printf("%.3fe-9  ", ((double)P(i, j)) * 1e9);
				}

				printf("\n");
			}

			printf("State correlation:\n");
			printf("\t0\t1\t2\t3\t4\t5\t6\t7\t8\t9\t10\t11\t12\t13\t14\t15\n");

			for (uint8_t i = 0; i <= State::size; i++) {
				printf("%d|  ", i);

				for (uint8_t j = 0; j <= State::size; j++) {
					float corr = sqrtf(fabsf(P(i, i) * P(j, j)));

					if (corr > 0.0f) {
						corr = fabsf(P(i, j)) / corr;
					}

					printf("%.3f\t", (double)corr);
				}

				printf("\n");
			}

			printf("Accel bias = (%f, %f, %f)\n", (double)accel_bias(0), (double)accel_bias(1), (double)accel_bias(2));

			Vector3f gyro_bias = _ekf->getGyroBias();
			printf("Gyro bias = (%f, %f, %f)\n", (double)gyro_bias(0), (double)gyro_bias(1), (double)gyro_bias(2));

			EXPECT_TRUE(false);
			break;
		}
	}
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

	EXPECT_TRUE(_ekf->control_status_flags().vehicle_at_rest);
	EXPECT_FALSE(_ekf->control_status_flags().in_air);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);
	EXPECT_TRUE(_ekf->control_status_flags().yaw_align);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);
	yawVarianceBigEnoughAfterHeadingReset();

	velocityAndPositionCloseToZero();

	// Fake position fusion obs var when at rest sq(0.01f)
	positionVarianceBigEnoughAfterOrientationInitialization(0.00001f);
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, initializeWithTilt)
{
	const float pitch = math::radians(30.0f);
	const float roll = math::radians(60.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_sensor_simulator.simulateOrientation(quat_sim);
	_sensor_simulator.runSeconds(_init_tilt_period);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);
	EXPECT_TRUE(_ekf->control_status_flags().yaw_align);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);
	yawVarianceBigEnoughAfterHeadingReset();

	velocityAndPositionCloseToZero();

	// Fake position fusion obs var when at rest sq(0.01f)
	positionVarianceBigEnoughAfterOrientationInitialization(0.00001f);
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, initializeWithTiltNotAtRest)
{
	const float pitch = math::radians(30.0f);
	const float roll = math::radians(60.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_ekf->set_in_air_status(true);
	_ekf->set_vehicle_at_rest(false);
	_sensor_simulator.simulateOrientation(quat_sim);
	//_sensor_simulator.runSeconds(_init_tilt_period);
	_sensor_simulator.runSeconds(10);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);

	velocityAndPositionCloseToZero();

	positionVarianceBigEnoughAfterOrientationInitialization(0.01f);
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, initializeWithPitch90)
{
	const float pitch = math::radians(90.0f);
	const float roll = math::radians(0.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_ekf->set_in_air_status(false);
	_ekf->set_vehicle_at_rest(true);
	_sensor_simulator.simulateOrientation(quat_sim);
	//_sensor_simulator.runSeconds(_init_tilt_period);
	_sensor_simulator.runSeconds(10);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);

	// TODO: Quaternion Variance is smaller and vel x is larger
	// in this case than in the other cases

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);

	velocityAndPositionCloseToZero();

	// Fake position fusion obs var when at rest sq(0.01f)
	positionVarianceBigEnoughAfterOrientationInitialization(0.00001f);
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}

TEST_F(EkfInitializationTest, initializeWithRoll90)
{
	const float pitch = math::radians(0.0f);
	const float roll = math::radians(90.0f);
	const Eulerf euler_angles_sim(roll, pitch, 0.0f);
	const Quatf quat_sim(euler_angles_sim);

	_ekf->set_in_air_status(false);
	_ekf->set_vehicle_at_rest(true);
	_sensor_simulator.simulateOrientation(quat_sim);
	//_sensor_simulator.runSeconds(_init_tilt_period);
	_sensor_simulator.runSeconds(10);

	EXPECT_TRUE(_ekf->control_status_flags().tilt_align);

	initializedOrienationIsMatchingGroundTruth(quat_sim);
	quaternionVarianceBigEnoughAfterOrientationInitialization(0.00001f);

	velocityAndPositionCloseToZero();

	// Fake position fusion obs var when at rest sq(0.01f)
	positionVarianceBigEnoughAfterOrientationInitialization(0.00001f);
	velocityVarianceBigEnoughAfterOrientationInitialization(0.0001f);

	_sensor_simulator.runSeconds(1.f);
	learningCorrectAccelBias();
}
