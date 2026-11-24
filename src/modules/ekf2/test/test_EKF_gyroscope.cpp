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
 * Gyro-related tests (bias convergence, ...)
 */

#include <functional>
#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"


class EkfGyroscopeTest : public ::testing::Test
{
public:

	EkfGyroscopeTest(): ::testing::Test(),
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

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}

	void testBias(const Vector3f &bias, float duration, const Vector3f &tolerance);
};

void EkfGyroscopeTest::testBias(const Vector3f &bias, float duration, const Vector3f &tolerance)
{
	_sensor_simulator._imu.setGyroData(bias);
	_sensor_simulator.runSeconds(duration);
	EXPECT_TRUE(_ekf->control_status_flags().vehicle_at_rest);

	const Vector3f estimated_bias = _ekf->getGyroBias();

	for (int i = 0; i < 3; i++) {
		EXPECT_NEAR(estimated_bias(i), bias(i), tolerance(i)) << "index " << i;
	}
}

TEST_F(EkfGyroscopeTest, biasEstimateZero)
{
	testBias(Vector3f(), 10, Vector3f());
}

TEST_F(EkfGyroscopeTest, biasEstimatePositive)
{
	// The estimate should track a slowly changing bias
	const float biases[4] = {0.001f, 0.002f, 0.0035f, 0.005f};
	Vector3f bias;

	for (int i = 0; i < 4; i ++) {
		bias.setAll(biases[i]);
		// The Z gyro bias takes more time to converge as the Z rotation variance is higher
		testBias(bias, 30, Vector3f(0.0008f, 0.0008f, 0.004f));
	}
}

TEST_F(EkfGyroscopeTest, biasEstimateNegative)
{
	const float biases[4] = {-0.001f, -0.002f, -0.0035, -0.005f};
	Vector3f bias;

	for (int i = 0; i < 4; i ++) {
		bias.setAll(biases[i]);
		testBias(bias, 30, Vector3f(0.0008f, 0.0008f, 0.004f));
	}
}

// Stand-in for the rotation check of the land detector: it needs both the bias-corrected
// angular velocity (vehicle_angular_velocity) and the uncorrected one to rotate, sees them
// with some latency, and its output reaches the EKF through the time-aligned system flag
// buffer, as in EKF2.
class GroundedVehicle
{
public:
	GroundedVehicle():
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf)
	{
		_ekf->init(0);
		_sensor_simulator.setMotionClearsAtRest(false);

		parameters *params = _ekf->getParamHandle();
		params->ekf2_mag_type = MagFuseType::INIT;
		params->ekf2_gbias_init = 0.04f;
		params->ekf2_gyr_b_lim = 0.15f;
	}

	void run(float duration, const std::function<Vector3f(float)> &angular_rate, bool handled = false)
	{
		static constexpr uint32_t kImuIntervalUs = 5'000;

		for (float t_end = _time + duration; _time < t_end; _time += kImuIntervalUs * 1e-6f) {
			const Vector3f rate = angular_rate(_time);

			const Vector3f rate_seen = _rate_history[_history_index];
			_rate_history[_history_index] = rate;
			_history_index = (_history_index + 1) % kDetectorLatencySamples;

			const uint64_t now = _sensor_simulator.getTime();

			const bool rotating = (rate_seen - _ekf->getGyroBias()).longerThan(math::radians(3.f))
					      && rate_seen.longerThan(math::radians(1.5f));

			if (handled || rotating) {
				_time_last_move_us = now;
			}

			systemFlagUpdate flags{};
			flags.time_us = now;
			flags.in_air = false;
			flags.at_rest = (now - _time_last_move_us) > 1'000'000;
			_ekf->setSystemFlagData(flags);

			if (flags.at_rest && !PX4_ISFINITE(_time_at_rest)) {
				_time_at_rest = _time;
			}

			_sensor_simulator._imu.setGyroData(rate);
			_sensor_simulator.runMicroseconds(kImuIntervalUs);

			_gyro_bias_z_max = fmaxf(_gyro_bias_z_max, fabsf(gyroBiasZ()));
		}
	}

	float gyroBiasZ() const { return _ekf->getGyroBias()(2); }
	float gyroBiasZMax() const { return _gyro_bias_z_max; }
	bool atRest() const { return _ekf->control_status_flags().vehicle_at_rest; }
	float timeAtRest() const { return _time_at_rest; }

private:
	static constexpr size_t kDetectorLatencySamples = 5; // 25 ms: rate filtering + scheduling

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;

	Vector3f _rate_history[kDetectorLatencySamples] {};
	size_t _history_index{0};
	uint64_t _time_last_move_us{0};
	float _time{0.f};
	float _time_at_rest{NAN};
	float _gyro_bias_z_max{0.f};
};

static void expectYawRotationIsNotLearnedAsGyroBias(const std::function<float(float)> &yaw_rate)
{
	// The rotation is real motion: the true gyro bias stays zero. Sweep the onset over one
	// 0.2 s zero gyro update window, as the outcome depends on where the window ends.
	for (float onset = 0.f; onset < 0.2f; onset += 0.005f) {
		GroundedVehicle vehicle;
		vehicle.run(5.f, [](float) { return Vector3f(); }, true);
		// Rest starts after 1 s and has to resume after the rotation
		vehicle.run(4.f, [&vehicle, &yaw_rate, onset](float t) {
			const float rotation_time = t - (vehicle.timeAtRest() + onset);
			return Vector3f(0.f, 0.f, (rotation_time >= 0.f) ? yaw_rate(rotation_time) : 0.f);
		});
		ASSERT_TRUE(PX4_ISFINITE(vehicle.timeAtRest()));

		EXPECT_LT(vehicle.gyroBiasZMax(), math::radians(0.1f)) << "onset " << onset << " s, bias "
				<< math::degrees(vehicle.gyroBiasZMax()) << " deg/s";
	}
}

TEST(EkfGyroscopeAtRest, yawTwistIsNotLearnedAsGyroBias)
{
	expectYawRotationIsNotLearnedAsGyroBias([](float t) { return (t < 0.04f) ? math::radians(100.f) : 0.f; });
}

TEST(EkfGyroscopeAtRest, slowYawOnsetIsNotLearnedAsGyroBias)
{
	// The rotation stays below the at-rest threshold for its first 0.1 s
	expectYawRotationIsNotLearnedAsGyroBias([](float t) { return (t < 1.f) ? math::radians(30.f) * t : 0.f; });
}

TEST(EkfGyroscopeAtRest, gyroBiasErrorAboveRotationThresholdRecovers)
{
	// A rotation slower than the at-rest threshold is indistinguishable from a gyro bias:
	// a turntable spinning up in steps gets learned as bias, then it stops and the estimate
	// is off by more than the threshold
	GroundedVehicle vehicle;
	vehicle.run(2.f, [](float) { return Vector3f(); });
	vehicle.run(20.f, [](float) { return Vector3f(0.f, 0.f, math::radians(2.5f)); });
	vehicle.run(40.f, [](float) { return Vector3f(0.f, 0.f, math::radians(5.f)); });
	ASSERT_GT(vehicle.gyroBiasZ(), math::radians(3.5f));

	// Only the bias-corrected rate still rotates
	vehicle.run(2.f, [](float) { return Vector3f(); });
	EXPECT_TRUE(vehicle.atRest());

	vehicle.run(40.f, [](float) { return Vector3f(); });
	EXPECT_LT(fabsf(vehicle.gyroBiasZ()), math::radians(3.f)) << math::degrees(vehicle.gyroBiasZ()) << " deg/s";
}
