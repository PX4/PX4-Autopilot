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

#include <gtest/gtest.h>
#include <FieldSensorBiasConvergence.hpp>
#include <FieldSensorBiasEstimator.hpp>

using namespace matrix;

class FieldSensorBiasConvergenceTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		_estimator.setLearningGain(kGain);
		_estimator.setField(_earth + _bias);
	}

	// rotate the vehicle at a constant rate, the earth field rotates the other way in body frame
	void rotate(const Vector3f &gyro, float duration, float dt = 0.01f)
	{
		for (float t = 0.f; t < duration; t += dt) {
			_earth = Dcmf(AxisAnglef(-gyro * dt)) * _earth;
			const Vector3f field = _earth + _bias;
			_estimator.updateEstimate(gyro, field, dt);
			_convergence.update(gyro, field, _estimator.getBias(), kGain, dt);

			if (_convergence.valid() && !_first_valid_seen) {
				_first_valid_seen = true;
				_first_valid_bias = _convergence.bias();
			}
		}
	}

	static constexpr float kGain{18.f};

	FieldSensorBiasEstimator _estimator;
	FieldSensorBiasConvergence _convergence;
	Vector3f _earth{0.2f, 0.f, 0.4f};
	Vector3f _bias{0.1f, -0.1f, 0.05f};
	bool _first_valid_seen{false};
	Vector3f _first_valid_bias{};
};

TEST_F(FieldSensorBiasConvergenceTest, NoRotationIsNeverValid)
{
	rotate(Vector3f{}, 60.f);

	EXPECT_FALSE(_convergence.valid());
	EXPECT_FALSE(_convergence.stable());
	EXPECT_FALSE(_convergence.observed(0));
}

TEST_F(FieldSensorBiasConvergenceTest, YawOnlyObservesHorizontalAxes)
{
	rotate(Vector3f{0.f, 0.f, 1.f}, 30.f);

	EXPECT_TRUE(_convergence.observed(0));
	EXPECT_TRUE(_convergence.observed(1));
	EXPECT_FALSE(_convergence.observed(2));
	EXPECT_TRUE(_convergence.stable());

	ASSERT_TRUE(_first_valid_seen);
	const Vector2f error{Vector2f(_first_valid_bias.xy()) - Vector2f(_bias.xy())};
	EXPECT_LT(error.norm(), 0.06f * Vector2f(_bias.xy()).norm());
}

TEST_F(FieldSensorBiasConvergenceTest, ShortWiggleIsNotValid)
{
	for (int i = 0; i < 4; i++) {
		rotate(Vector3f{0.f, 0.f, 1.5f}, 0.25f);
		rotate(Vector3f{0.f, 0.f, -1.5f}, 0.25f);
	}

	rotate(Vector3f{}, 20.f);

	EXPECT_FALSE(_convergence.valid());
}

TEST_F(FieldSensorBiasConvergenceTest, TumbleObservesAllAxes)
{
	rotate(Vector3f{1.f, 0.f, 0.f}, 15.f);
	rotate(Vector3f{0.f, 1.f, 0.f}, 15.f);
	rotate(Vector3f{0.f, 0.f, 1.f}, 15.f);

	EXPECT_TRUE(_convergence.observed(0));
	EXPECT_TRUE(_convergence.observed(1));
	EXPECT_TRUE(_convergence.observed(2));
	EXPECT_TRUE(_convergence.stable());
	EXPECT_LT((_estimator.getBias() - _bias).norm(), 0.06f * _bias.norm());
}

TEST_F(FieldSensorBiasConvergenceTest, FieldDisturbanceInvalidates)
{
	rotate(Vector3f{0.f, 0.f, 1.f}, 30.f);
	ASSERT_TRUE(_convergence.valid());

	_earth += Vector3f{0.1f, 0.f, 0.f};
	bool invalidated = false;

	for (int i = 0; i < 40; i++) {
		rotate(Vector3f{0.f, 0.f, 1.f}, 0.1f);
		invalidated |= !_convergence.valid();
	}

	EXPECT_TRUE(invalidated);
}

TEST_F(FieldSensorBiasConvergenceTest, LowerSampleRate)
{
	rotate(Vector3f{0.f, 0.f, 1.f}, 30.f, 0.02f);

	EXPECT_TRUE(_convergence.stable());

	ASSERT_TRUE(_first_valid_seen);
	const Vector2f error{Vector2f(_first_valid_bias.xy()) - Vector2f(_bias.xy())};
	EXPECT_LT(error.norm(), 0.06f * Vector2f(_bias.xy()).norm());
}

TEST_F(FieldSensorBiasConvergenceTest, SmallBiasIsNotValid)
{
	_bias = Vector3f{0.02f, -0.02f, 0.f};
	_estimator.setField(_earth + _bias);

	rotate(Vector3f{0.f, 0.f, 1.f}, 30.f);

	EXPECT_TRUE(_convergence.observed(0));
	EXPECT_FALSE(_convergence.valid());
}

TEST_F(FieldSensorBiasConvergenceTest, BiasOnlyFromPassedWindows)
{
	// with 1/64 s steps a 2 s window is exactly 128 samples, 30 s are 15 windows
	const float dt = 1.f / 64.f;
	const Vector3f yaw{0.f, 0.f, 1.f};

	rotate(yaw, 30.f, dt);
	ASSERT_TRUE(_convergence.stable());

	const Vector3f validated = _convergence.bias();
	EXPECT_LT((Vector2f(validated.xy()) - Vector2f(_bias.xy())).norm(), 0.06f * Vector2f(_bias.xy()).norm());
	EXPECT_FLOAT_EQ(validated(2), 0.f) << "z not observed";

	// e.g. put down next to steel at the start of a window
	_bias += Vector3f{0.1f, 0.f, 0.f};
	rotate(yaw, 1.5f, dt);

	EXPECT_TRUE(_convergence.stable()) << "window not ended yet";
	EXPECT_TRUE((_estimator.getBias() - validated).longerThan(0.02f));
	EXPECT_EQ(_convergence.bias(), validated);

	rotate(yaw, 0.5f, dt);

	EXPECT_FALSE(_convergence.valid()) << "window failed";
	EXPECT_EQ(_convergence.bias(), validated);

	rotate(yaw, 30.f, dt);

	EXPECT_TRUE(_convergence.stable());
	EXPECT_LT((Vector2f(_convergence.bias().xy()) - Vector2f(_bias.xy())).norm(), 0.06f * Vector2f(_bias.xy()).norm());
}

TEST_F(FieldSensorBiasConvergenceTest, StillWindowsDoNotCountTowardStable)
{
	const float dt = 1.f / 64.f;
	const Vector3f yaw{0.f, 0.f, 1.f};

	for (int i = 0; i < 30 && !_convergence.valid(); i++) {
		rotate(yaw, 2.f, dt);
	}

	ASSERT_TRUE(_convergence.valid());
	ASSERT_FALSE(_convergence.stable());

	// e.g. put down right after the first passed window
	rotate(Vector3f{}, 20.f, dt);

	EXPECT_TRUE(_convergence.valid());
	EXPECT_FALSE(_convergence.stable());

	rotate(yaw, 10.f, dt);

	EXPECT_TRUE(_convergence.stable());
}
