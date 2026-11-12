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
 * Test code for the 2D velocity smoothing library
 * Run this test only using make tests TESTFILTER=VelocitySmoothing2D
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>

#include "VelocitySmoothing.hpp"
#include "VelocitySmoothing2D.hpp"

using namespace matrix;

static constexpr float MAX_JERK = 4.f;
static constexpr float MAX_ACCEL = 3.f;
static constexpr float MAX_VEL = 20.f;

class VelocitySmoothing2DTest : public ::testing::Test
{
public:
	VelocitySmoothing2DTest()
	{
		_trajectory.setMaxJerk(MAX_JERK);
		_trajectory.setMaxAccel(MAX_ACCEL);
		_trajectory.setMaxVel(MAX_VEL);
	}

	// Integrate the planned profile without replanning and check the limits on the way
	void flyPlan(float dt = 0.002f)
	{
		const int n_steps = static_cast<int>(ceilf(_trajectory.getTotalTime() / dt));

		for (int i = 0; i < n_steps; i++) {
			_trajectory.updateTraj(dt);
			EXPECT_LE(_trajectory.getCurrentJerk().norm(), MAX_JERK + 1e-3f);
			EXPECT_LE(_trajectory.getCurrentAcceleration().norm(), MAX_ACCEL + 1e-3f);
		}
	}

	void expectArrived(const Vector2f &velocity_setpoint)
	{
		EXPECT_NEAR(_trajectory.getCurrentVelocity()(0), velocity_setpoint(0), 0.01f);
		EXPECT_NEAR(_trajectory.getCurrentVelocity()(1), velocity_setpoint(1), 0.01f);
		EXPECT_NEAR(_trajectory.getCurrentAcceleration().norm(), 0.f, 0.01f);
	}

	VelocitySmoothing2D _trajectory;
};

TEST_F(VelocitySmoothing2DTest, allZeroCase)
{
	VelocitySmoothing2D trajectory;
	trajectory.updateDurations(Vector2f{});
	trajectory.updateTraj(0.f);

	EXPECT_FLOAT_EQ(trajectory.getTotalTime(), 0.f);
	EXPECT_FLOAT_EQ(trajectory.getCurrentJerk().norm(), 0.f);
	EXPECT_FLOAT_EQ(trajectory.getCurrentAcceleration().norm(), 0.f);
	EXPECT_FLOAT_EQ(trajectory.getCurrentVelocity().norm(), 0.f);
	EXPECT_FLOAT_EQ(trajectory.getCurrentPosition().norm(), 0.f);
}

TEST_F(VelocitySmoothing2DTest, matchesTheOneDimensionalPlanner)
{
	// GIVEN: the same one-dimensional problems for the 1D and the 2D planner
	const float a0[] = {0.f, 1.5f, -2.f, 0.5f, 2.9f};
	const float v0[] = {0.f, 3.f, -1.f, 10.f, 0.f};
	const float v_sp[] = {10.f, -4.f, -1.f, 10.3f, -0.2f};

	for (int i = 0; i < 5; i++) {
		VelocitySmoothing trajectory_1d(a0[i], v0[i], 0.f);
		trajectory_1d.setMaxJerk(MAX_JERK);
		trajectory_1d.setMaxAccel(MAX_ACCEL);
		trajectory_1d.setMaxVel(MAX_VEL);
		trajectory_1d.updateDurations(v_sp[i]);

		_trajectory.reset(Vector2f{a0[i], 0.f}, Vector2f{v0[i], 0.f}, Vector2f{});
		_trajectory.updateDurations(Vector2f{v_sp[i], 0.f});

		// THEN: the phase durations are the same
		EXPECT_NEAR(_trajectory.getT1(), trajectory_1d.getT1(), 1e-3f) << "case " << i;
		EXPECT_NEAR(_trajectory.getT2(), trajectory_1d.getT2(), 1e-3f) << "case " << i;
		EXPECT_NEAR(_trajectory.getT3(), trajectory_1d.getT3(), 1e-3f) << "case " << i;
	}
}

TEST_F(VelocitySmoothing2DTest, isIsotropic)
{
	// GIVEN: a problem and the same problem rotated by an arbitrary angle
	const Vector2f a0{1.f, -0.5f};
	const Vector2f v0{3.f, 2.f};
	const Vector2f v_sp{-6.f, 7.f};
	const float angle = 1.1f;
	const Dcm2f rotation{angle};

	_trajectory.reset(a0, v0, Vector2f{});
	_trajectory.updateDurations(v_sp);
	const float T1 = _trajectory.getT1();
	const float T2 = _trajectory.getT2();
	const float T3 = _trajectory.getT3();
	flyPlan();
	const Vector2f velocity = _trajectory.getCurrentVelocity();

	_trajectory.reset(rotation * a0, rotation * v0, Vector2f{});
	_trajectory.updateDurations(rotation * v_sp);

	// THEN: the durations are the same and the result is the rotated one
	EXPECT_NEAR(_trajectory.getT1(), T1, 1e-4f);
	EXPECT_NEAR(_trajectory.getT2(), T2, 1e-4f);
	EXPECT_NEAR(_trajectory.getT3(), T3, 1e-4f);
	flyPlan();
	EXPECT_NEAR((_trajectory.getCurrentVelocity() - rotation * velocity).norm(), 0.f, 1e-3f);
}

TEST_F(VelocitySmoothing2DTest, reachesTheSetpointWithinTheLimits)
{
	// GIVEN: initial accelerations in all directions and velocity changes of all sizes
	for (int i = 0; i < 200; i++) {
		const float angle_a = 0.1f * i;
		const float angle_v = 0.37f * i;
		const Vector2f a0 = (0.9f * MAX_ACCEL * (i % 7) / 6.f) * Vector2f{cosf(angle_a), sinf(angle_a)};
		const Vector2f v_sp = (0.01f + 0.1f * (i % 23)) * Vector2f{cosf(angle_v), sinf(angle_v)};

		_trajectory.reset(a0, Vector2f{}, Vector2f{});

		// WHEN: the profile is planned and flown
		_trajectory.updateDurations(v_sp);
		flyPlan();

		// THEN: the velocity setpoint is reached with zero acceleration
		expectArrived(v_sp);
	}
}

TEST_F(VelocitySmoothing2DTest, excessAccelerationGivenTotalTime)
{
	// GIVEN: an initial acceleration larger than needed to make the velocity change in the given time
	const Vector2f a0{0.f, -1.5f};
	const Vector2f v_sp{0.f, -3.f};
	_trajectory.reset(a0, Vector2f{}, Vector2f{});
	_trajectory.updateDurations(v_sp);
	const float total_time = 3.f * _trajectory.getTotalTime();

	// WHEN: the total time is given
	_trajectory.updateDurationsGivenTotalTime(total_time);

	// THEN: the acceleration is reduced first and the setpoint is reached exactly at the given time
	EXPECT_NEAR(_trajectory.getTotalTime(), total_time, 1e-3f);
	_trajectory.updateTraj(0.01f);
	EXPECT_GT(_trajectory.getCurrentJerk()(1), 0.f);
	const int n_steps = static_cast<int>(roundf(total_time / 0.01f)) - 1;

	for (int i = 0; i < n_steps; i++) {
		_trajectory.updateTraj(0.01f);
		EXPECT_GE(_trajectory.getCurrentVelocity()(1), v_sp(1) - 0.01f);
	}

	expectArrived(v_sp);
}

TEST_F(VelocitySmoothing2DTest, givenTotalTime)
{
	// GIVEN: a turn planned in minimum time
	const Vector2f a0{2.f, 1.f};
	const Vector2f v0{10.f, 0.f};
	const Vector2f v_sp{0.f, 10.f};
	_trajectory.reset(a0, v0, Vector2f{});
	_trajectory.updateDurations(v_sp);

	// WHEN: it has to take longer, e.g. to synchronize with a slow vertical trajectory
	const float total_time = 1.7f * _trajectory.getTotalTime();
	_trajectory.updateDurationsGivenTotalTime(total_time);

	// THEN: the setpoint is reached at the given time
	EXPECT_NEAR(_trajectory.getTotalTime(), total_time, 1e-3f);
	flyPlan();
	expectArrived(v_sp);
}
