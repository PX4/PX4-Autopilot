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

#include "FlightTaskAutoFollowTarget.hpp"

#include <drivers/drv_hrt.h>
#include <hrt_work.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform_common/time.h>

// to run: make tests TESTFILTER=FlightTaskAutoFollowTarget

using namespace time_literals;
using matrix::Vector3f;

class FlightTaskAutoFollowTargetTestPeer : public FlightTaskAutoFollowTarget
{
public:
	using FlightTaskAutoFollowTarget::updateTargetPositionVelocityFilter;
	void setDeltatime(float deltatime) { _deltatime = deltatime; }
	const Vector3f &targetPosition() const { return _target_position_velocity_filter.getState(); }
	const Vector3f &targetVelocity() const { return _target_position_velocity_filter.getRate(); }
};

// stop the work queue manager at the end so the process exits
class FlightTaskAutoFollowTargetTestEnvironment : public ::testing::Environment
{
public:
	void TearDown() override { px4::WorkQueueManagerStop(); }
};
static const auto *global_env = ::testing::AddGlobalTestEnvironment(new FlightTaskAutoFollowTargetTestEnvironment());

// one item keeps the queue alive between the target estimators of the single tests
class WorkQueueKeeper : public px4::ScheduledWorkItem
{
public:
	WorkQueueKeeper() : ScheduledWorkItem("wq_keeper", px4::wq_configurations::nav_and_controllers) {}
private:
	void Run() override {}
};

class FlightTaskAutoFollowTargetTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		// the task owns a target estimator, which attaches to a work queue at construction
		static bool wq_manager_started = false;

		if (!wq_manager_started) {
			hrt_init();
			hrt_work_queue_init();
			ASSERT_EQ(px4::WorkQueueManagerStart(), 0);
			wq_manager_started = true;
		}

		const hrt_abstime wq_wait_start = hrt_absolute_time();

		while (px4::WorkQueueFindOrCreate(px4::wq_configurations::nav_and_controllers) == nullptr) {
			ASSERT_LT(hrt_elapsed_time(&wq_wait_start), 10_s) << "work queue manager did not start";
			px4_usleep(100_ms);
		}

		static WorkQueueKeeper *wq_keeper = new WorkQueueKeeper();
		ASSERT_NE(wq_keeper, nullptr);
	}

	static follow_target_estimator_s estimate(hrt_abstime timestamp, const Vector3f &position, const Vector3f &velocity,
			uint8_t pos_reset_counter, const Vector3f &delta_pos)
	{
		follow_target_estimator_s follow_target_estimator{};
		follow_target_estimator.timestamp = timestamp;
		follow_target_estimator.valid = true;
		position.copyTo(follow_target_estimator.pos_est);
		velocity.copyTo(follow_target_estimator.vel_est);
		follow_target_estimator.pos_reset_counter = pos_reset_counter;
		delta_pos.copyTo(follow_target_estimator.delta_pos);
		return follow_target_estimator;
	}

	static constexpr float kDeltatime = 0.02f;
	static constexpr hrt_abstime kDeltatimeUs = 20_ms;
};

// The target drives north at 2 m/s while the estimate is carried over to a new local frame.
// The filtered target must move with it instead of sliding over to the new frame
TEST_F(FlightTaskAutoFollowTargetTest, FilteredTargetMovesWithTheEstimate)
{
	FlightTaskAutoFollowTargetTestPeer task;
	// fails without a home position, but sets up the target filter first
	task.activate(FlightTask::empty_trajectory_setpoint);
	task.setDeltatime(kDeltatime);

	const Vector3f start{10.f, 0.f, -5.f};
	const Vector3f velocity{2.f, 0.f, 0.f};
	hrt_abstime timestamp = 10_s;
	float time = 0.f;

	// GIVEN: a filter that settled on the target, the counter about to wrap around
	for (int i = 0; i < 500; i++) {
		timestamp += kDeltatimeUs;
		time += kDeltatime;
		task.updateTargetPositionVelocityFilter(estimate(timestamp, start + velocity * time, velocity, 255, Vector3f{}));
	}

	ASSERT_LT((task.targetPosition() - (start + velocity * time)).norm(), 0.05f);

	// WHEN: the estimate moves 100 m south, 30 m west and 3 m down with the frame
	const Vector3f delta_pos{-100.f, -30.f, 3.f};
	float max_position_error = 0.f;
	float max_velocity_error = 0.f;

	for (int i = 0; i < 150; i++) {
		timestamp += kDeltatimeUs;
		time += kDeltatime;
		task.updateTargetPositionVelocityFilter(estimate(timestamp, start + velocity * time + delta_pos, velocity, 0,
							delta_pos));
		max_position_error = math::max(max_position_error,
					       (task.targetPosition() - (start + velocity * time + delta_pos)).norm());
		max_velocity_error = math::max(max_velocity_error, (task.targetVelocity() - velocity).norm());
	}

	// THEN: the filtered target stays on the estimate and keeps its velocity
	EXPECT_LT(max_position_error, 0.05f);
	EXPECT_LT(max_velocity_error, 0.05f);
}

// The first estimate after activation already reports a reset while the filter is still empty
TEST_F(FlightTaskAutoFollowTargetTest, ResetReportedBeforeTheFilterStarted)
{
	FlightTaskAutoFollowTargetTestPeer task;
	task.activate(FlightTask::empty_trajectory_setpoint);
	task.setDeltatime(kDeltatime);

	const Vector3f target{10.f, 0.f, -5.f};
	const Vector3f delta_pos{-100.f, 0.f, 0.f};
	hrt_abstime timestamp = 10_s;

	// WHEN: the filter starts from an estimate with a reset count
	task.updateTargetPositionVelocityFilter(estimate(timestamp, target, Vector3f{}, 3, delta_pos));

	// THEN: it starts on the estimate
	EXPECT_LT((task.targetPosition() - target).norm(), 1e-3f);

	// AND: the same count doesn't move it again
	for (int i = 0; i < 50; i++) {
		timestamp += kDeltatimeUs;
		task.updateTargetPositionVelocityFilter(estimate(timestamp, target, Vector3f{}, 3, delta_pos));
	}

	EXPECT_LT((task.targetPosition() - target).norm(), 1e-3f);
}
