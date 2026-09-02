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
 * @file MulticopterPositionControlTest.cpp
 *
 * When the estimator selector hands over to another instance it publishes the
 * shift it applied to the local position estimate. The setpoint has to move by
 * the same amount, otherwise the controller reads the shift as tracking error
 * and flies the vehicle to correct for something that never moved.
 */

#include <gtest/gtest.h>

#include "MulticopterPositionControl.hpp"

#include <hrt_work.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform_common/time.h>

#include <memory>

#include <matrix/math.hpp>

using namespace time_literals;

// PX4 destroys a work queue when its last WorkItem detaches, so one keeper item pins
// the queue for the whole process. It has to go before the manager stops, otherwise it
// outlives its queue holding a dangling pointer.
class WorkQueueKeeper : public px4::ScheduledWorkItem
{
public:
	WorkQueueKeeper() : ScheduledWorkItem("wq_keeper", px4::wq_configurations::nav_and_controllers) {}

private:
	void Run() override {}
};

static std::unique_ptr<WorkQueueKeeper> work_queue_keeper;

class MulticopterPositionControlTestEnvironment : public ::testing::Environment
{
public:
	void SetUp() override { setvbuf(stdout, nullptr, _IONBF, 0); }
	void TearDown() override
	{
		work_queue_keeper.reset();
		px4::WorkQueueManagerStop();
	}
};

static const auto *global_env = ::testing::AddGlobalTestEnvironment(new MulticopterPositionControlTestEnvironment());

class MulticopterPositionControlTestPeer
{
public:
	static void adjustSetpointForEKFResets(MulticopterPositionControl &controller,
					       const vehicle_local_position_s &vehicle_local_position,
					       trajectory_setpoint_s &setpoint)
	{
		// the cases that do not care about the fallback pass a throwaway one
		trajectory_setpoint_s unused_fallback{};
		controller.adjustSetpointForEKFResets(vehicle_local_position, setpoint, unused_fallback);
	}

	static void adjustSetpointForEKFResets(MulticopterPositionControl &controller,
					       const vehicle_local_position_s &vehicle_local_position,
					       trajectory_setpoint_s &setpoint,
					       trajectory_setpoint_s &fallback_setpoint)
	{
		controller.adjustSetpointForEKFResets(vehicle_local_position, setpoint, fallback_setpoint);
	}

	static void setVelocityFilterState(MulticopterPositionControl &controller, const matrix::Vector2f &xy, float z)
	{
		controller._vel_xy_lp_filter.reset(xy);
		controller._vel_z_lp_filter.reset(z);
	}

	static matrix::Vector2f velocityFilterStateXy(const MulticopterPositionControl &controller)
	{
		return controller._vel_xy_lp_filter.getState();
	}

	static float velocityFilterStateZ(const MulticopterPositionControl &controller)
	{
		return controller._vel_z_lp_filter.getState();
	}
};

class MulticopterPositionControlTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		// the controller attaches to the work queue at construction
		static bool wq_manager_started = false;

		if (!wq_manager_started) {
			hrt_work_queue_init();
			ASSERT_EQ(px4::WorkQueueManagerStart(), 0);
			wq_manager_started = true;
		}

		if (!work_queue_keeper) {
			const hrt_abstime wait_start = hrt_absolute_time();

			while (px4::WorkQueueFindOrCreate(px4::wq_configurations::nav_and_controllers) == nullptr) {
				ASSERT_LT(hrt_elapsed_time(&wait_start), 5_s);
				px4_usleep(10_ms);
			}

			work_queue_keeper = std::make_unique<WorkQueueKeeper>();
		}

		_controller = new MulticopterPositionControl();
		ASSERT_NE(_controller, nullptr);
	}

	void TearDown() override
	{
		delete _controller;
		_controller = nullptr;
	}

protected:
	// A local position sample that is newer than the setpoint, which is what the
	// adjustment requires before it will touch anything.
	vehicle_local_position_s makeLocalPosition() const
	{
		vehicle_local_position_s local_position{};
		local_position.timestamp = hrt_absolute_time();
		return local_position;
	}

	trajectory_setpoint_s makeSetpoint() const
	{
		trajectory_setpoint_s setpoint{};
		setpoint.timestamp = hrt_absolute_time() - 100_ms;
		setpoint.position[0] = 10.f;
		setpoint.position[1] = 20.f;
		setpoint.position[2] = -30.f;
		setpoint.velocity[0] = 1.f;
		setpoint.velocity[1] = 2.f;
		setpoint.velocity[2] = 3.f;
		setpoint.yaw = 0.5f;
		return setpoint;
	}

	MulticopterPositionControl *_controller{nullptr};
};

// WHY: with no reset the setpoint is the commanded one and must be passed through.
TEST_F(MulticopterPositionControlTest, NoResetLeavesTheSetpointAlone)
{
	vehicle_local_position_s local_position = makeLocalPosition();
	trajectory_setpoint_s setpoint = makeSetpoint();
	const trajectory_setpoint_s original = setpoint;

	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, setpoint);

	EXPECT_FLOAT_EQ(setpoint.position[0], original.position[0]);
	EXPECT_FLOAT_EQ(setpoint.position[1], original.position[1]);
	EXPECT_FLOAT_EQ(setpoint.position[2], original.position[2]);
	EXPECT_FLOAT_EQ(setpoint.yaw, original.yaw);
}

// WHY: this is the selector handover case. The estimate moved by delta_z, so the
// setpoint has to move with it or the controller sees a step in its own error.
TEST_F(MulticopterPositionControlTest, VerticalResetShiftsTheSetpointByTheSameDelta)
{
	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.z_reset_counter = 1;
	local_position.delta_z = -1.5f;

	trajectory_setpoint_s setpoint = makeSetpoint();
	const float expected = setpoint.position[2] + local_position.delta_z;

	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, setpoint);

	EXPECT_FLOAT_EQ(setpoint.position[2], expected);
	// only the axis that was reset moves
	EXPECT_FLOAT_EQ(setpoint.position[0], 10.f);
	EXPECT_FLOAT_EQ(setpoint.position[1], 20.f);
}

// WHY: a horizontal handover shifts north and east together.
TEST_F(MulticopterPositionControlTest, HorizontalResetShiftsTheSetpointByTheSameDelta)
{
	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.xy_reset_counter = 1;
	local_position.delta_xy[0] = 2.5f;
	local_position.delta_xy[1] = -3.5f;

	trajectory_setpoint_s setpoint = makeSetpoint();

	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, setpoint);

	EXPECT_FLOAT_EQ(setpoint.position[0], 12.5f);
	EXPECT_FLOAT_EQ(setpoint.position[1], 16.5f);
	EXPECT_FLOAT_EQ(setpoint.position[2], -30.f);
}

// WHY: velocity setpoints ride through a velocity reset the same way.
TEST_F(MulticopterPositionControlTest, VelocityResetShiftsTheVelocitySetpoint)
{
	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.vxy_reset_counter = 1;
	local_position.delta_vxy[0] = 0.5f;
	local_position.delta_vxy[1] = -0.25f;
	local_position.vz_reset_counter = 1;
	local_position.delta_vz = 0.75f;

	trajectory_setpoint_s setpoint = makeSetpoint();

	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, setpoint);

	EXPECT_FLOAT_EQ(setpoint.velocity[0], 1.5f);
	EXPECT_FLOAT_EQ(setpoint.velocity[1], 1.75f);
	EXPECT_FLOAT_EQ(setpoint.velocity[2], 3.75f);
}

// WHY: the counters are latched after the first pass, so a second pass on the same
// sample must not apply the shift again. Applying it twice would walk the setpoint
// away by one delta on every cycle for as long as the counter stayed put.
TEST_F(MulticopterPositionControlTest, SameResetIsNotAppliedTwice)
{
	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.z_reset_counter = 1;
	local_position.delta_z = -1.5f;

	trajectory_setpoint_s setpoint = makeSetpoint();
	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, setpoint);
	const float after_first = setpoint.position[2];

	local_position.timestamp = hrt_absolute_time();
	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, setpoint);

	EXPECT_FLOAT_EQ(setpoint.position[2], after_first);
}

// WHY: a setpoint that is newer than the estimate sample was produced after the
// reset was already accounted for, so shifting it again would double count.
TEST_F(MulticopterPositionControlTest, SetpointNewerThanTheEstimateIsNotShifted)
{
	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.z_reset_counter = 1;
	local_position.delta_z = -1.5f;

	trajectory_setpoint_s setpoint = makeSetpoint();
	setpoint.timestamp = local_position.timestamp + 100_ms;

	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, setpoint);

	EXPECT_FLOAT_EQ(setpoint.position[2], -30.f);
}

// WHY: an empty setpoint has nothing to shift, and the counters still have to be
// latched so the next real setpoint is not shifted by a reset it never saw.
TEST_F(MulticopterPositionControlTest, UnsetSetpointIsNotShiftedAndStillLatchesTheCounter)
{
	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.z_reset_counter = 1;
	local_position.delta_z = -1.5f;

	trajectory_setpoint_s empty{};
	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, empty);
	EXPECT_FLOAT_EQ(empty.position[2], 0.f);

	trajectory_setpoint_s setpoint = makeSetpoint();
	local_position.timestamp = hrt_absolute_time();
	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, setpoint);

	EXPECT_FLOAT_EQ(setpoint.position[2], -30.f);
}

// WHY: a heading reset wraps, so the shifted yaw has to come back inside +-pi rather
// than drifting outside it.
TEST_F(MulticopterPositionControlTest, HeadingResetShiftsYawAndWraps)
{
	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.heading_reset_counter = 1;
	local_position.delta_heading = 3.f;

	trajectory_setpoint_s setpoint = makeSetpoint();
	setpoint.yaw = 3.f;

	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, setpoint);

	// 3 + 3 is outside +-pi, so it wraps to 6 - 2*pi rather than staying at 6
	EXPECT_NEAR(setpoint.yaw, -0.283185307f, 1e-6f);
}

// WHY: the guard is a strict less than, so a setpoint stamped at the same time as the
// estimate sample is not older than it and must not be shifted.
TEST_F(MulticopterPositionControlTest, SetpointStampedWithTheEstimateIsNotShifted)
{
	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.z_reset_counter = 1;
	local_position.delta_z = -1.5f;

	trajectory_setpoint_s setpoint = makeSetpoint();
	setpoint.timestamp = local_position.timestamp;

	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, setpoint);

	EXPECT_FLOAT_EQ(setpoint.position[2], -30.f);
}

// WHY: skipping the shift must still latch the counter, otherwise the next setpoint
// would be shifted by a reset that happened before it existed.
TEST_F(MulticopterPositionControlTest, SkippedShiftStillLatchesTheCounter)
{
	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.z_reset_counter = 1;
	local_position.delta_z = -1.5f;

	trajectory_setpoint_s newer = makeSetpoint();
	newer.timestamp = local_position.timestamp + 100_ms;
	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, newer);
	EXPECT_FLOAT_EQ(newer.position[2], -30.f);

	trajectory_setpoint_s older = makeSetpoint();
	local_position.timestamp = hrt_absolute_time();
	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, older);

	EXPECT_FLOAT_EQ(older.position[2], -30.f);
}

// WHY: an instance handover moves every state at once, so all five counters can change
// in a single sample. Each axis has to take its own delta and nothing may be double
// counted across axes.
TEST_F(MulticopterPositionControlTest, AllCountersChangingInOneSampleShiftEveryAxis)
{
	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.vxy_reset_counter = 1;
	local_position.delta_vxy[0] = 0.25f;
	local_position.delta_vxy[1] = -0.5f;
	local_position.vz_reset_counter = 1;
	local_position.delta_vz = 0.75f;
	local_position.xy_reset_counter = 1;
	local_position.delta_xy[0] = 2.5f;
	local_position.delta_xy[1] = -3.5f;
	local_position.z_reset_counter = 1;
	local_position.delta_z = -1.5f;
	local_position.heading_reset_counter = 1;
	local_position.delta_heading = 0.25f;

	trajectory_setpoint_s setpoint = makeSetpoint();

	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, setpoint);

	EXPECT_FLOAT_EQ(setpoint.velocity[0], 1.25f);
	EXPECT_FLOAT_EQ(setpoint.velocity[1], 1.5f);
	EXPECT_FLOAT_EQ(setpoint.velocity[2], 3.75f);
	EXPECT_FLOAT_EQ(setpoint.position[0], 12.5f);
	EXPECT_FLOAT_EQ(setpoint.position[1], 16.5f);
	EXPECT_FLOAT_EQ(setpoint.position[2], -31.5f);
	EXPECT_FLOAT_EQ(setpoint.yaw, 0.75f);
}

// WHY: the controller filters the measured velocity. If the estimate steps and the filter
// state stays behind, the filtered velocity carries a step the vehicle never flew, so the
// filter has to be moved by the same delta as the state it tracks.
TEST_F(MulticopterPositionControlTest, VelocityResetCarriesTheVelocityFilterState)
{
	MulticopterPositionControlTestPeer::setVelocityFilterState(*_controller, matrix::Vector2f(4.f, -2.f), 1.f);

	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.vxy_reset_counter = 1;
	local_position.delta_vxy[0] = 0.5f;
	local_position.delta_vxy[1] = -1.5f;
	local_position.vz_reset_counter = 1;
	local_position.delta_vz = 0.25f;

	trajectory_setpoint_s setpoint = makeSetpoint();

	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, setpoint);

	const matrix::Vector2f filter_xy = MulticopterPositionControlTestPeer::velocityFilterStateXy(*_controller);
	EXPECT_FLOAT_EQ(filter_xy(0), 4.5f);
	EXPECT_FLOAT_EQ(filter_xy(1), -3.5f);
	EXPECT_FLOAT_EQ(MulticopterPositionControlTestPeer::velocityFilterStateZ(*_controller), 1.25f);
}

// WHY: the filter reset sits outside the timestamp guard on purpose. The guard is about
// whether a setpoint predates the reset, which says nothing about the measured velocity,
// so a setpoint newer than the estimate must still leave the filters corrected.
TEST_F(MulticopterPositionControlTest, FilterStateIsCarriedEvenWhenTheSetpointShiftIsSkipped)
{
	MulticopterPositionControlTestPeer::setVelocityFilterState(*_controller, matrix::Vector2f(4.f, -2.f), 1.f);

	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.vxy_reset_counter = 1;
	local_position.delta_vxy[0] = 0.5f;
	local_position.delta_vxy[1] = -1.5f;
	local_position.vz_reset_counter = 1;
	local_position.delta_vz = 0.25f;

	// newer than the estimate, so the guard rejects the setpoint shift
	trajectory_setpoint_s setpoint = makeSetpoint();
	setpoint.timestamp = local_position.timestamp + 100_ms;

	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, setpoint);

	// the setpoint is untouched
	EXPECT_FLOAT_EQ(setpoint.velocity[0], 1.f);
	EXPECT_FLOAT_EQ(setpoint.velocity[2], 3.f);

	// but the filters still moved with the estimate
	const matrix::Vector2f filter_xy = MulticopterPositionControlTestPeer::velocityFilterStateXy(*_controller);
	EXPECT_FLOAT_EQ(filter_xy(0), 4.5f);
	EXPECT_FLOAT_EQ(filter_xy(1), -3.5f);
	EXPECT_FLOAT_EQ(MulticopterPositionControlTestPeer::velocityFilterStateZ(*_controller), 1.25f);
}

// WHY: Run() adjusts the incoming setpoint and stores a fallback for the case where the
// control update rejects it. Both describe the same frame, so a reset has to move both in
// the same cycle. The counters are latched once, so anything not shifted here is never
// shifted at all.
TEST_F(MulticopterPositionControlTest, FallbackSetpointIsShiftedInTheSameCycleAsTheCurrentOne)
{
	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.z_reset_counter = 1;
	local_position.delta_z = -1.5f;
	local_position.xy_reset_counter = 1;
	local_position.delta_xy[0] = 2.5f;
	local_position.delta_xy[1] = -3.5f;

	trajectory_setpoint_s current = makeSetpoint();
	trajectory_setpoint_s last_valid = makeSetpoint();

	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, current, last_valid);

	EXPECT_FLOAT_EQ(current.position[2], -31.5f);
	EXPECT_FLOAT_EQ(last_valid.position[2], -31.5f);
	EXPECT_FLOAT_EQ(last_valid.position[0], 12.5f);
	EXPECT_FLOAT_EQ(last_valid.position[1], 16.5f);
}

// WHY: the fallback is stored across cycles, so a reset already applied to it must not be
// applied again on the next sample.
TEST_F(MulticopterPositionControlTest, FallbackSetpointIsNotShiftedTwice)
{
	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.z_reset_counter = 1;
	local_position.delta_z = -1.5f;

	trajectory_setpoint_s current = makeSetpoint();
	trajectory_setpoint_s last_valid = makeSetpoint();

	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, current, last_valid);
	EXPECT_FLOAT_EQ(last_valid.position[2], -31.5f);

	local_position.timestamp = hrt_absolute_time();
	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, current, last_valid);

	EXPECT_FLOAT_EQ(last_valid.position[2], -31.5f);
}

// WHY: the fallback carries its own timestamp, so it gets the same guard as any other
// setpoint rather than inheriting the decision made for the current one.
TEST_F(MulticopterPositionControlTest, FallbackSetpointHonoursItsOwnTimestampGuard)
{
	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.z_reset_counter = 1;
	local_position.delta_z = -1.5f;

	trajectory_setpoint_s current = makeSetpoint();
	trajectory_setpoint_s last_valid = makeSetpoint();
	last_valid.timestamp = local_position.timestamp + 100_ms;

	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, current, last_valid);

	EXPECT_FLOAT_EQ(current.position[2], -31.5f);
	EXPECT_FLOAT_EQ(last_valid.position[2], -30.f);
}

// WHY: two setpoints are adjusted in one call but there was only one reset, so the velocity
// filter must move by one delta, not two.
TEST_F(MulticopterPositionControlTest, VelocityFilterMovesOnceWhenBothSetpointsAreAdjusted)
{
	MulticopterPositionControlTestPeer::setVelocityFilterState(*_controller, matrix::Vector2f(4.f, -2.f), 1.f);

	vehicle_local_position_s local_position = makeLocalPosition();
	local_position.vxy_reset_counter = 1;
	local_position.delta_vxy[0] = 0.5f;
	local_position.delta_vxy[1] = -1.5f;
	local_position.vz_reset_counter = 1;
	local_position.delta_vz = 0.25f;

	trajectory_setpoint_s current = makeSetpoint();
	trajectory_setpoint_s last_valid = makeSetpoint();

	MulticopterPositionControlTestPeer::adjustSetpointForEKFResets(*_controller, local_position, current, last_valid);

	const matrix::Vector2f filter_xy = MulticopterPositionControlTestPeer::velocityFilterStateXy(*_controller);
	EXPECT_FLOAT_EQ(filter_xy(0), 4.5f);
	EXPECT_FLOAT_EQ(filter_xy(1), -3.5f);
	EXPECT_FLOAT_EQ(MulticopterPositionControlTestPeer::velocityFilterStateZ(*_controller), 1.25f);
}
