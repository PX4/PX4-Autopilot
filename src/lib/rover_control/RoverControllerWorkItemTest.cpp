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
 * Functional test for RoverControllerWorkItem: the contract the rover modules rely on to run each controller on its own
 * trigger topic, only while enabled, and in cascade order when controllers share a trigger topic.
 */

#include <gtest/gtest.h>

#include "RoverControllerWorkItem.hpp"

#include <atomic>
#include <drivers/drv_hrt.h>
#include <hrt_work.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_angular_velocity.h>

using namespace time_literals;

namespace
{

// Stop the work queue manager at the end so the process exits instead of timing out
class RoverControllerWorkItemTestEnvironment : public ::testing::Environment
{
public:
	void TearDown() override { px4::WorkQueueManagerStop(); }
};

const auto *global_env = ::testing::AddGlobalTestEnvironment(new RoverControllerWorkItemTestEnvironment());

// Global run sequence, to check the order in which controllers sharing a trigger run
std::atomic<int> run_sequence{0};

class TestController : public RoverControllerWorkItem
{
public:
	TestController(const char *name) : RoverControllerWorkItem(name, ORB_ID(vehicle_angular_velocity)) {}

	std::atomic<int> runs{0};
	std::atomic<int> enables{0};
	std::atomic<int> last_run_sequence{0};

private:
	void runController() override
	{
		last_run_sequence = ++run_sequence;
		++runs;
	}

	void onEnable() override { ++enables; }
};

class RoverControllerWorkItemTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		static bool wq_manager_started = false;

		if (!wq_manager_started) {
			// The gtest harness starts no platform threads, the hrt callout worker must exist before the work queues run
			hrt_init();
			hrt_work_queue_init();
			ASSERT_EQ(px4::WorkQueueManagerStart(), 0);
			wq_manager_started = true;
		}

		const hrt_abstime wq_wait_start = hrt_absolute_time();

		while (px4::WorkQueueFindOrCreate(px4::wq_configurations::nav_and_controllers) == nullptr) {
			ASSERT_LT(hrt_elapsed_time(&wq_wait_start), 10_s) << "work queue manager did not start";
			px4_usleep(10_ms);
		}

		// Keep the controllers alive for the whole process: a work queue is destroyed when its last item detaches,
		// which would race the next test's construction against the dying queue
		static TestController *upper = new TestController("test_upper_control");
		static TestController *lower = new TestController("test_lower_control");
		_upper = upper;
		_lower = lower;
		_upper->setEnabled(false);
		_lower->setEnabled(false);
		settle();
	}

protected:
	void publishTrigger()
	{
		vehicle_angular_velocity_s angular_velocity{};
		angular_velocity.timestamp = hrt_absolute_time();
		_trigger_pub.publish(angular_velocity);
		settle();
	}

	// Let the work queue run everything that is scheduled
	static void settle() { px4_usleep(20_ms); }

	TestController *_upper{nullptr};
	TestController *_lower{nullptr};
	uORB::Publication<vehicle_angular_velocity_s> _trigger_pub{ORB_ID(vehicle_angular_velocity)};
};

} // namespace

TEST_F(RoverControllerWorkItemTest, InitRegistersAndReleasesTrigger)
{
	EXPECT_TRUE(_upper->init());
	EXPECT_FALSE(_upper->enabled());

	const int runs = _upper->runs;
	publishTrigger();
	EXPECT_EQ(_upper->runs, runs); // init() must not leave the callback registered
}

TEST_F(RoverControllerWorkItemTest, DisabledControllerIsNotWoken)
{
	const int runs = _upper->runs;
	publishTrigger();
	publishTrigger();
	EXPECT_EQ(_upper->runs, runs);
}

TEST_F(RoverControllerWorkItemTest, EnablingRunsOnceAndThenOnEveryTrigger)
{
	const int runs = _upper->runs;
	const int enables = _upper->enables;

	EXPECT_TRUE(_upper->setEnabled(true));
	EXPECT_TRUE(_upper->enabled());
	settle();
	EXPECT_EQ(_upper->enables, enables + 1);
	EXPECT_EQ(_upper->runs, runs + 1); // runs right away, without waiting for the next trigger

	// Staying enabled neither calls onEnable() again nor schedules an extra run
	EXPECT_TRUE(_upper->setEnabled(true));
	settle();
	EXPECT_EQ(_upper->enables, enables + 1);
	EXPECT_EQ(_upper->runs, runs + 1);

	publishTrigger();
	publishTrigger();
	EXPECT_EQ(_upper->runs, runs + 3);
}

TEST_F(RoverControllerWorkItemTest, DisablingStopsTheController)
{
	EXPECT_TRUE(_upper->setEnabled(true));
	settle();
	EXPECT_TRUE(_upper->setEnabled(false));
	EXPECT_FALSE(_upper->enabled());

	const int runs = _upper->runs;
	publishTrigger();
	EXPECT_EQ(_upper->runs, runs);
}

TEST_F(RoverControllerWorkItemTest, ControllersSharingTheTriggerRunInRegistrationOrder)
{
	// The lower controller is enabled first, so it runs first
	EXPECT_TRUE(_lower->setEnabled(true));
	EXPECT_TRUE(_upper->setEnabled(true));
	settle();
	publishTrigger();
	EXPECT_LT(_lower->last_run_sequence, _upper->last_run_sequence);

	// Re-registering the lower controller moves it behind the upper one, as the modules do to keep the cascade order
	EXPECT_TRUE(_lower->reregisterTrigger());
	EXPECT_TRUE(_lower->enabled());
	publishTrigger();
	EXPECT_LT(_upper->last_run_sequence, _lower->last_run_sequence);
}

TEST_F(RoverControllerWorkItemTest, ReregisteringADisabledControllerKeepsItDisabled)
{
	EXPECT_TRUE(_lower->reregisterTrigger());
	EXPECT_FALSE(_lower->enabled());

	const int runs = _lower->runs;
	publishTrigger();
	EXPECT_EQ(_lower->runs, runs);
}
