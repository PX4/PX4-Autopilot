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
 * Functional test for the EKF2 instance selector, replaying the failure
 * pattern from https://github.com/PX4/PX4-Autopilot/issues/27013.
 *
 * to run: make tests TESTFILTER=EKF2Selector
 */

#include <gtest/gtest.h>

#include "EKF2Selector.hpp"

#include <hrt_work.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform_common/time.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_selector_status.h>

using namespace time_literals;

// filter_fault_flags bit 10: bad vertical accelerometer data
static constexpr uint32_t kFaultBadAccVertical = (1u << 10);

// Keep output unbuffered so a CI harness kill cannot swallow it, and stop the
// work queue manager at the end so the process exits instead of timing out.
class EKF2SelectorTestEnvironment : public ::testing::Environment
{
public:
	void SetUp() override { setvbuf(stdout, nullptr, _IONBF, 0); }
	void TearDown() override { px4::WorkQueueManagerStop(); }
};

static const auto *global_env = ::testing::AddGlobalTestEnvironment(new EKF2SelectorTestEnvironment());

// PX4 destroys a work queue when its last WorkItem detaches, so deleting the
// selector between tests would race the next test's construction against the
// dying queue. One keeper item pins the queue for the whole process.
class WorkQueueKeeper : public px4::ScheduledWorkItem
{
public:
	WorkQueueKeeper() : ScheduledWorkItem("wq_keeper", px4::wq_configurations::nav_and_controllers) {}

private:
	void Run() override {}
};

class EKF2SelectorTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		// the selector attaches to the work queue at construction
		static bool wq_manager_started = false;

		if (!wq_manager_started) {
			// the gtest harness starts no platform threads: the hrt callout
			// worker must exist before the selector's delayed self-scheduling
			// runs, or the work queue blocks on an uninitialized semaphore
			hrt_work_queue_init();
			ASSERT_EQ(px4::WorkQueueManagerStart(), 0);
			wq_manager_started = true;
		}

		// the manager thread reports running asynchronously: wait for the
		// selector's queue to be servable instead of sleeping a fixed time
		const hrt_abstime wq_wait_start = hrt_absolute_time();

		while (px4::WorkQueueFindOrCreate(px4::wq_configurations::nav_and_controllers) == nullptr) {
			ASSERT_LT(hrt_elapsed_time(&wq_wait_start), 10_s) << "work queue manager did not start";
			px4_usleep(100_ms);
		}

		static WorkQueueKeeper *wq_keeper = new WorkQueueKeeper();
		ASSERT_NE(wq_keeper, nullptr);

		// shared across tests: a fresh PublicationMulti per test would
		// advertise new uORB instances while the selector watches 0..N
		if (_status_pub[0] == nullptr) {
			_status_pub[0] = new uORB::PublicationMulti<estimator_status_s>(ORB_ID(estimator_status));
			_status_pub[1] = new uORB::PublicationMulti<estimator_status_s>(ORB_ID(estimator_status));
			_status_pub[0]->advertise();
			_status_pub[1]->advertise();
		}

		_selector = new EKF2Selector();
		ASSERT_NE(_selector, nullptr);

		settleBaseline();
	}

	void TearDown() override
	{
		_selector->Stop();
		delete _selector;
	}

	// publish one instance's estimator_status the way EKF2 would
	void publishStatus(int instance, float test_ratio, uint32_t fault_flags)
	{
		estimator_status_s status{};
		status.timestamp_sample = hrt_absolute_time();
		status.accel_device_id = 1000 + instance;
		status.gyro_device_id = 2000 + instance;
		status.vel_test_ratio = test_ratio;
		status.pos_test_ratio = test_ratio;
		status.hgt_test_ratio = test_ratio;
		status.filter_fault_flags = fault_flags;
		status.timestamp = hrt_absolute_time();
		_status_pub[instance]->publish(status);
	}

	// run the scenario for a wall-clock duration at ~100 Hz
	void runFor(hrt_abstime duration, float ratio0, uint32_t faults0, float ratio1, uint32_t faults1)
	{
		const hrt_abstime start = hrt_absolute_time();

		while (hrt_elapsed_time(&start) < duration) {
			publishStatus(0, ratio0, faults0);
			publishStatus(1, ratio1, faults1);
			_selector->ScheduleNow();
			px4_usleep(5_ms);
		}
	}

	// settle on both instances healthy with instance 0 selected
	void settleBaseline()
	{
		const hrt_abstime start = hrt_absolute_time();

		hrt_abstime last_report = start;

		// generous timeout: a loaded machine can restart the 1 s health hysteresis
		while (hrt_elapsed_time(&start) < 20_s) {
			estimator_selector_status_s status{};
			_selector_status_sub.copy(&status);

			if (hrt_elapsed_time(&last_report) > 2_s) {
				last_report = hrt_absolute_time();
				printf("settle: elapsed=%.1fs now=%llu selector status timestamp=%llu primary=%d healthy=%d/%d changed=%lu\n",
				       hrt_elapsed_time(&start) * 1e-6, (unsigned long long)last_report,
				       (unsigned long long)status.timestamp, status.primary_instance,
				       status.healthy[0], status.healthy[1], (unsigned long)status.instance_changed_count);
			}

			// only trust a status this selector instance produced
			const bool fresh = (status.timestamp > start);

			if (fresh && (status.primary_instance == 0) && (status.instances_available == 2)
			    && status.healthy[0] && status.healthy[1]) {
				return;
			}

			// a single request can be consumed during a transient
			if (!fresh || (status.primary_instance != 0)) {
				_selector->RequestInstance(0);
			}

			publishStatus(0, 0.1f, 0);
			publishStatus(1, 0.1f, 0);
			_selector->ScheduleNow();
			px4_usleep(5_ms);
		}

		estimator_selector_status_s status{};
		_selector_status_sub.copy(&status);
		FAIL() << "selector did not settle on a healthy instance 0:"
		       << " primary=" << (int)status.primary_instance
		       << " available=" << (int)status.instances_available
		       << " healthy0=" << status.healthy[0]
		       << " healthy1=" << status.healthy[1]
		       << " changed=" << status.instance_changed_count;
	}

	// peg instance 0's test ratio until the selector switches to instance 1
	void degradeUntilSwitched()
	{
		const hrt_abstime start = hrt_absolute_time();

		while (hrt_elapsed_time(&start) < 15_s) {
			publishStatus(0, 2.f, 0);
			publishStatus(1, 0.1f, 0);
			_selector->ScheduleNow();
			px4_usleep(5_ms);

			if (primaryInstance() == 1) {
				return;
			}
		}

		FAIL() << "selector never switched away from the degraded primary";
	}

	uint8_t primaryInstance()
	{
		estimator_selector_status_s status{};
		_selector_status_sub.copy(&status);
		return status.primary_instance;
	}

	uint32_t instanceChangedCount()
	{
		estimator_selector_status_s status{};
		_selector_status_sub.copy(&status);
		return status.instance_changed_count;
	}

	EKF2Selector *_selector{nullptr};
	static uORB::PublicationMulti<estimator_status_s> *_status_pub[2];
	uORB::Subscription _selector_status_sub{ORB_ID(estimator_selector_status)};
};

uORB::PublicationMulti<estimator_status_s> *EKF2SelectorTest::_status_pub[2] {nullptr, nullptr};

// two healthy instances: no churn
TEST_F(EKF2SelectorTest, staysOnHealthyInstance)
{
	const uint32_t switches = instanceChangedCount();

	runFor(500_ms, 0.1f, 0, 0.1f, 0);

	EXPECT_EQ(primaryInstance(), 0);
	EXPECT_EQ(instanceChangedCount(), switches); // no changes after settling
}

// a hard fault on the primary fails over immediately to a clean alternative
TEST_F(EKF2SelectorTest, cleanFallbackIsImmediate)
{
	const hrt_abstime fault_start = hrt_absolute_time();
	hrt_abstime switched_after = 0;

	while (hrt_elapsed_time(&fault_start) < 5_s) {
		publishStatus(0, 0.1f, kFaultBadAccVertical);
		publishStatus(1, 0.1f, 0);
		_selector->ScheduleNow();
		px4_usleep(5_ms);

		if (primaryInstance() == 1) {
			switched_after = hrt_elapsed_time(&fault_start);
			break;
		}
	}

	EXPECT_EQ(primaryInstance(), 1);
	EXPECT_GT(switched_after, 0u);
	EXPECT_LT(switched_after, 1_s); // immediate, not the warned-fallback delay
}

// the switch-away half of b7efd4f947 still works
TEST_F(EKF2SelectorTest, switchesAwayFromDegradedPrimary)
{
	degradeUntilSwitched();
	EXPECT_EQ(primaryInstance(), 1);
}

// the issue 27013 pattern: brief faults on the primary must not bounce the
// selector to a nominally healthy instance with a pegged test ratio
TEST_F(EKF2SelectorTest, noFallbackToSustainedWarnedInstance)
{
	degradeUntilSwitched();
	const uint32_t switches_before = instanceChangedCount();

	// clear windows exceed the 1 s healthy hysteresis, so these are three
	// separate faults, not one long unhealthy episode
	for (int cycle = 0; cycle < 3; cycle++) {
		runFor(800_ms, 2.f, 0, 0.1f, kFaultBadAccVertical);	// faulted primary, diverged alternative
		runFor(1600_ms, 2.f, 0, 0.1f, 0);	// fault clears, primary re-heals
	}

	EXPECT_EQ(primaryInstance(), 1);
	EXPECT_EQ(instanceChangedCount(), switches_before); // no whipsaw
}

// a timed out primary falls back to the warned instance without delay
TEST_F(EKF2SelectorTest, timedOutPrimaryFallsBackImmediately)
{
	runFor(2500_ms, 2.f, 0, 0.1f, 0);
	ASSERT_EQ(primaryInstance(), 1);

	// primary goes silent, only the warned instance 0 keeps publishing
	const hrt_abstime start = hrt_absolute_time();

	while (hrt_elapsed_time(&start) < 3_s) {
		publishStatus(0, 2.f, 0);
		_selector->ScheduleNow();
		px4_usleep(5_ms);

		if (primaryInstance() == 0) {
			break;
		}
	}

	EXPECT_EQ(primaryInstance(), 0);
}

// a persistently faulted primary still falls back to the warned instance
TEST_F(EKF2SelectorTest, sustainedFaultFallsBackEventually)
{
	degradeUntilSwitched();

	runFor(7000_ms, 2.f, 0, 0.1f, kFaultBadAccVertical);
	EXPECT_EQ(primaryInstance(), 0);
}
