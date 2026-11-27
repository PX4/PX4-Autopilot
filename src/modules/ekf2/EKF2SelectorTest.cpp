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

#include <drivers/drv_hrt.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_selector_status.h>

using namespace time_literals;

// filter_fault_flags bit 10: bad vertical accelerometer data
static constexpr uint32_t kFaultBadAccVertical = (1u << 10);

// The test steps the selector itself on simulated time. No work queue is
// started, so neither its scheduling nor its uORB callbacks run it.
class EKF2SelectorTestable : public EKF2Selector
{
public:
	using EKF2Selector::Update;
};

class EKF2SelectorTest : public ::testing::Test
{
public:
	// what one estimator instance publishes each cycle
	struct Status {
		bool publish;
		float test_ratio;
		uint32_t fault_flags;
	};

	static constexpr Status kGood{true, 0.1f, 0};
	static constexpr Status kWarned{true, 2.f, 0}; // nominally healthy, failing test ratio
	static constexpr Status kFaulted{true, 0.1f, kFaultBadAccVertical};
	static constexpr Status kSilent{false, 0.f, 0};

	static constexpr hrt_abstime kStep{10_ms}; // the selector's update period

	// shared across tests: a fresh PublicationMulti per test would advertise
	// new uORB instances while the selector watches 0..N
	static void SetUpTestSuite()
	{
		for (auto &pub : _status_pub) {
			ASSERT_TRUE(pub.advertise());
		}
	}

	void SetUp() override
	{
		settleBaseline();
	}

	void publishStatus(int instance, const Status &s)
	{
		if (!s.publish) {
			return;
		}

		estimator_status_s status{};
		status.timestamp_sample = _now;
		status.accel_device_id = 1000 + instance;
		status.gyro_device_id = 2000 + instance;
		status.vel_test_ratio = s.test_ratio;
		status.pos_test_ratio = s.test_ratio;
		status.hgt_test_ratio = s.test_ratio;
		status.filter_fault_flags = s.fault_flags;
		status.timestamp = _now;
		_status_pub[instance].publish(status);
	}

	// one selector cycle: advance the clock, publish, run the selector
	void step(const Status &s0, const Status &s1)
	{
		_now += kStep;
		publishStatus(0, s0);
		publishStatus(1, s1);
		_selector.Update(_now);
	}

	void runFor(hrt_abstime duration, const Status &s0, const Status &s1)
	{
		const hrt_abstime start = _now;

		while (_now - start < duration) {
			step(s0, s1);
		}
	}

	// step until the selector picks the instance or the timeout passes,
	// returns the simulated time it took
	hrt_abstime runUntilPrimary(uint8_t instance, hrt_abstime timeout, const Status &s0, const Status &s1)
	{
		const hrt_abstime start = _now;

		while ((_now - start < timeout) && (primaryInstance() != instance)) {
			step(s0, s1);
		}

		return _now - start;
	}

	// a fresh selector picks instance 0 and both pass the 1 s health hysteresis
	void settleBaseline()
	{
		const hrt_abstime start = _now;

		runFor(1500_ms, kGood, kGood);

		const estimator_selector_status_s status = selectorStatus();
		ASSERT_GT(status.timestamp, start) << "no status from this test's selector";
		ASSERT_EQ(status.primary_instance, 0);
		ASSERT_EQ(status.instances_available, 2);
		ASSERT_TRUE(status.healthy[0]);
		ASSERT_TRUE(status.healthy[1]);
	}

	// peg instance 0's test ratio until the selector switches to instance 1
	void degradeUntilSwitched()
	{
		runUntilPrimary(1, 15_s, kWarned, kGood);
		ASSERT_EQ(primaryInstance(), 1) << "selector never switched away from the degraded primary";
	}

	estimator_selector_status_s selectorStatus()
	{
		estimator_selector_status_s status{};
		_selector_status_sub.copy(&status);
		return status;
	}

	uint8_t primaryInstance() { return selectorStatus().primary_instance; }
	uint32_t instanceChangedCount() { return selectorStatus().instance_changed_count; }

	// Starts well after boot: the selector treats a never selected instance as
	// last selected at time 0 and holds it off for 10 s after that. The clock
	// keeps running across tests so that no sample left on uORB by an earlier
	// test is in the future of a later one.
	static hrt_abstime _now;
	static uORB::PublicationMulti<estimator_status_s> _status_pub[2];

	EKF2SelectorTestable _selector;
	uORB::Subscription _selector_status_sub{ORB_ID(estimator_selector_status)};
};

hrt_abstime EKF2SelectorTest::_now{100_s};
uORB::PublicationMulti<estimator_status_s> EKF2SelectorTest::_status_pub[2] {
	{ORB_ID(estimator_status)},
	{ORB_ID(estimator_status)},
};

// two healthy instances: no churn
TEST_F(EKF2SelectorTest, staysOnHealthyInstance)
{
	const uint32_t switches = instanceChangedCount();

	runFor(5_s, kGood, kGood);

	EXPECT_EQ(primaryInstance(), 0);
	EXPECT_EQ(instanceChangedCount(), switches);
}

// a hard fault on the primary fails over immediately to a clean alternative
TEST_F(EKF2SelectorTest, cleanFallbackIsImmediate)
{
	const hrt_abstime switched_after = runUntilPrimary(1, 5_s, kFaulted, kGood);

	EXPECT_EQ(primaryInstance(), 1);
	EXPECT_LT(switched_after, 1_s); // immediate, not the warned-fallback delay
}

// the switch-away half of b7efd4f947 still works
TEST_F(EKF2SelectorTest, switchesAwayFromDegradedPrimary)
{
	ASSERT_NO_FATAL_FAILURE(degradeUntilSwitched());
}

// the issue 27013 pattern: brief faults on the primary must not bounce the
// selector to a nominally healthy instance with a pegged test ratio
TEST_F(EKF2SelectorTest, noFallbackToSustainedWarnedInstance)
{
	ASSERT_NO_FATAL_FAILURE(degradeUntilSwitched());
	const uint32_t switches_before = instanceChangedCount();

	// clear windows exceed the 1 s healthy hysteresis, so these are three
	// separate faults, not one long unhealthy episode
	for (int cycle = 0; cycle < 3; cycle++) {
		runFor(800_ms, kWarned, kFaulted); // faulted primary, diverged alternative
		runFor(1600_ms, kWarned, kGood);   // fault clears, primary re-heals
	}

	EXPECT_EQ(primaryInstance(), 1);
	EXPECT_EQ(instanceChangedCount(), switches_before); // no whipsaw
}

// a timed out primary falls back to the warned instance without delay
TEST_F(EKF2SelectorTest, timedOutPrimaryFallsBackImmediately)
{
	ASSERT_NO_FATAL_FAILURE(degradeUntilSwitched());

	// primary goes silent, only the warned instance 0 keeps publishing
	const hrt_abstime switched_after = runUntilPrimary(0, 3_s, kWarned, kSilent);

	EXPECT_EQ(primaryInstance(), 0);
	EXPECT_LT(switched_after, 1_s); // well within the 5 s warned-fallback delay
}

// a persistently faulted primary still falls back to the warned instance
TEST_F(EKF2SelectorTest, sustainedFaultFallsBackEventually)
{
	ASSERT_NO_FATAL_FAILURE(degradeUntilSwitched());

	runFor(7_s, kWarned, kFaulted);

	EXPECT_EQ(primaryInstance(), 0);
}
