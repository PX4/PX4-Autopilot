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
 * Interval gating of SubscriptionInterval::updated() / update().
 */

#include <gtest/gtest.h>

#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/orb_test.h>

namespace uORB
{
namespace test
{

class uORBSubscriptionIntervalTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite()
	{
		uORB::Manager::initialize();
	}

	static void TearDownTestSuite()
	{
		uORB::Manager::terminate();
	}

	void publish(int32_t val)
	{
		orb_test_s message{};
		message.timestamp = hrt_absolute_time();
		message.val = val;
		ASSERT_TRUE(_pub.publish(message));
	}

	uORB::Publication<orb_test_s> _pub{ORB_ID(orb_test)};
};

TEST_F(uORBSubscriptionIntervalTest, noIntervalReportsEveryPublication)
{
	uORB::SubscriptionInterval sub{ORB_ID(orb_test)};
	orb_test_s msg{};

	EXPECT_FALSE(sub.updated());

	publish(1);
	EXPECT_TRUE(sub.updated());
	EXPECT_TRUE(sub.update(&msg));
	EXPECT_EQ(msg.val, 1);
	EXPECT_FALSE(sub.updated());
	EXPECT_FALSE(sub.update(&msg));

	publish(2);
	EXPECT_TRUE(sub.update(&msg));
	EXPECT_EQ(msg.val, 2);
}

TEST_F(uORBSubscriptionIntervalTest, intervalGatesUpdates)
{
	static constexpr uint32_t interval_us = 200000;

	uORB::SubscriptionInterval sub{ORB_ID(orb_test), interval_us};
	orb_test_s msg{};

	// The first copy only aligns the schedule; the gate is armed from the second copy on.
	publish(1);
	EXPECT_TRUE(sub.update(&msg));
	publish(2);
	EXPECT_TRUE(sub.update(&msg));
	EXPECT_EQ(msg.val, 2);

	publish(3);
	EXPECT_FALSE(sub.updated());
	EXPECT_FALSE(sub.update(&msg));
	EXPECT_EQ(msg.val, 2);

	px4_usleep(interval_us + interval_us / 2);
	EXPECT_TRUE(sub.updated());
	EXPECT_TRUE(sub.update(&msg));
	EXPECT_EQ(msg.val, 3);

	// No new data: elapsed interval alone must not report an update.
	px4_usleep(interval_us + interval_us / 2);
	EXPECT_FALSE(sub.updated());
}

} // namespace test
} // namespace uORB
