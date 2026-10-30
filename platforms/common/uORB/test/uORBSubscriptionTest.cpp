/****************************************************************************
 *
 *   Copyright (c) 2019-2024 PX4 Development Team. All rights reserved.
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
 * Test for Subscription
 */

#include <gtest/gtest.h>
#include <uORB/Subscription.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/orb_test_medium.h>

namespace uORB
{
namespace test
{


class uORBSubscriptionTestable  : public  uORB::Subscription
{
public:
	uORBSubscriptionTestable() : Subscription(ORB_ID(orb_test), 0)
	{
	}

	void setNodeValue(void *node)
	{
		_node = node;

	}
	void *getNodeValue()
	{
		return _node;
	}

};


class uORBSubscriptionTest : public ::testing::Test
{
protected:
	uORBSubscriptionTestable  testable;

	static void SetUpTestSuite()
	{
		uORB::Manager::initialize();

		orb_test_s message{};
		orb_advertise(ORB_ID(orb_test), &message);

	}
	static void TearDownTestSuite()
	{
		uORB::Manager::terminate();
	}

	void TearDown() override
	{
		testable.setNodeValue(nullptr);
	}


};

TEST_F(uORBSubscriptionTest, updateWhenSubscribedThenNotSubscribedTwice)
{
	int anyValue = 1;
	testable.setNodeValue(&anyValue);

	testable.updated();

	ASSERT_EQ(testable.getNodeValue(), &anyValue) << "Original node value don't have to be overrwiten";
}

TEST_F(uORBSubscriptionTest, updateWhenNotSubscribedThenSubscribed)
{
	testable.setNodeValue(nullptr);

	testable.updated();

	ASSERT_NE(testable.getNodeValue(), nullptr) << "Node value after 'updated' have to be initialized";
}

TEST_F(uORBSubscriptionTest, subscribeAfterFailedLookupsOnceTopicIsAdvertised)
{
	// orb_test_medium has no device node yet: repeated lookups fail
	uORB::Subscription sub{ORB_ID(orb_test_medium)};
	ASSERT_FALSE(sub.advertised());
	ASSERT_FALSE(sub.updated());
	ASSERT_FALSE(sub.valid());
	EXPECT_EQ(sub.get_last_generation(), 0u) << "Generation of an unsubscribed subscription must stay 0";

	orb_test_medium_s message{};
	orb_advert_t pub = orb_advertise(ORB_ID(orb_test_medium), &message);
	ASSERT_NE(pub, nullptr);

	EXPECT_TRUE(sub.advertised()) << "Adding the device node must make the next lookup succeed";
	EXPECT_TRUE(sub.valid());

	orb_unadvertise(pub);
}
}
}
