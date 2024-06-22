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

namespace uORB
{
namespace test
{


class SubscriptionTestable  : public  uORB::Subscription
{
public:
	SubscriptionTestable() : Subscription(ORB_ID(orb_test), 0)
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


class SubscriptionTest : public ::testing::Test
{
protected:
	SubscriptionTestable  testable;

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

TEST_F(SubscriptionTest, updateWhenSubscribedThenNotSubscribedTwice)
{
	int anyValue = 1;
	testable.setNodeValue(&anyValue);

	testable.updated();

	ASSERT_EQ(testable.getNodeValue(), &anyValue) << "Original node value don't have to be overrwiten";
}

TEST_F(SubscriptionTest, updateWhenNotSubscribedThenSubscribed)
{
	testable.setNodeValue(nullptr);

	testable.updated();

	ASSERT_NE(testable.getNodeValue(), nullptr) << "Node value after 'updated' have to be initialized";
}
}
}
