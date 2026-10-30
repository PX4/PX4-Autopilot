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
 * Callback subscriptions destroyed while their topic publishes. A publisher calls call() until the
 * subscription is unregistered, so a subclass that leaves that to ~SubscriptionCallback() gets called
 * after its own destructor has run, which aborts with "pure virtual method called".
 */

#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>

using namespace time_literals;

namespace uORB
{
namespace test
{

// Publishes orb_test as fast as it can for as long as it exists
class Publisher
{
public:
	Publisher()
	{
		_pub.advertise();
		_thread = std::thread([this] {
			orb_test_s message{};

			while (!_stop.load())
			{
				message.val++;
				_pub.publish(message);
			}
		});
	}

	~Publisher()
	{
		_stop.store(true);
		_thread.join();
	}

private:
	uORB::Publication<orb_test_s> _pub{ORB_ID(orb_test)};
	std::atomic<bool> _stop{false};
	std::thread _thread;
};

class TestWorkItem : public px4::WorkItem
{
public:
	TestWorkItem() : WorkItem("uorb_callback_test", px4::wq_configurations::test1) {}

	void Run() override {}
};

class uORBSubscriptionCallbackTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite()
	{
		uORB::Manager::initialize();
		ASSERT_EQ(px4::WorkQueueManagerStart(), 0);

		// the manager reports running from its own thread
		const hrt_abstime start = hrt_absolute_time();

		while (px4::WorkQueueFindOrCreate(px4::wq_configurations::test1) == nullptr) {
			ASSERT_LT(hrt_elapsed_time(&start), 10_s) << "work queue manager did not start";
			px4_usleep(10_ms);
		}
	}

	static void TearDownTestSuite()
	{
		px4::WorkQueueManagerStop();
		uORB::Manager::terminate();
	}

	// Each cycle is one chance to hit the window; without the fix a few thousand are plenty
	static constexpr int CYCLES = 20000;
};

TEST_F(uORBSubscriptionCallbackTest, DestroyBlockingWhilePublishing)
{
	Publisher publisher;

	for (int i = 0; i < CYCLES; i++) {
		uORB::SubscriptionBlocking<orb_test_s> sub{ORB_ID(orb_test)};
		ASSERT_TRUE(sub.registerCallback());
	}
}

TEST_F(uORBSubscriptionCallbackTest, DestroyWorkItemSubscriptionWhilePublishing)
{
	TestWorkItem work_item;
	Publisher publisher;

	for (int i = 0; i < CYCLES; i++) {
		uORB::SubscriptionCallbackWorkItem sub{&work_item, ORB_ID(orb_test)};
		ASSERT_TRUE(sub.registerCallback());
	}
}

} // namespace test
} // namespace uORB
