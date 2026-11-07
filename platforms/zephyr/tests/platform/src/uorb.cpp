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

#include "px4_test.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/orb_test.h>

#include <zephyr/ztest.h>

class DelayedPublisher : public px4::ScheduledWorkItem
{
public:
	DelayedPublisher() : ScheduledWorkItem("test_publisher", px4::wq_configurations::test1) {}

	void Run() override
	{
		orb_test_s data{};
		data.val = 42;
		data.timestamp = hrt_absolute_time();
		_publication.publish(data);
	}

private:
	uORB::Publication<orb_test_s> _publication{ORB_ID(orb_multitest)};
};

ZTEST(px4_uorb, test_publish_subscribe)
{
	uORB::Subscription subscription{ORB_ID(orb_test)};
	uORB::Publication<orb_test_s> publication{ORB_ID(orb_test)};

	orb_test_s data{};
	data.val = 1234;
	data.timestamp = hrt_absolute_time();
	zassert_true(publication.publish(data));

	orb_test_s received{};
	zassert_true(subscription.update(&received));
	zassert_equal(received.val, 1234);
	zassert_equal(received.timestamp, data.timestamp);
	zassert_false(subscription.updated());
}

ZTEST(px4_uorb, test_poll)
{
	const int fd = orb_subscribe(ORB_ID(orb_multitest));
	zassert_true(fd >= 0);

	px4_pollfd_struct_t fds{};
	fds.fd = fd;
	fds.events = POLLIN;

	hrt_abstime start = hrt_absolute_time();
	zassert_equal(px4_poll(&fds, 1, 10), 0);
	zassert_true(hrt_elapsed_time(&start) >= 10000);

	DelayedPublisher publisher;
	publisher.ScheduleDelayed(5000);

	start = hrt_absolute_time();
	zassert_equal(px4_poll(&fds, 1, 1000), 1);
	zassert_true(hrt_elapsed_time(&start) < 100000);

	orb_test_s received{};
	zassert_ok(orb_copy(ORB_ID(orb_multitest), fd, &received));
	zassert_equal(received.val, 42);

	orb_unsubscribe(fd);
}

ZTEST_SUITE(px4_uorb, NULL, px4_test_setup, NULL, NULL, NULL);
