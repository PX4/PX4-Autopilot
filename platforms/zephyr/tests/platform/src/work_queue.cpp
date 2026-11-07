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
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <string.h>

#include <zephyr/ztest.h>

class TestItem : public px4::ScheduledWorkItem
{
public:
	TestItem() : ScheduledWorkItem("test_item", px4::wq_configurations::test1)
	{
		px4_sem_init(&sem, 0, 0);
	}

	~TestItem() override
	{
		px4_sem_destroy(&sem);
	}

	void Run() override
	{
		strncpy(thread_name, k_thread_name_get(k_current_get()), sizeof(thread_name) - 1);
		count.fetch_add(1);
		px4_sem_post(&sem);
	}

	px4_sem_t sem;
	px4::atomic<int> count{0};
	char thread_name[16] {};
};

static void schedule_now(void *arg)
{
	static_cast<TestItem *>(arg)->ScheduleNow();
}

ZTEST(px4_work_queue, test_schedule_now)
{
	TestItem item;
	item.ScheduleNow();

	zassert_ok(px4_test_sem_wait(&item.sem, 100));
	zassert_str_equal(item.thread_name, px4::wq_configurations::test1.name);
}

ZTEST(px4_work_queue, test_schedule_from_isr)
{
	TestItem item;
	struct hrt_call call {};
	hrt_call_after(&call, 1000, schedule_now, &item);

	zassert_ok(px4_test_sem_wait(&item.sem, 100));
	zassert_equal(item.count.load(), 1);
}

ZTEST(px4_work_queue, test_schedule_on_interval)
{
	TestItem item;
	item.ScheduleOnInterval(2000);
	k_sleep(K_MSEC(21));
	item.ScheduleClear();

	zassert_within(item.count.load(), 10, 1);
}

ZTEST_SUITE(px4_work_queue, NULL, px4_test_setup, NULL, NULL, NULL);
