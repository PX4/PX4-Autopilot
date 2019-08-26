/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

#include "uORBTest_UnitTest.hpp"
#include "../uORBCommon.hpp"
#include <px4_config.h>
#include <px4_time.h>
#include <stdio.h>
#include <errno.h>
#include <poll.h>
#include <math.h>
#include <lib/cdev/CDev.hpp>

ORB_DEFINE(orb_test, struct orb_test, sizeof(orb_test), "ORB_TEST:int val;hrt_abstime time;");
ORB_DEFINE(orb_multitest, struct orb_test, sizeof(orb_test), "ORB_MULTITEST:int val;hrt_abstime time;");

ORB_DEFINE(orb_test_medium, struct orb_test_medium, sizeof(orb_test_medium),
	   "ORB_TEST_MEDIUM:int val;hrt_abstime time;char[64] junk;");
ORB_DEFINE(orb_test_medium_multi, struct orb_test_medium, sizeof(orb_test_medium),
	   "ORB_TEST_MEDIUM_MULTI:int val;hrt_abstime time;char[64] junk;");
ORB_DEFINE(orb_test_medium_queue, struct orb_test_medium, sizeof(orb_test_medium),
	   "ORB_TEST_MEDIUM_MULTI:int val;hrt_abstime time;char[64] junk;");
ORB_DEFINE(orb_test_medium_queue_poll, struct orb_test_medium, sizeof(orb_test_medium),
	   "ORB_TEST_MEDIUM_MULTI:int val;hrt_abstime time;char[64] junk;");

ORB_DEFINE(orb_test_large, struct orb_test_large, sizeof(orb_test_large),
	   "ORB_TEST_LARGE:int val;hrt_abstime time;char[512] junk;");

uORBTest::UnitTest &uORBTest::UnitTest::instance()
{
	static uORBTest::UnitTest t;
	return t;
}

int uORBTest::UnitTest::pubsublatency_main()
{
	/* poll on test topic and output latency */
	float latency_integral = 0.0f;

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[3] {};

	int test_multi_sub = orb_subscribe_multi(ORB_ID(orb_test), 0);
	int test_multi_sub_medium = orb_subscribe_multi(ORB_ID(orb_test_medium), 0);
	int test_multi_sub_large = orb_subscribe_multi(ORB_ID(orb_test_large), 0);

	orb_test_large t{};

	/* clear all ready flags */
	orb_copy(ORB_ID(orb_test), test_multi_sub, &t);
	orb_copy(ORB_ID(orb_test_medium), test_multi_sub_medium, &t);
	orb_copy(ORB_ID(orb_test_large), test_multi_sub_large, &t);

	fds[0].fd = test_multi_sub;
	fds[0].events = POLLIN;
	fds[1].fd = test_multi_sub_medium;
	fds[1].events = POLLIN;
	fds[2].fd = test_multi_sub_large;
	fds[2].events = POLLIN;

	const unsigned maxruns = 1000;
	unsigned timingsgroup = 0;
	int current_value = t.val;
	int num_missed = 0;

	// timings has to be on the heap to keep frame size below 2048 bytes
	unsigned *timings = new unsigned[maxruns];
	unsigned timing_min = 9999999, timing_max = 0;

	for (unsigned i = 0; i < maxruns; i++) {
		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test), test_multi_sub, &t);
			timingsgroup = 0;

		} else if (fds[1].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test_medium), test_multi_sub_medium, &t);
			timingsgroup = 1;

		} else if (fds[2].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test_large), test_multi_sub_large, &t);
			timingsgroup = 2;
		}

		if (pret < 0) {
			PX4_ERR("poll error %d, %d", pret, errno);
			continue;
		}

		num_missed += t.val - current_value - 1;
		current_value = t.val;

		unsigned elt = (unsigned)hrt_elapsed_time_atomic(&t.time);
		latency_integral += elt;
		timings[i] = elt;

		if (elt > timing_max) {
			timing_max = elt;
		}

		if (elt < timing_min) {
			timing_min = elt;
		}
	}

	orb_unsubscribe(test_multi_sub);
	orb_unsubscribe(test_multi_sub_medium);
	orb_unsubscribe(test_multi_sub_large);

	if (pubsubtest_print) {
		char fname[32];
		sprintf(fname, PX4_STORAGEDIR"/uorb_timings%u.txt", timingsgroup);
		FILE *f = fopen(fname, "w");

		if (f == nullptr) {
			PX4_ERR("Error opening file!");
			delete[] timings;
			return PX4_ERROR;
		}

		for (unsigned i = 0; i < maxruns; i++) {
			fprintf(f, "%u\n", timings[i]);
		}

		fclose(f);
	}


	float std_dev = 0.f;
	float mean = latency_integral / maxruns;

	for (unsigned i = 0; i < maxruns; i++) {
		float diff = (float)timings[i] - mean;
		std_dev += diff * diff;
	}

	delete[] timings;

	PX4_INFO("mean:    %8.4f us", static_cast<double>(mean));
	PX4_INFO("std dev: %8.4f us", static_cast<double>(sqrtf(std_dev / (maxruns - 1))));
	PX4_INFO("min:     %3i us", timing_min);
	PX4_INFO("max:     %3i us", timing_max);
	PX4_INFO("missed topic updates: %i", num_missed);

	pubsubtest_passed = true;

	if (static_cast<float>(latency_integral / maxruns) > 100.0f) {
		pubsubtest_res = PX4_ERROR;

	} else {
		pubsubtest_res = PX4_OK;
	}

	return pubsubtest_res;
}

int uORBTest::UnitTest::test()
{
	int ret = test_single();

	if (ret != OK) {
		return ret;
	}

	ret = test_multi();

	if (ret != OK) {
		return ret;
	}

	ret = test_multi_reversed();

	if (ret != OK) {
		return ret;
	}

	ret = test_unadvertise();

	if (ret != OK) {
		return ret;
	}

	ret = test_multi2();

	if (ret != OK) {
		return ret;
	}

	ret = test_queue();

	if (ret != OK) {
		return ret;
	}

	return test_queue_poll_notify();
}

int uORBTest::UnitTest::test_unadvertise()
{
	PX4_INFO("Testing unadvertise");

	//we still have the advertisements from the previous test_multi calls.
	for (int i = 0; i < 4; ++i) {
		int ret = orb_unadvertise(_pfd[i]);

		if (ret != PX4_OK) {
			PX4_ERR("orb_unadvertise failed (%i)", ret);
			return PX4_ERROR;
		}
	}

	//try to advertise and see whether we get the right instance
	int instance_test[4];
	struct orb_test t;

	for (int i = 0; i < 4; ++i) {
		_pfd[i] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance_test[i], ORB_PRIO_MAX);

		if (instance_test[i] != i) {
			PX4_ERR("got wrong instance (should be %i, is %i)", i, instance_test[i]);
			return PX4_ERROR;
		}
	}

	for (int i = 0; i < 4; ++i) {
		orb_unadvertise(_pfd[i]);
	}

	PX4_INFO("PASS unadvertise");
	return PX4_OK;
}


int uORBTest::UnitTest::info()
{
	return OK;
}

int uORBTest::UnitTest::test_single()
{
	PX4_INFO("try single-topic support");

	struct orb_test t, u;
	int sfd;
	orb_advert_t ptopic;
	bool updated;

	t.val = 0;
	ptopic = orb_advertise(ORB_ID(orb_test), &t);

	if (ptopic == nullptr) {
		PX4_ERR("advertise failed: %d", errno);
		return PX4_ERROR;
	}

	PX4_INFO("publish handle %p", ptopic);
	sfd = orb_subscribe(ORB_ID(orb_test));

	if (sfd < 0) {
		PX4_ERR("subscribe failed: %d", errno);
		return PX4_ERROR;
	}

	PX4_INFO("subscribe fd %d", sfd);
	u.val = 1;

	if (PX4_OK != orb_copy(ORB_ID(orb_test), sfd, &u)) {
		PX4_ERR("copy(1) failed: %d", errno);
		return PX4_ERROR;
	}

	if (u.val != t.val) {
		PX4_ERR("copy(1) mismatch: %d expected %d", u.val, t.val);
		return PX4_ERROR;
	}

	if (PX4_OK != orb_check(sfd, &updated)) {
		PX4_ERR("check(1) failed");
		return PX4_ERROR;
	}

	if (updated) {
		PX4_ERR("spurious updated flag");
		return PX4_ERROR;
	}

	t.val = 2;
	PX4_INFO("try publish");

	if (PX4_OK != orb_publish(ORB_ID(orb_test), ptopic, &t)) {
		PX4_ERR("publish failed");
		return PX4_ERROR;
	}

	if (PX4_OK != orb_check(sfd, &updated)) {
		PX4_ERR("check(2) failed");
		return PX4_ERROR;
	}

	if (!updated) {
		PX4_ERR("missing updated flag");
		return PX4_ERROR;
	}

	if (PX4_OK != orb_copy(ORB_ID(orb_test), sfd, &u)) {
		PX4_ERR("copy(2) failed: %d", errno);
		return PX4_ERROR;
	}

	if (u.val != t.val) {
		PX4_ERR("copy(2) mismatch: %d expected %d", u.val, t.val);
		return PX4_ERROR;
	}

	orb_unsubscribe(sfd);

	int ret = orb_unadvertise(ptopic);

	if (ret != PX4_OK) {
		PX4_ERR("orb_unadvertise failed: %i", ret);
		return PX4_ERROR;
	}

	PX4_INFO("PASS single-topic test");
	return PX4_OK;
}

int uORBTest::UnitTest::test_multi()
{
	/* this routine tests the multi-topic support */
	PX4_INFO("try multi-topic support");

	struct orb_test t {}, u {};
	t.val = 0;
	int instance0;
	_pfd[0] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance0, ORB_PRIO_MAX);

	PX4_INFO("advertised");

	int instance1;
	_pfd[1] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance1, ORB_PRIO_MIN);

	if (instance0 != 0) {
		PX4_ERR("mult. id0: %d", instance0);
		return PX4_ERROR;
	}

	if (instance1 != 1) {
		PX4_ERR("mult. id1: %d", instance1);
		return PX4_ERROR;
	}

	t.val = 103;

	if (PX4_OK != orb_publish(ORB_ID(orb_multitest), _pfd[0], &t)) {
		PX4_ERR("mult. pub0 fail");
		return PX4_ERROR;
	}

	PX4_INFO("published");

	t.val = 203;

	if (PX4_OK != orb_publish(ORB_ID(orb_multitest), _pfd[1], &t)) {
		PX4_ERR("mult. pub1 fail");
		return PX4_ERROR;
	}

	/* subscribe to both topics and ensure valid data is received */
	int sfd0 = orb_subscribe_multi(ORB_ID(orb_multitest), 0);

	if (PX4_OK != orb_copy(ORB_ID(orb_multitest), sfd0, &u)) {
		PX4_ERR("sub #0 copy failed: %d", errno);
		return PX4_ERROR;
	}

	if (u.val != 103) {
		PX4_ERR("sub #0 val. mismatch: %d", u.val);
		return PX4_ERROR;
	}

	int sfd1 = orb_subscribe_multi(ORB_ID(orb_multitest), 1);

	if (PX4_OK != orb_copy(ORB_ID(orb_multitest), sfd1, &u)) {
		PX4_ERR("sub #1 copy failed: %d", errno);
		return PX4_ERROR;
	}

	if (u.val != 203) {
		PX4_ERR("sub #1 val. mismatch: %d", u.val);
		return PX4_ERROR;
	}

	/* test priorities */
	int prio;

	if (PX4_OK != orb_priority(sfd0, &prio)) {
		PX4_ERR("prio #0");
		return PX4_ERROR;
	}

	if (prio != ORB_PRIO_MAX) {
		PX4_ERR("prio: %d", prio);
		return PX4_ERROR;
	}

	if (PX4_OK != orb_priority(sfd1, &prio)) {
		PX4_ERR("prio #1");
		return PX4_ERROR;
	}

	if (prio != ORB_PRIO_MIN) {
		PX4_ERR("prio: %d", prio);
		return PX4_ERROR;
	}

	if (PX4_OK != latency_test<struct orb_test>(ORB_ID(orb_test), false)) {
		PX4_ERR("latency test failed");
		return PX4_ERROR;
	}

	orb_unsubscribe(sfd0);
	orb_unsubscribe(sfd1);

	PX4_INFO("PASS multi-topic test");
	return PX4_OK;
}

int uORBTest::UnitTest::pub_test_multi2_entry(int argc, char *argv[])
{
	uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
	return t.pub_test_multi2_main();
}

int uORBTest::UnitTest::pub_test_multi2_main()
{
	int data_next_idx = 0;
	const int num_instances = 3;
	orb_advert_t orb_pub[num_instances];
	struct orb_test_medium data_topic;

	for (int i = 0; i < num_instances; ++i) {
		orb_advert_t &pub = orb_pub[i];
		int idx = i;
//		PX4_WARN("advertise %i, t=%" PRIu64, i, hrt_absolute_time());
		pub = orb_advertise_multi(ORB_ID(orb_test_medium_multi), &data_topic, &idx, ORB_PRIO_DEFAULT);

		if (idx != i) {
			_thread_should_exit = true;
			PX4_ERR("Got wrong instance! should be: %i, but is %i", i, idx);
			return -1;
		}
	}

	px4_usleep(100 * 1000);

	int message_counter = 0, num_messages = 50 * num_instances;

	while (message_counter++ < num_messages) {
		px4_usleep(2); //make sure the timestamps are different
		orb_advert_t &pub = orb_pub[data_next_idx];

		data_topic.time = hrt_absolute_time();
		data_topic.val = data_next_idx;

		orb_publish(ORB_ID(orb_test_medium_multi), pub, &data_topic);

		data_next_idx = (data_next_idx + 1) % num_instances;

		if (data_next_idx == 0) {
			px4_usleep(50 * 1000);
		}
	}

	px4_usleep(100 * 1000);
	_thread_should_exit = true;

	for (int i = 0; i < num_instances; ++i) {
		orb_unadvertise(orb_pub[i]);
	}

	return 0;
}

int uORBTest::UnitTest::test_multi2()
{
	PX4_INFO("Testing multi-topic 2 test (queue simulation)");
	//test: first subscribe, then advertise

	_thread_should_exit = false;
	const int num_instances = 3;
	int orb_data_fd[num_instances];
	int orb_data_next = 0;

	for (int i = 0; i < num_instances; ++i) {
		orb_data_fd[i] = orb_subscribe_multi(ORB_ID(orb_test_medium_multi), i);
	}

	char *const args[1] = { nullptr };
	int pubsub_task = px4_task_spawn_cmd("uorb_test_multi",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MAX - 5,
					     2000,
					     (px4_main_t)&uORBTest::UnitTest::pub_test_multi2_entry,
					     args);

	if (pubsub_task < 0) {
		PX4_ERR("failed launching task");
		return PX4_ERROR;
	}

	hrt_abstime last_time = 0;

	while (!_thread_should_exit) {

		bool updated = false;
		int orb_data_cur_fd = orb_data_fd[orb_data_next];
		orb_check(orb_data_cur_fd, &updated);

		if (updated) {
			orb_test_medium msg{};
			orb_copy(ORB_ID(orb_test_medium_multi), orb_data_cur_fd, &msg);

			px4_usleep(10000);

			if (last_time >= msg.time && last_time != 0) {
				PX4_ERR("Timestamp not increasing! (%" PRIu64 " >= %" PRIu64 ")", last_time, msg.time);
				return PX4_ERROR;
			}

			last_time = msg.time;

			orb_data_next = (orb_data_next + 1) % num_instances;
		}
	}

	for (int i = 0; i < num_instances; ++i) {
		orb_unsubscribe(orb_data_fd[i]);
	}

	PX4_INFO("PASS multi-topic 2 test (queue simulation)");
	return PX4_OK;
}

int uORBTest::UnitTest::test_multi_reversed()
{
	PX4_INFO("try multi-topic support subscribing before publishing");

	/* For these tests 0 and 1 instances are taken from before, therefore continue with 2 and 3. */

	/* Subscribe first and advertise afterwards. */
	int sfd2 = orb_subscribe_multi(ORB_ID(orb_multitest), 2);

	if (sfd2 < 0) {
		PX4_ERR("sub. id2: ret: %d", sfd2);
		return PX4_ERROR;
	}

	struct orb_test t {}, u {};

	t.val = 0;

	int instance2;

	_pfd[2] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance2, ORB_PRIO_MAX);

	int instance3;

	_pfd[3] = orb_advertise_multi(ORB_ID(orb_multitest), &t, &instance3, ORB_PRIO_MIN);

	PX4_INFO("advertised");

	if (instance2 != 2) {
		PX4_ERR("mult. id2: %d", instance2);
		return PX4_ERROR;
	}

	if (instance3 != 3) {
		PX4_ERR("mult. id3: %d", instance3);
		return PX4_ERROR;
	}

	t.val = 204;

	if (PX4_OK != orb_publish(ORB_ID(orb_multitest), _pfd[2], &t)) {
		PX4_ERR("mult. pub0 fail");
		return PX4_ERROR;
	}


	t.val = 304;

	if (PX4_OK != orb_publish(ORB_ID(orb_multitest), _pfd[3], &t)) {
		PX4_ERR("mult. pub1 fail");
		return PX4_ERROR;
	}

	PX4_INFO("published");

	if (PX4_OK != orb_copy(ORB_ID(orb_multitest), sfd2, &u)) {
		PX4_ERR("sub #2 copy failed: %d", errno);
		return PX4_ERROR;
	}

	if (u.val != 204) {
		PX4_ERR("sub #3 val. mismatch: %d", u.val);
		return PX4_ERROR;
	}

	int sfd3 = orb_subscribe_multi(ORB_ID(orb_multitest), 3);

	if (PX4_OK != orb_copy(ORB_ID(orb_multitest), sfd3, &u)) {
		PX4_ERR("sub #3 copy failed: %d", errno);
		return PX4_ERROR;
	}

	if (u.val != 304) {
		PX4_ERR("sub #3 val. mismatch: %d", u.val);
		return PX4_ERROR;
	}

	PX4_INFO("PASS multi-topic reversed");
	return PX4_OK;
}

int uORBTest::UnitTest::test_queue()
{
	PX4_INFO("Testing orb queuing");

	orb_test_medium t{};
	orb_test_medium u{};

	int sfd = orb_subscribe(ORB_ID(orb_test_medium_queue));

	if (sfd < 0) {
		PX4_ERR("subscribe failed: %d", errno);
		return PX4_ERROR;
	}

	const int queue_size = 11;
	t.val = 0;
	orb_advert_t ptopic = orb_advertise_queue(ORB_ID(orb_test_medium_queue), &t, queue_size);

	if (ptopic == nullptr) {
		PX4_ERR("advertise failed: %d", errno);
		return PX4_ERROR;
	}

	bool updated = false;
	orb_check(sfd, &updated);

	if (!updated) {
		PX4_ERR("update flag not set");
		return PX4_ERROR;
	}

	if (PX4_OK != orb_copy(ORB_ID(orb_test_medium_queue), sfd, &u)) {
		PX4_ERR("copy(1) failed: %d", errno);
		return PX4_ERROR;
	}

	if (u.val != t.val) {
		PX4_ERR("copy(1) mismatch: %d expected %d", u.val, t.val);
		return PX4_ERROR;
	}

	orb_check(sfd, &updated);

	if (updated) {
		PX4_ERR("spurious updated flag");
		return PX4_ERROR;
	}

	// no messages in the queue anymore

	PX4_INFO("  Testing to write some elements...");

	for (int i = 0; i < queue_size - 2; ++i) {
		t.val = i;
		orb_publish(ORB_ID(orb_test_medium_queue), ptopic, &t);
	}

	for (int i = 0; i < queue_size - 2; ++i) {
		orb_check(sfd, &updated);

		if (!updated) {
			PX4_ERR("update flag not set, i %i", i);
			return PX4_ERROR;
		}

		orb_copy(ORB_ID(orb_test_medium_queue), sfd, &u);

		if (u.val != i) {
			PX4_ERR("got wrong element from the queue (got %i, should be %i)", u.val, i);
			return PX4_ERROR;
		}
	}

	orb_check(sfd, &updated);

	if (updated) {
		PX4_ERR("update flag set, queue_size %i", queue_size);
		return PX4_ERROR;
	}

	PX4_INFO("  Testing overflow...");
	int overflow_by = 3;

	for (int i = 0; i < queue_size + overflow_by; ++i) {
		t.val = i;
		orb_publish(ORB_ID(orb_test_medium_queue), ptopic, &t);
	}

	for (int i = 0; i < queue_size; ++i) {
		orb_check(sfd, &updated);

		if (!updated) {
			PX4_ERR("update flag not set, i %i", i);
			return PX4_ERROR;
		}

		orb_copy(ORB_ID(orb_test_medium_queue), sfd, &u);

		if (u.val != i + overflow_by) {
			PX4_ERR("got wrong element from the queue (got %i, should be %i)", u.val, i + overflow_by);
			return PX4_ERROR;
		}
	}

	orb_check(sfd, &updated);

	if (updated) {
		PX4_ERR("update flag set, queue_size %i", queue_size);
		return PX4_ERROR;
	}

	PX4_INFO("  Testing underflow...");

	for (int i = 0; i < queue_size; ++i) {
		orb_check(sfd, &updated);

		if (updated) {
			PX4_ERR("update flag set, i %i", i);
			return PX4_ERROR;
		}

		orb_copy(ORB_ID(orb_test_medium_queue), sfd, &u);

		if (u.val != queue_size + overflow_by - 1) {
			PX4_ERR("got wrong element from the queue (got %i, should be %i)", u.val, queue_size + overflow_by - 1);
			return PX4_ERROR;
		}
	}

	t.val = 943;
	orb_publish(ORB_ID(orb_test_medium_queue), ptopic, &t);

	orb_check(sfd, &updated);

	if (!updated) {
		PX4_ERR("update flag not set");
		return PX4_ERROR;
	}


	orb_copy(ORB_ID(orb_test_medium_queue), sfd, &u);

	if (u.val != t.val) {
		PX4_ERR("got wrong element from the queue (got %i, should be %i)", u.val, t.val);
		return PX4_ERROR;
	}

	orb_unadvertise(ptopic);

	PX4_INFO("PASS orb queuing");
	return PX4_OK;
}


int uORBTest::UnitTest::pub_test_queue_entry(int argc, char *argv[])
{
	uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
	return t.pub_test_queue_main();
}

int uORBTest::UnitTest::pub_test_queue_main()
{
	struct orb_test_medium t;
	orb_advert_t ptopic;
	const int queue_size = 50;
	t.val = 0;

	if ((ptopic = orb_advertise_queue(ORB_ID(orb_test_medium_queue_poll), &t, queue_size)) == nullptr) {
		_thread_should_exit = true;
		PX4_ERR("advertise failed: %d", errno);
		return PX4_ERROR;
	}

	int message_counter = 0, num_messages = 20 * queue_size;
	++t.val;

	while (message_counter < num_messages) {

		//simulate burst
		int burst_counter = 0;

		while (burst_counter++ < queue_size / 2 + 7) { //make interval non-boundary aligned
			orb_publish(ORB_ID(orb_test_medium_queue_poll), ptopic, &t);
			++t.val;
		}

		message_counter += burst_counter;
		px4_usleep(20 * 1000); //give subscriber a chance to catch up
	}

	_num_messages_sent = t.val;
	px4_usleep(100 * 1000);
	_thread_should_exit = true;
	orb_unadvertise(ptopic);

	return 0;
}

int uORBTest::UnitTest::test_queue_poll_notify()
{
	PX4_INFO("Testing orb queuing (poll & notify)");

	struct orb_test_medium t;
	int sfd;

	if ((sfd = orb_subscribe(ORB_ID(orb_test_medium_queue_poll))) < 0) {
		PX4_ERR("subscribe failed: %d", errno);
		return PX4_ERROR;
	}

	_thread_should_exit = false;

	char *const args[1] = { nullptr };
	int pubsub_task = px4_task_spawn_cmd("uorb_test_queue",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MIN + 5,
					     1500,
					     (px4_main_t)&uORBTest::UnitTest::pub_test_queue_entry,
					     args);

	if (pubsub_task < 0) {
		PX4_ERR("failed launching task");
		return PX4_ERROR;
	}

	int next_expected_val = 0;
	px4_pollfd_struct_t fds[1];
	fds[0].fd = sfd;
	fds[0].events = POLLIN;

	while (!_thread_should_exit) {

		int poll_ret = px4_poll(fds, 1, 500);

		if (poll_ret == 0) {
			if (_thread_should_exit) {
				break;
			}

			PX4_ERR("poll timeout");
			return PX4_ERROR;

		} else if (poll_ret < 0) {
			PX4_ERR("poll error (%d, %d)", poll_ret, errno);
			return PX4_ERROR;
		}

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(orb_test_medium_queue_poll), sfd, &t);

			if (next_expected_val != t.val) {
				PX4_ERR("copy mismatch: %d expected %d", t.val, next_expected_val);
				return PX4_ERROR;
			}

			++next_expected_val;
		}
	}

	if (_num_messages_sent != next_expected_val) {
		PX4_ERR("number of sent and received messages mismatch (sent: %i, received: %i)", _num_messages_sent,
			next_expected_val);
		return PX4_ERROR;
	}

	PX4_INFO("PASS orb queuing (poll & notify), got %i messages", next_expected_val);
	return PX4_OK;
}

int uORBTest::UnitTest::pubsubtest_threadEntry(int argc, char *argv[])
{
	uORBTest::UnitTest &t = uORBTest::UnitTest::instance();
	return t.pubsublatency_main();
}
