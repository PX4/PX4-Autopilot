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
 * @file uORBTest_Bench.cpp
 *
 * Micro-benchmarks for the uORB hot paths: `uorb_tests bench`.
 *
 * Each case runs a tight loop of one operation and reports nanoseconds per
 * operation. The loop total is timed with hrt_absolute_time(), so a case must
 * run long enough for its two clock reads to be negligible; the iteration
 * counts below give at least a few milliseconds per repetition on target.
 * The minimum over repetitions is the number least disturbed by interrupts
 * and other threads; the median shows how stable the run was.
 */

#include "uORBTest_Bench.hpp"

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/orb_test_large.h>
#include <uORB/topics/orb_test_medium.h>
#include <uORB/topics/uORBTopics.hpp>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#ifdef __PX4_NUTTX
#include <malloc.h>
#endif

namespace
{

constexpr unsigned REPETITIONS = 7;

struct Result {
	const char *name;
	uint64_t ns_per_op_min;
	uint64_t ns_per_op_median;
};

// Count-only callback: measures the publish-side dispatch, not a work queue.
class CountingCallback : public uORB::SubscriptionCallback
{
public:
	CountingCallback(const orb_metadata *meta, uint32_t interval_us) :
		SubscriptionCallback(meta, interval_us)
	{}

	void call(unsigned generation) override { _calls++; }

	unsigned calls() const { return _calls; }

private:
	unsigned _calls{0};
};

template <typename Op>
Result run(const char *name, unsigned iterations, Op op)
{
	uint64_t ns_per_op[REPETITIONS];

	if (iterations == 0) {
		return {name, 0, 0};
	}

	for (unsigned rep = 0; rep < REPETITIONS; rep++) {
		const hrt_abstime start = hrt_absolute_time();

		for (unsigned i = 0; i < iterations; i++) {
			op();
		}

		const hrt_abstime elapsed_us = hrt_elapsed_time(&start);
		ns_per_op[rep] = (elapsed_us * 1000) / iterations;
	}

	// insertion sort: REPETITIONS is tiny
	for (unsigned i = 1; i < REPETITIONS; i++) {
		const uint64_t v = ns_per_op[i];
		unsigned j = i;

		while (j > 0 && ns_per_op[j - 1] > v) {
			ns_per_op[j] = ns_per_op[j - 1];
			j--;
		}

		ns_per_op[j] = v;
	}

	return {name, ns_per_op[0], ns_per_op[REPETITIONS / 2]};
}

void print(const Result &r)
{
	PX4_INFO_RAW("  %-44s %8" PRIu64 " %8" PRIu64 "\n", r.name, r.ns_per_op_min, r.ns_per_op_median);
}

#ifdef __PX4_NUTTX
size_t heap_used() { return mallinfo().uordblks; }
#else
size_t heap_used() { return 0; }
#endif

} // namespace

int uORBTest::bench(bool populate)
{
	constexpr unsigned N_POLL = 200000;	// no-data checks
	constexpr unsigned N_PUBLISH = 20000;	// publish and copy loops
	constexpr unsigned N_LOOKUP = 20000;	// node lookups

	if (populate) {
		// Give every topic a node, as on a running system, so lookups walk a
		// realistic list. Subscribing creates the node without advertising it.
		const size_t heap_before = heap_used();
		unsigned nodes = 0;
		const orb_metadata *const *topics = orb_get_topics();

		for (size_t i = 0; i < orb_topics_count(); i++) {
			const orb_sub_t sub = orb_subscribe(topics[i]);

			if (orb_sub_valid(sub)) {
				nodes++;
				orb_unsubscribe(sub);
			}
		}

		PX4_INFO_RAW("populated: %u topic nodes, heap +%zu B\n", nodes, heap_used() - heap_before);
	}

	PX4_INFO_RAW("uORB bench: %u repetitions per case, ns/op\n", REPETITIONS);
	PX4_INFO_RAW("  %-44s %8s %8s\n", "case", "min", "median");

	orb_test_s msg{};
	msg.timestamp = hrt_absolute_time();
	uORB::Publication<orb_test_s> pub{ORB_ID(orb_test)};

	if (!pub.publish(msg)) {
		PX4_ERR("advertise failed");
		return -1;
	}

	// --- subscriber-side polling with no new data ---------------------------
	{
		uORB::Subscription sub{ORB_ID(orb_test)};
		sub.copy(&msg);
		volatile bool sink = false;
		print(run("Subscription::updated(), no data", N_POLL, [&] { sink = sub.updated(); }));
	}

	{
		uORB::SubscriptionInterval sub{ORB_ID(orb_test), 1000000};
		sub.copy(&msg);
		volatile bool sink = false;
		print(run("SubscriptionInterval::updated(), 1s, no data", N_POLL, [&] { sink = sub.updated(); }));
	}

	{
		uORB::SubscriptionInterval sub{ORB_ID(orb_test)};
		sub.copy(&msg);
		volatile bool sink = false;
		print(run("SubscriptionInterval::updated(), 0, no data", N_POLL, [&] { sink = sub.updated(); }));
	}

	// --- publish ------------------------------------------------------------
	unsigned publish_failures = 0;
	auto publish = [&] { publish_failures += pub.publish(msg) ? 0 : 1; };

	print(run("publish, no subscribers", N_PUBLISH, publish));

	{
		uORB::Subscription sub{ORB_ID(orb_test)};
		print(run("publish + Subscription::update()", N_PUBLISH, [&] {
			publish();
			sub.update(&msg);
		}));
	}

	{
		uORB::SubscriptionInterval sub{ORB_ID(orb_test)};
		print(run("publish + SubscriptionInterval::update(), 0", N_PUBLISH, [&] {
			publish();
			sub.update(&msg);
		}));
	}

	{
		CountingCallback cb[4] = {{ORB_ID(orb_test), 0}, {ORB_ID(orb_test), 0}, {ORB_ID(orb_test), 0}, {ORB_ID(orb_test), 0}};

		for (auto &c : cb) {
			c.registerCallback();
		}

		print(run("publish, 4 callbacks, interval 0", N_PUBLISH, publish));

		for (auto &c : cb) {
			c.set_interval_us(1000000);
		}

		print(run("publish, 4 callbacks, interval 1s", N_PUBLISH, publish));

		unsigned calls = 0;

		for (auto &c : cb) {
			calls += c.calls();
		}

		// every publish of both loops must have reached all four callbacks
		PX4_INFO_RAW("  %-44s %8u (expected %u)\n", "  callback invocations", calls, 2 * REPETITIONS * N_PUBLISH * 4);
	}

	{
		orb_test_medium_s medium{};
		uORB::Publication<orb_test_medium_s> pub_medium{ORB_ID(orb_test_medium)};
		pub_medium.publish(medium);
		uORB::Subscription sub{ORB_ID(orb_test_medium)};
		print(run("publish + update(), queued topic", N_PUBLISH, [&] {
			pub_medium.publish(medium);
			sub.update(&medium);
		}));
	}

	// --- advertise and lookup -----------------------------------------------
	{
		// Instances persist, so only the first run pays for node creation.
		px4_usleep(50000);	// let deferred frees from earlier commands (task stacks) settle first
		const size_t heap_before = heap_used();
		const hrt_abstime start = hrt_absolute_time();
		orb_advert_t handles[ORB_MULTI_MAX_INSTANCES] {};
		unsigned advertised = 0;

		for (auto &handle : handles) {
			int instance = 0;
			handle = orb_advertise_multi(ORB_ID(orb_test_large), nullptr, &instance);
			advertised += (handle != nullptr) ? 1 : 0;
		}

		const uint64_t elapsed_us = hrt_elapsed_time(&start);
		PX4_INFO_RAW("  %-44s %8" PRIu64 " us, heap +%zu B\n", "advertise_multi x instances", elapsed_us,
			     heap_used() - heap_before);
		PX4_INFO_RAW("  %-44s %8u\n", "  instances advertised", advertised);

		// Release the instances so a repeat run re-claims the existing nodes.
		for (auto &handle : handles) {
			if (handle != nullptr) {
				orb_unadvertise(handle);
			}
		}

		volatile int sink = 0;
		const int last = ORB_MULTI_MAX_INSTANCES - 1;
		print(run("orb_exists(), last instance", N_LOOKUP, [&] { sink = orb_exists(ORB_ID(orb_test_large), last); }));

		print(run("Subscription subscribe/unsubscribe", N_LOOKUP, [&] {
			uORB::Subscription sub{ORB_ID(orb_test_large), static_cast<uint8_t>(last)};
			sub.unsubscribe();
		}));
	}

	PX4_INFO_RAW("  %-44s %8u\n", "publish failures", publish_failures);

	return publish_failures == 0 ? 0 : -1;
}
