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
 * @file uORBTest_Flight.cpp
 *
 * A synthetic sensor-to-actuator pipeline at flight rates on the real work
 * queues: `uorb_tests flight [seconds]`.
 *
 *   source (2 kHz, 80 B)  -> rate filter (rate_ctrl, 3 idle polls, republish)
 *                         -> rate control (rate_ctrl, 3 idle polls, 16 B actuator)
 *                         -> 6 consumers (nav_and_controllers / hp_default, 2 idle polls)
 *   actuator              -> 400 Hz poller
 *
 * Every message carries the source timestamp, so each stage measures the
 * latency from the sensor sample to its own run. Missed samples are counted
 * from gaps in the sequence number. CPU load is the cpuload topic averaged
 * before and during the run. Only test topics are used, so it can run next
 * to the real drivers on a board.
 */

#include "uORBTest_Bench.hpp"

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/orb_test_medium.h>

#include <drivers/drv_hrt.h>
#include <inttypes.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

namespace
{

constexpr unsigned SOURCE_INTERVAL_US = 500;	// 2 kHz, a gyro
constexpr unsigned POLLER_INTERVAL_US = 2500;	// 400 Hz, a PWM output
constexpr unsigned FAN_OUT = 6;			// consumers of the filtered rate, like vehicle_angular_velocity
constexpr unsigned IDLE_TOPICS = 3;		// advertised once, never updated: the parameter_update pattern
constexpr uint32_t POLL_INTERVAL_US = 1000000;
constexpr unsigned HIST_BUCKETS = 256;		// 1 us buckets below HIST_FINE_US, 8 us above; last collects the rest
constexpr unsigned HIST_FINE_US = 128;
constexpr unsigned HIST_COARSE_US = 8;
constexpr unsigned DRAIN_MS = 50;
constexpr unsigned WARMUP_MS = 500;		// work queue threads and nodes come up before recording starts

// Set by flight() once the pipeline has warmed up; the stages only record while it is set.
px4::atomic<bool> g_recording{false};

// Written only by the owning work item's thread; read after the pipeline stopped.
struct LatencyStats {
	uint32_t count{0};
	uint32_t missed{0};
	uint64_t sum_us{0};
	uint32_t max_us{0};
	uint32_t hist[HIST_BUCKETS] {};
	int32_t last_seq{-1};

	void add(hrt_abstime sample_time, int32_t seq)
	{
		const uint32_t lat = hrt_elapsed_time(&sample_time);
		count++;
		sum_us += lat;
		max_us = (lat > max_us) ? lat : max_us;
		unsigned bucket = (lat < HIST_FINE_US) ? lat : HIST_FINE_US + (lat - HIST_FINE_US) / HIST_COARSE_US;
		hist[(bucket < HIST_BUCKETS) ? bucket : HIST_BUCKETS - 1]++;

		if (last_seq >= 0 && seq > last_seq + 1) {
			missed += seq - last_seq - 1;
		}

		last_seq = seq;
	}

	uint32_t percentile(unsigned pct) const
	{
		const uint64_t target = ((uint64_t)count * pct + 99) / 100;
		uint64_t seen = 0;

		for (unsigned i = 0; i < HIST_BUCKETS; i++) {
			seen += hist[i];

			if (seen >= target) {
				return (i < HIST_FINE_US) ? i : HIST_FINE_US + (i - HIST_FINE_US) * HIST_COARSE_US;
			}
		}

		return HIST_FINE_US + (HIST_BUCKETS - HIST_FINE_US) * HIST_COARSE_US;
	}

	void print(const char *name, unsigned seconds, bool show_missed = true) const
	{
		if (count == 0) {
			PX4_INFO_RAW("  %-20s %8s\n", name, "no data");
			return;
		}

		char missed_str[8];
		snprintf(missed_str, sizeof(missed_str), "%" PRIu32, missed);
		PX4_INFO_RAW("  %-20s %8" PRIu32 " %6s %6" PRIu32 " %6" PRIu64 " %6" PRIu32 " %6" PRIu32 "\n", name, count,
			     show_missed ? missed_str : "-",
			     count / seconds, sum_us / count, percentile(99), max_us);
	}
};

// orb_test instances that are advertised and published once, then only polled.
class IdleTopics
{
public:
	IdleTopics()
	{
		for (unsigned i = 0; i < IDLE_TOPICS; i++) {
			orb_test_s msg{};
			msg.timestamp = hrt_absolute_time();
			int instance = -1;
			_handles[i] = orb_advertise_multi(ORB_ID(orb_test), &msg, &instance);
			_ok = _ok && (_handles[i] != nullptr);
			_instances[i] = (_handles[i] != nullptr) ? static_cast<uint8_t>(instance) : 0;
		}
	}

	~IdleTopics()
	{
		for (auto &handle : _handles) {
			if (handle != nullptr) {
				orb_unadvertise(handle);
			}
		}
	}

	bool ok() const { return _ok; }

	uint8_t instance(unsigned i) const { return _instances[i % IDLE_TOPICS]; }

private:
	orb_advert_t _handles[IDLE_TOPICS] {};
	uint8_t _instances[IDLE_TOPICS] {};
	bool _ok{true};
};

// What a module does with its parameter_update-style subscriptions each iteration.
class IdlePolls
{
public:
	IdlePolls(const IdleTopics &topics, unsigned n) : _n((n < IDLE_TOPICS) ? n : IDLE_TOPICS)
	{
		for (unsigned i = 0; i < _n; i++) {
			_subs[i] = uORB::SubscriptionInterval{ORB_ID(orb_test), POLL_INTERVAL_US, topics.instance(i)};
			orb_test_s msg;
			_subs[i].copy(&msg);	// consume the initial sample, as a module does at start-up
		}
	}

	void poll()
	{
		for (unsigned i = 0; i < _n; i++) {
			if (_subs[i].updated()) {
				_spurious++;
			}
		}
	}

	unsigned spurious() const { return _spurious; }

private:
	uORB::SubscriptionInterval _subs[IDLE_TOPICS];
	const unsigned _n;
	unsigned _spurious{0};
};

class Source : public px4::ScheduledWorkItem
{
public:
	Source() : ScheduledWorkItem("uorb_flight_source", px4::wq_configurations::test1) {}
	~Source() override { ScheduleClear(); }

	bool advertise() { return _pub.advertise(); }
	void start() { ScheduleOnInterval(SOURCE_INTERVAL_US); }
	void stop() { ScheduleClear(); }
	uint8_t instance() { return _pub.get_instance(); }
	uint32_t published() const { return _published.load(); }

private:
	void Run() override
	{
		orb_test_medium_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.val = _seq++;
		_pub.publish(msg);

		if (g_recording.load()) {
			_published.fetch_add(1);
		}
	}

	uORB::PublicationMulti<orb_test_medium_s> _pub{ORB_ID(orb_test_medium)};
	px4::atomic<uint32_t> _published{0};
	int32_t _seq{0};
};

// Callback-driven stage: drains its input, does its idle polls and, when it has
// an output topic, republishes with the source timestamp.
template <typename In, typename Out>
class Stage : public px4::WorkItem
{
public:
	Stage(const char *name, const px4::wq_config_t &wq, const orb_metadata *in_meta, uint8_t in_instance,
	      ORB_ID out_id, const IdleTopics &idle, unsigned polls) :
		WorkItem(name, wq),
		_name(name),
		_sub(this, in_meta, in_instance),
		_out_id(out_id),
		_pub(out_id),
		_polls(idle, polls)
	{
	}

	~Stage() override { _sub.unregisterCallback(); }

	// Call only after stop(): the acquire on the run counter orders every write
	// the work queue thread made in Run() before the caller's reads.
	const LatencyStats &stats()
	{
		(void)_runs_done.load();
		return _stats;
	}

	bool start()
	{
		if (publishes() && !_pub.advertise()) {
			return false;
		}

		return _sub.registerCallback();
	}

	void stop() { _sub.unregisterCallback(); }

	uint8_t out_instance() { return _pub.get_instance(); }
	const char *name() const { return _name; }
	unsigned spurious_polls() const { return _polls.spurious(); }

private:
	bool publishes() const { return _out_id != ORB_ID::INVALID; }

	void Run() override
	{
		In in;

		while (_sub.update(&in)) {
			if (g_recording.load()) {
				_stats.add(in.timestamp, in.val);
			}

			_polls.poll();

			if (publishes()) {
				Out out{};
				out.timestamp = in.timestamp;
				out.val = in.val;
				_pub.publish(out);
			}
		}

		_runs_done.fetch_add(1);
	}

	const char *_name;
	uORB::SubscriptionCallbackWorkItem _sub;
	const ORB_ID _out_id;
	uORB::PublicationMulti<Out> _pub;
	IdlePolls _polls;
	LatencyStats _stats;
	px4::atomic<uint32_t> _runs_done{0};
};

class Poller : public px4::ScheduledWorkItem
{
public:
	Poller(const orb_metadata *meta, uint8_t instance) :
		ScheduledWorkItem("uorb_flight_poller", px4::wq_configurations::hp_default),
		_sub(meta, instance)
	{
	}

	~Poller() override { ScheduleClear(); }

	void start()
	{
		orb_test_s msg;
		_sub.copy(&msg);	// drop whatever a previous run left in the node
		ScheduleOnInterval(POLLER_INTERVAL_US);
	}

	void stop() { ScheduleClear(); }

	// Call only after stop(); see Stage::stats().
	const LatencyStats &stats()
	{
		(void)_runs_done.load();
		return _stats;
	}

private:
	void Run() override
	{
		orb_test_s msg;

		if (_sub.update(&msg) && g_recording.load()) {
			_stats.add(msg.timestamp, msg.val);
		}

		_runs_done.fetch_add(1);
	}

	uORB::Subscription _sub;
	LatencyStats _stats;
	px4::atomic<uint32_t> _runs_done{0};
};

// Average cpuload.load over a window, or -1 when load_mon is not publishing.
float average_load(unsigned window_ms)
{
	uORB::Subscription sub{ORB_ID(cpuload)};
	cpuload_s msg{};
	sub.copy(&msg);
	float sum = 0.f;
	unsigned n = 0;

	for (unsigned elapsed = 0; elapsed < window_ms; elapsed += 100) {
		px4_usleep(100000);

		if (sub.update(&msg)) {
			sum += msg.load;
			n++;
		}
	}

	return (n > 0) ? sum / n : -1.f;
}

void print_load(const char *label, float load)
{
	if (load < 0.f) {
		PX4_INFO_RAW("  %-20s %8s\n", label, "n/a");

	} else {
		PX4_INFO_RAW("  %-20s %7.1f%%\n", label, (double)(load * 100.f));
	}
}

} // namespace

int uORBTest::flight(unsigned seconds)
{
	using RateStage = Stage<orb_test_medium_s, orb_test_medium_s>;
	using ControlStage = Stage<orb_test_medium_s, orb_test_s>;
	using Consumer = Stage<orb_test_medium_s, orb_test_s>;

	static const char *const consumer_names[FAN_OUT] = {"consumer 0", "consumer 1", "consumer 2", "consumer 3", "consumer 4", "consumer 5"};

	PX4_INFO_RAW("uORB flight pipeline: %u s, source %u Hz, %u consumers, poller %u Hz\n", seconds,
		     1000000 / SOURCE_INTERVAL_US, FAN_OUT, 1000000 / POLLER_INTERVAL_US);

	const float load_before = average_load(2000);

	// Everything lives on the heap: the latency histograms do not fit an nsh stack.
	IdleTopics *idle = new IdleTopics();
	Source *source = new Source();
	RateStage *rate = nullptr;
	ControlStage *control = nullptr;
	Consumer *consumers[FAN_OUT] {};
	Poller *poller = nullptr;
	int ret = -1;

	// Each stage starts before the next one is wired to its output instance.
	bool ok = (idle != nullptr) && idle->ok() && (source != nullptr) && source->advertise();

	if (!ok) {
		PX4_ERR("source: advertise failed");
	}

	if (ok) {
		rate = new RateStage("rate filter", px4::wq_configurations::rate_ctrl, ORB_ID(orb_test_medium), source->instance(),
				     ORB_ID::orb_test_medium, *idle, 3);
		ok = (rate != nullptr) && rate->start();

		if (!ok) {
			PX4_ERR("rate filter: start failed");
		}
	}

	if (ok) {
		control = new ControlStage("rate control", px4::wq_configurations::rate_ctrl, ORB_ID(orb_test_medium),
					   rate->out_instance(), ORB_ID::orb_test, *idle, 3);
		ok = (control != nullptr) && control->start();

		if (!ok) {
			PX4_ERR("rate control: start failed");
		}
	}

	if (ok) {
		for (unsigned i = 0; i < FAN_OUT; i++) {
			const px4::wq_config_t &wq = (i % 2 == 0) ? px4::wq_configurations::nav_and_controllers :
						     px4::wq_configurations::hp_default;
			consumers[i] = new Consumer(consumer_names[i], wq, ORB_ID(orb_test_medium), rate->out_instance(), ORB_ID::INVALID,
						    *idle, 2);
			ok = ok && (consumers[i] != nullptr) && consumers[i]->start();
		}

		poller = new Poller(ORB_ID(orb_test), control->out_instance());
		ok = ok && (poller != nullptr);

		if (!ok) {
			PX4_ERR("consumers: start failed");
		}
	}

	if (ok) {
		poller->start();
		source->start();
		px4_usleep(WARMUP_MS * 1000);

		g_recording.store(true);
		const float load_during = average_load(seconds * 1000);
		g_recording.store(false);

		source->stop();
		px4_usleep(DRAIN_MS * 1000);
		rate->stop();
		control->stop();

		for (auto *c : consumers) {
			c->stop();
		}

		poller->stop();
		px4_usleep(DRAIN_MS * 1000);

		// stats() first: its acquire orders the work queue threads' writes (including the poll counters).
		rate->stats();
		control->stats();

		for (auto *c : consumers) {
			c->stats();
		}

		unsigned spurious = rate->spurious_polls() + control->spurious_polls();

		for (auto *c : consumers) {
			spurious += c->spurious_polls();
		}

		PX4_INFO_RAW("  %-20s %8s %6s %6s %6s %6s %6s\n", "stage", "samples", "missed", "Hz", "mean", "p99", "max");
		PX4_INFO_RAW("  %-20s %8" PRIu32 " %6s %6" PRIu32 "\n", "source", source->published(), "-",
			     source->published() / seconds);
		rate->stats().print(rate->name(), seconds);
		control->stats().print(control->name(), seconds);

		for (auto *c : consumers) {
			c->stats().print(c->name(), seconds);
		}

		poller->stats().print("poller (sample age)", seconds, false);
		PX4_INFO_RAW("  %-20s %8u\n", "spurious idle polls", spurious);
		print_load("cpu load before", load_before);
		print_load("cpu load during", load_during);

		if (load_before >= 0.f && load_during >= 0.f) {
			PX4_INFO_RAW("  %-20s %+7.1f%%\n", "cpu load delta", (double)((load_during - load_before) * 100.f));
		}

		ret = (spurious == 0) ? 0 : -1;
	}

	// Stop everything before anything is deleted: no stage may run or stay registered while a peer goes away.
	if (source != nullptr) { source->stop(); }

	for (auto *c : consumers) {
		if (c != nullptr) { c->stop(); }
	}

	if (rate != nullptr) { rate->stop(); }

	if (control != nullptr) { control->stop(); }

	if (poller != nullptr) { poller->stop(); }

	delete poller;

	for (auto *c : consumers) {
		delete c;
	}

	delete control;
	delete rate;
	delete source;
	delete idle;

	return ret;
}
