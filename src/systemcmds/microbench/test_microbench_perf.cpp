/****************************************************************************
 *
 *  Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * @file test_microbench_perf.cpp
 * Microbenchmark perf_counter hot-path operations (perf_count / perf_begin /
 * perf_end). Used to quantify the overhead of making the counters atomic.
 *
 * Each operation is run in a tight loop and timed once with hrt_absolute_time(),
 * then reported as ns per call. A plain (non-atomic) volatile counter loop is
 * included as a reference baseline so the atomic overhead is visible within a
 * single binary, without needing a separate non-atomic build.
 */

#include <unit_test.h>

#include <stdint.h>

#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

namespace MicroBenchPerf
{

// Number of iterations per measurement. Large enough that the loop runs for
// several hundred microseconds even on fast hosts, so the hrt (1 us) timer
// resolution does not dominate the per-op result.
static constexpr unsigned ITERATIONS = 1000000;

// Report total elapsed and ns/op for a measured loop.
#define REPORT(name, elapsed_us) \
	PX4_INFO("%-28s %6.1f ns/op  (%llu us / %u iters)", name, \
		 (double)(elapsed_us) * 1000.0 / (double)ITERATIONS, \
		 (unsigned long long)(elapsed_us), ITERATIONS)

class MicroBenchPerf : public UnitTest
{
public:
	bool run_tests() override;

private:
	bool time_reference_nonatomic();
	bool time_perf_count();
	bool time_perf_begin_end();
};

// Plain non-atomic increment of a volatile counter: the baseline the atomic
// perf_count() is compared against. volatile prevents the loop being elided.
bool MicroBenchPerf::time_reference_nonatomic()
{
	static volatile uint64_t counter = 0;

	const hrt_abstime t0 = hrt_absolute_time();

	for (unsigned i = 0; i < ITERATIONS; i++) {
		counter = counter + 1;
	}

	const hrt_abstime elapsed = hrt_absolute_time() - t0;
	REPORT("reference non-atomic ++", elapsed);
	return true;
}

bool MicroBenchPerf::time_perf_count()
{
	perf_counter_t c = perf_alloc(PC_COUNT, "microbench_perf_count");

	const hrt_abstime t0 = hrt_absolute_time();

	for (unsigned i = 0; i < ITERATIONS; i++) {
		perf_count(c);
	}

	const hrt_abstime elapsed = hrt_absolute_time() - t0;
	REPORT("perf_count (PC_COUNT)", elapsed);
	perf_free(c);
	return true;
}

bool MicroBenchPerf::time_perf_begin_end()
{
	perf_counter_t c = perf_alloc(PC_ELAPSED, "microbench_perf_elapsed");

	const hrt_abstime t0 = hrt_absolute_time();

	for (unsigned i = 0; i < ITERATIONS; i++) {
		perf_begin(c);
		perf_end(c);
	}

	const hrt_abstime elapsed = hrt_absolute_time() - t0;
	REPORT("perf_begin + perf_end", elapsed);
	perf_free(c);
	return true;
}

bool MicroBenchPerf::run_tests()
{
	ut_run_test(time_reference_nonatomic);
	ut_run_test(time_perf_count);
	ut_run_test(time_perf_begin_end);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_microbench_perf, MicroBenchPerf)

} // namespace MicroBenchPerf
