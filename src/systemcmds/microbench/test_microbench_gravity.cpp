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
 * @file test_microbench_gravity.cpp
 * Benchmark for LatLonAlt::Wgs84::gravity() — the Somigliana latitude-dependent
 * gravity helper used by EKF2 and SIH. Measures the one-time cost of the
 * sin + sqrt computation on the target MCU.
 */

#include <unit_test.h>

#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/px4_config.h>

#include <lib/lat_lon_alt/lat_lon_alt.hpp>

namespace MicroBenchGravity
{

#ifdef __PX4_NUTTX
#include <nuttx/irq.h>
static irqstate_t flags;
#endif

void lock()
{
#ifdef __PX4_NUTTX
	flags = px4_enter_critical_section();
#endif
}

void unlock()
{
#ifdef __PX4_NUTTX
	px4_leave_critical_section(flags);
#endif
}

#define PERF(name, op, count) do { \
		reset(); \
		perf_counter_t p = perf_alloc(PC_ELAPSED, name); \
		for (int rep = 0; rep < 10; rep++) { \
			px4_usleep(1000); \
			lock(); \
			perf_begin(p); \
			for (int i = 0; i < (count)/10; i++) { \
				op; \
				op; \
				op; \
				op; \
				op; \
				op; \
				op; \
				op; \
				op; \
				op; \
			} \
			perf_end(p); \
			unlock(); \
			reset(); \
		} \
		perf_print_counter(p); \
		perf_free(p); \
	} while (0)

class MicroBenchGravity : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool time_gravity();
	void reset();

	// volatile so the compiler cannot eliminate the call as dead code
	volatile double lat_rad;
	volatile float  gravity_out;
};

void MicroBenchGravity::reset()
{
	srand(time(nullptr));
	// random latitude in [-pi/2, pi/2] — representative of real flight locations
	lat_rad = ((double)rand() / RAND_MAX - 0.5) * M_PI;
	gravity_out = 0.f;
}

bool MicroBenchGravity::time_gravity()
{
	// 100 calls: the function runs at most once per ~111 km of latitude travel,
	// so even 1 call cost is the relevant metric; 100 gives a stable average.
	PERF("gravity() 100 ops", gravity_out = LatLonAlt::Wgs84::gravity(lat_rad), 100);
	return true;
}

bool MicroBenchGravity::run_tests()
{
	ut_run_test(time_gravity);
	return (_tests_failed == 0);
}

ut_declare_test_c(test_microbench_gravity, MicroBenchGravity)

} // namespace MicroBenchGravity
