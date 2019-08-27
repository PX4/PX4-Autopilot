/****************************************************************************
 *
 *  Copyright (C) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file test_microbench_matrix.cpp
 * Tests for the microbench matrix math library.
 */

#include <unit_test.h>

#include <time.h>
#include <stdlib.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/config.h>
#include <px4_platform_common/micro_hal.h>

#include <matrix/math.hpp>

namespace MicroBenchMatrix
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
		px4_usleep(1000); \
		reset(); \
		perf_counter_t p = perf_alloc(PC_ELAPSED, name); \
		for (int i = 0; i < count; i++) { \
			lock(); \
			perf_begin(p); \
			op; \
			perf_end(p); \
			unlock(); \
			reset(); \
		} \
		perf_print_counter(p); \
		perf_free(p); \
	} while (0)

class MicroBenchMatrix : public UnitTest
{
public:
	virtual bool run_tests();

private:

	bool time_px4_matrix();

	void reset();

	matrix::Quatf q;
	matrix::Eulerf e;
	matrix::Dcmf d;

};

bool MicroBenchMatrix::run_tests()
{
	ut_run_test(time_px4_matrix);

	return (_tests_failed == 0);
}

template<typename T>
T random(T min, T max)
{
	const T scale = rand() / (T) RAND_MAX; /* [0, 1.0] */
	return min + scale * (max - min);      /* [min, max] */
}

void MicroBenchMatrix::reset()
{
	srand(time(nullptr));

	// initialize with random data
	q = matrix::Quatf(rand(), rand(), rand(), rand());
	e = matrix::Eulerf(random(-2.0 * M_PI, 2.0 * M_PI), random(-2.0 * M_PI, 2.0 * M_PI), random(-2.0 * M_PI, 2.0 * M_PI));
	d = q;
}

ut_declare_test_c(test_microbench_matrix, MicroBenchMatrix)

bool MicroBenchMatrix::time_px4_matrix()
{
	PERF("matrix Euler from Quaternion", e = q, 1000);
	PERF("matrix Euler from Dcm", e = d, 1000);

	PERF("matrix Quaternion from Euler", q = e, 1000);
	PERF("matrix Quaternion from Dcm", q = d, 1000);

	PERF("matrix Dcm from Euler", d = e, 1000);
	PERF("matrix Dcm from Quaternion", d = q, 1000);

	return true;
}

} // namespace MicroBenchMatrix
