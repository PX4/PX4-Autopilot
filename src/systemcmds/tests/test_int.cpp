/****************************************************************************
 *
 *  Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file test_int.cpp
 * Tests for integer types.
 */

#include <unit_test.h>

#include <errno.h>
#include <fcntl.h>
#include <float.h>
#include <math.h>
#include <px4_platform_common/config.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

typedef union {
	int32_t i;
	int64_t l;
	uint8_t b[8];
} test_32_64_t;

class IntTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool math64bitTests();
	bool math3264MixedMathTests();
};

bool IntTest::math64bitTests()
{
	int64_t large = 354156329598;
	int64_t calc = large * 5;

	ut_assert("354156329598 * 5 == 1770781647990", calc == 1770781647990);

	return true;
}

bool IntTest::math3264MixedMathTests()
{
	int32_t small = 50;
	int32_t large_int = 2147483647; // MAX INT value

	uint64_t small_times_large = large_int * (uint64_t)small;

	ut_assert("64bit calculation: 50 * 2147483647 (max int val) == 107374182350", small_times_large == 107374182350);

	return true;
}


bool IntTest::run_tests()
{
	ut_run_test(math64bitTests);
	ut_run_test(math3264MixedMathTests);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_int, IntTest)
