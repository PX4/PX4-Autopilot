/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Simon Wilks <sjwilks@gmail.com>
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

#include "unit_test.h"

#include <systemlib/err.h>

UnitTest::UnitTest() :
	_tests_run(0),
	_tests_failed(0),
	_tests_passed(0),
	_assertions(0)
{
}

UnitTest::~UnitTest()
{
}

void UnitTest::print_results(void)
{
	warnx(_tests_failed ? "SOME TESTS FAILED" : "ALL TESTS PASSED");
	warnx("  Tests passed : %d", _tests_passed);
	warnx("  Tests failed : %d", _tests_failed);
	warnx("  Assertions : %d", _assertions);
}

/// @brief Used internally to the ut_assert macro to print assert failures.
void UnitTest::_print_assert(const char* msg, const char* test, const char* file, int line)
{
	warnx("Assertion failed: %s - %s (%s:%d)", msg, test, file, line);
}

/// @brief Used internally to the ut_compare macro to print assert failures.
void UnitTest::_print_compare(const char* msg, const char *v1_text, int v1, const char *v2_text, int v2, const char* file, int line)
{
	warnx("Compare failed: %s - (%s:%d) (%s:%d) (%s:%d)", msg, v1_text, v1, v2_text, v2, file, line);
}
