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
 * @file test_search_min.cpp
 * Tests arithmetic search algorithms.
 */

#include <unit_test.h>
#include <float.h>
#include <math.h>

#include <mathlib/math/SearchMin.hpp>

// linear function
float _linear_function(float x)
{
	float slope = 2.0f;
	return slope * x - 1.4f;

}

//linear function without slope
float _linear_function_flat(float x)
{
	return 1.4f;
}

// quadratic function with min at 2
float _quadratic_function(float x)
{
	return ((x - 2.0f) * (x - 2.0f) + 3.0f);
}

class SearchMinTest : public UnitTest
{
public:
	virtual bool run_tests();

private:
	bool _init_inputs();
	bool _init_inputs_flipped();
	bool _init_inputs_negative();
	bool _init_tol_larger_than_range();
	bool _init_tol_larger_than_range_flipped();
	bool _no_extremum();

};


bool SearchMinTest::run_tests()
{
	ut_run_test(_init_inputs);
	ut_run_test(_init_inputs_flipped);
	ut_run_test(_init_inputs_negative);
	ut_run_test(_init_tol_larger_than_range);
	ut_run_test(_init_tol_larger_than_range_flipped);
	ut_run_test(_no_extremum);

	return (_tests_failed == 0);
}

bool SearchMinTest::_init_inputs()
{
	float a = 1.0f;
	float b = 4.0f;
	float tol = 0.001f;
	float (*fun)(float);
	float (*fun2)(float);

	fun = &_linear_function;
	fun2 = &_quadratic_function;

	float opt = math::goldensection(a, b, fun, tol);
	float opt2 = math::goldensection(a, b, fun2, tol);
	ut_assert("linear function opt not equal min ", fabsf(opt - a) <= (tol * 2.0f));
	ut_assert("quad function opt not equal min ", fabsf(opt2 - 2.0f) <= (tol * 2.0f));

	return true;
}

bool SearchMinTest::_init_inputs_flipped()
{
	float a = 4.0f;
	float b = 1.0f;
	float tol = 0.001f;
	float (*fun)(float);
	float (*fun2)(float);

	fun = &_linear_function;
	fun2 = &_quadratic_function;

	float opt = math::goldensection(a, b, fun, tol);
	float opt2 = math::goldensection(a, b, fun2, tol);

	ut_assert("linear function opt not equal min", fabsf(opt - b) <= (tol * 2.0f));
	ut_assert("quad function opt not equal min ", fabsf(opt2 - 2.0f) <= (tol * 2.0f));

	return true;
}

bool SearchMinTest::_init_inputs_negative()
{
	float a = -4.0f;
	float b = -2.0f;
	float tol = 0.001f;
	float (*fun)(float);
	float (*fun2)(float);

	fun = &_linear_function;
	fun2 = &_quadratic_function;

	float opt = math::goldensection(a, b, fun, tol);
	float opt2 = math::goldensection(a, b, fun2, tol);

	ut_assert("linear function opt not equal min", fabsf(opt - a) <= (tol * 2.0f));
	ut_assert("quad function opt not equal min ", fabsf(opt2 - b) <= (tol * 2.0f));

	return true;
}

bool SearchMinTest::_init_tol_larger_than_range()
{
	float a = 1.0f;
	float b = 4.0f;
	float tol = 6.0f;
	float (*fun)(float);
	float (*fun2)(float);

	fun = &_linear_function;
	fun2 = &_quadratic_function;

	float opt = math::goldensection(a, b, fun, tol);
	float opt2 = math::goldensection(a, b, fun2, tol);

	ut_assert("linear function opt not equal min", fabsf(opt - (b + a) / 2.0f) <= (0.001f * 2.0f));
	ut_assert("quad function opt not equal min ", fabsf(opt2 - (b + a) / 2.0f) <= (0.001f * 2.0f));

	return true;
}

bool SearchMinTest::_init_tol_larger_than_range_flipped()
{
	float a = 4.0f;
	float b = 1.0f;
	float tol = 6.0f;
	float (*fun)(float);
	float (*fun2)(float);

	fun = &_linear_function;
	fun2 = &_quadratic_function;

	float opt = math::goldensection(a, b, fun, tol);
	float opt2 = math::goldensection(a, b, fun2, tol);

	ut_assert("linear function opt not equal min", fabsf(opt - (b + a) / 2.0f) <= (0.001f * 2.0f));
	ut_assert("quad function opt not equal min ", fabsf(opt2 - (b + a) / 2.0f) <= (0.001f * 2.0f));

	return true;
}

bool SearchMinTest::_no_extremum()
{
	float a = 1.f;
	float b = 4.0f;
	float tol = 0.001f;
	float (*fun)(float);
	fun = &_linear_function_flat;

	float opt = math::goldensection(a, b, fun, tol);
	ut_assert("linear function function opt not equal min", fabsf(fun(opt) - fun(b)) <= (tol));

	return true;
}

ut_declare_test_c(test_search_min, SearchMinTest)
