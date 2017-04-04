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

#ifndef UNIT_TEST_H_
#define UNIT_TEST_H_

#include <px4_log.h>

#define ut_declare_test_c(test_function, test_class)	\
	extern "C" {										\
		int test_function(int argc, char *argv[])		\
		{												\
			test_class* test = new test_class();		\
			bool success = test->run_tests();			\
			test->print_results();						\
			return success ? 0 : -1;					\
		}												\
	}

/// @brief Base class to be used for unit tests.
class __EXPORT UnitTest
{
public:

	UnitTest();
	virtual ~UnitTest();

	/// @brief Override to run your unit tests. Unit tests should be called using ut_run_test macro.
	/// @return true: all unit tests succeeded, false: one or more unit tests failed
	virtual bool run_tests(void) = 0;

	/// @brief Prints results from running of unit tests.
	void print_results(void);

/// @brief Macro to create a function which will run a unit test class and print results.
#define ut_declare_test(test_function, test_class)	\
	bool test_function(void)			\
	{						\
		test_class* test = new test_class();	\
		bool success = test->run_tests();	\
		test->print_results();			\
		delete test;				\
		return success;				\
	}

protected:

/// @brief Runs a single unit test. Unit tests must have the function signature of bool test(void). The unit
/// test should return true if it succeeded, false for fail.
#define ut_run_test(test)					\
	do {							\
		PX4_INFO("RUNNING TEST: %s", #test);		\
		_tests_run++;					\
		_init();						\
		if (!test()) {					\
			PX4_ERR("TEST FAILED: %s", #test);	\
			_tests_failed++;			\
		} else {					\
			PX4_INFO("TEST PASSED: %s", #test);	\
			_tests_passed++;			\
		}						\
		_cleanup();					\
	} while (0)

/// @brief Used to assert a value within a unit test.
#define ut_assert(message, test)						\
	do {									\
		if (!(test)) {							\
			_print_assert(message, #test, __FILE__, __LINE__);	\
			return false;						\
		} else {							\
			_assertions++;						\
		}								\
	} while (0)

/// @brief Used to assert a value within a unit test.
#define ut_test(test) ut_assert("test", test)

/// @brief To assert specifically to true.
#define ut_assert_true(test)						\
	do {									\
		if ((test) != true) {							\
			_print_assert("result not true", #test, __FILE__, __LINE__);	\
			return false;						\
		} else {							\
			_assertions++;						\
		}								\
	} while (0)

/// @brief To assert specifically to true.
#define ut_assert_false(test)						\
	do {									\
		if ((test) != false) {							\
			_print_assert("result not false", #test, __FILE__, __LINE__);	\
			return false;						\
		} else {							\
			_assertions++;						\
		}								\
	} while (0)

/// @brief Used to compare two integer values within a unit test. If possible use ut_compare instead of ut_assert
/// since it will give you better error reporting of the actual values being compared.
#define ut_compare(message, v1, v2)								\
	do {											\
		int _v1 = v1;									\
		int _v2 = v2;									\
		if (_v1 != _v2) {								\
			_print_compare(message, #v1, _v1, #v2, _v2, __FILE__, __LINE__);	\
			return false;								\
		} else {									\
			_assertions++;								\
		}										\
	} while (0)

/// @brief Used to compare two float values within a unit test. If possible use ut_compare_float instead of ut_assert
/// since it will give you better error reporting of the actual values being compared.
#define ut_compare_float(message, v1, v2, precision)						\
	do {											\
		int _p = pow(10.0f, precision);							\
		int _v1 = (int)(v1 * _p + 0.5f);						\
		int _v2 = (int)(v2 * _p + 0.5f);						\
		if (_v1 != _v2) {								\
			_print_compare(message, #v1, _v1, #v2, _v2, __FILE__, __LINE__);	\
			return false;								\
		} else {									\
			_assertions++;								\
		}										\
	} while (0)

/// @brief Used to compare two integer values within a unit test. If possible use ut_less_than instead of ut_assert
/// since it will give you better error reporting of the actual values being compared.
#define ut_less_than(message, v1_smaller, v2_bigger)								\
	do {											\
		int _v1 = v1_smaller;							\
		int _v2 = v2_bigger;							\
		if (!(_v1 < _v2)) {								\
			_print_compare(message, #v1_smaller, _v1, #v2_bigger, _v2, __FILE__, __LINE__);	\
			return false;								\
		} else {									\
			_assertions++;								\
		}										\
	} while (0)

	virtual void _init(void) { };		///< Run before each unit test. Override to provide custom behavior.
	virtual void _cleanup(void) { };	///< Run after each unit test. Override to provide custom behavior.

	void _print_assert(const char *msg, const char *test, const char *file, int line);
	void _print_compare(const char *msg, const char *v1_text, int v1, const char *v2_text, int v2, const char *file,
			    int line);

	int _tests_run;		///< The number of individual unit tests run
	int _tests_failed;	///< The number of unit tests which failed
	int _tests_passed;	///< The number of unit tests which passed
	int _assertions;	///< Total number of assertions tested by all unit tests
};

#endif /* UNIT_TEST_H_ */
