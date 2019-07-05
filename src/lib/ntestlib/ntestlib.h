/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ntestlib.h
 *
 * This is a minimal Unit testing library which mimicks some of the gtest
 * macros as well as the test harness output to be used with NuttX where
 * the C++ STL is not available.
 *
 * @note The nomenclature is the same as for gtest, each test function is
 *       called a test while multiple tests together in one file are
 *       called a test case:
 *       TEST(CaseName, TestName)
 *
 * Most of this library is based on Beat Küngs work in
 * https://github.com/Auterion/mavlink-testing-suite/
 *
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <px4_posix.h>
#include <cstdio>
#include <containers/List.hpp>


/**
 * @class TestBase
 * Base class for each test.
 */
class TestBase
{
public:
	enum class Result { Success = 0, Failed, Timeout, NotImplemented };

	TestBase() = default;
	virtual ~TestBase() = default;

	Result run();

	Result getLastResult() { return _result; }
	static const char *str(const Result result);

	template <typename T>
	void checkEq(const T &a, const T &b, const char *a_str, const char *b_str,
		     const char *file, int line, bool abort_on_failure)
	{
		if (a == b) {
			return;
		}

		printEq(a, b, a_str, b_str, file, line);

		_result = Result::Failed;

		if (abort_on_failure) {
			abort_test();
		}
	}

	template <typename T>
	void checkBool(const T &actual, const char *actual_str, bool expected,
		       const char *file, int line, bool abort_on_failure)
	{
		if (actual == expected) {
			return;
		}

		printBool(actual, actual_str, expected, file, line);

		_result = Result::Failed;

		if (abort_on_failure) {
			abort_test();
		}
	}


#define EXPECT_EQ(a_, b_) checkEq((a_), (b_), #a_, #b_, __FILE__, __LINE__, false)
#define ASSERT_EQ(a_, b_) checkEq((a_), (b_), #a_, #b_, __FILE__, __LINE__, true)

#define EXPECT_TRUE(expr_) checkBool((expr_), #expr_, true, __FILE__, __LINE__, false);
#define ASSERT_TRUE(expr_) checkBool((expr_), #expr_, true, __FILE__, __LINE__, true);

#define EXPECT_FALSE(expr_) checkBool((expr_), #expr_, false, __FILE__, __LINE__, false);
#define ASSERT_FALSE(expr_) checkBool((expr_), #expr_, false, __FILE__, __LINE__, true);

protected:
	virtual void run_testbody() = 0;

private:
	static void *run_trampoline(void *self);
	const char *extractFilename(const char *path);
	void abort_test();

	void print_any(bool b) { printf("%s", (b ? "true" : "false")); }
	void print_any(int i) { printf("%i", i); }
	void print_any(long int i) { printf("%li", i); }
	void print_any(unsigned u) { printf("%u", u); }
	void print_any(long unsigned u) { printf("%lu", u); }
	void print_any(float f) { printf("%f", (double)f); }
	void print_any(double d) { printf("%f", d); }
	void print_any(...)
	{
		printf("<Unknown type>\n");
	}

	template <typename T>
	void printEq(const T &a, const T &b, const char *a_str, const char *b_str,
		     const char *file, int line)
	{
		printf("%s:%d Failure\n", file, line);
		printf("Expected equality of these values:\n");
		printf("  %s\n", a_str);
		printf("    Which is: ");
		print_any(a);
		printf("\n");
		printf("  %s\n", b_str);
		printf("    Which is: ");
		print_any(b);
		printf("\n");
	}

	template <typename T>
	void printBool(const T &actual, const char *actual_str, bool expected,
		       const char *file, int line)
	{
		printf("%s:%d Failure\n", file, line);
		printf("Value of: %s\n", actual_str);
		printf("  Actual: ");
		print_any(actual);
		printf("\n");
		printf("Expected: ");
		print_any(expected);
		printf("\n");
	}

	Result _result{Result::Success};
};

/**
 * Base class for TestRegistrar
 * See TestRegistrar below for explanations
 */
class ITestRegistrar
{
public:
	virtual TestBase *instantiateTest() = 0;
};

/**
 * This is the factory, the common interface to "tests".
 * Tests registers themselves here and the factory can serve them on demand.
 * It is a Singleton.
 */
class TestFactory
{
public:
	static TestFactory &instance();

	TestFactory(TestFactory const &) = delete;
	void operator=(TestFactory const &) = delete;

	struct Test : public ListNode<Test *> {
		TestBase *testbase;
		const char *testname;
	};

	struct Case : public ListNode<Case *> {
		List<Test *> tests;
		const char *casename;
	};

	List<Case *> &getAllCases();
	unsigned getNumTests();
	unsigned getNumCases();

	void registerTest(ITestRegistrar *registrar,
			  const char *casename, const char *testname);

	Case *findCase(const char *casename);
private:
	TestFactory() = default;

	List<Case *> _cases;
};

/**
 * Helper class that registers a test upon construction.
 */
template <class TTest>
class TestRegistrar : public ITestRegistrar
{
public:
	explicit TestRegistrar(const char *casename, const char *testname);
	TestBase *instantiateTest() override;

private:
	const char *_casename;
	const char *_testname;
};

template <class TTest>
TestRegistrar<TTest>::TestRegistrar(
	const char *casename,
	const char *testname) :
	_casename(casename),
	_testname(testname)
{
	TestFactory &factory = TestFactory::instance();
	factory.registerTest(this, casename, testname);
}

template <class TTest>
TestBase *TestRegistrar<TTest>::instantiateTest()
{
	return new TTest();
}

/*
 * The hook below creates global static TestRegistrars which make sure all
 * tests compiled in are instantiated on program init.
 *
 * These TestRegistrars are stripped by the linker when compiling for Nuttx,
 * unless the library/executable linking to the this library is linked with
 * the flags: "-Wl,--whole-archive <libfoo.a> -Wl,--no-whole-archive".
 *
 * The flag __attribute__((used)) which could be used alternatively seems to
 * be ignored by the arm-none-eabi toolchain.
 */
#define REGISTER_TEST(CASENAME, TESTNAME, CONCATNAME) \
	class CONCATNAME : public TestBase \
	{ \
	public: \
		CONCATNAME() = default; \
		virtual ~CONCATNAME() override = default; \
		virtual void run_testbody() override; \
	}; \
	namespace { \
	static constexpr auto CONCATNAME##casestr = #CASENAME; \
	static constexpr auto CONCATNAME##teststr = #TESTNAME; \
	static TestRegistrar<CONCATNAME> CONCATNAME##_registrar( \
			CONCATNAME##casestr, \
			CONCATNAME##teststr); \
	} \
	void CONCATNAME::run_testbody()

#ifndef REGISTER_MAIN
#define REGISTER_MAIN(CASENAME) int unit_##CASENAME_main(int argc,  int **argv) { \
		TestFactory &factory = TestFactory::instance(); \
		auto kase = factory.findCase(#CASENAME); \
		for (auto *test : kase->tests) { \
			test->testbase->run(); \
		} \
		return 0; \
	}
#endif

#define TEST(CASENAME, TESTNAME) \
	REGISTER_MAIN(CASENAME) \
	REGISTER_TEST(CASENAME, TESTNAME, CASENAME##TESTNAME)
