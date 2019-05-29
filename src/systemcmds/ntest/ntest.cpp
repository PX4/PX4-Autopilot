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
 * @file ntest_runner.cpp
 *
 * Runs ntests which is a light implementation of gtest for NuttX
 * where the C++ STL is not available.
 */

#include <px4_log.h>
#include <px4_module.h>
#include <cstdio>
#include <cstring>
#include <ntestlib/ntestlib.h>


__BEGIN_DECLS
__EXPORT int ntest_main(int argc, char *argv[]);
__END_DECLS

static void usage(const char *reason);
static int run_all_tests();
static void list_tests();


static void
usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This command runs the ntests.

### Examples
Run all ntests
$ ntest all

List all tests
$ ntest list_tests

)DESCR_STR");


	PRINT_MODULE_USAGE_NAME("ntest", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("all", "Run all tests");
	PRINT_MODULE_USAGE_COMMAND_DESCR("list_tests", "List all tests");
}

int
ntest_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage(nullptr);
		return 1;
	}

	if (strcmp(argv[1], "all") == 0) {
		return run_all_tests();

	} else if (strcmp(argv[1], "list_tests") == 0) {
		list_tests();
		return 1;

	} else {
		usage(nullptr);
		return 1;
	}
}

int run_all_tests()
{
	const unsigned num_cases = TestFactory::instance().getNumCases();
	const unsigned num_tests = TestFactory::instance().getNumTests();

	printf("[==========] Running %d test%s from %d test case%s.\n",
			num_tests,
			(num_tests > 1 ? "s" : ""),
			num_cases,
			(num_cases > 1 ? "s" : ""));

	unsigned num_failed = 0;
	for (auto kase : TestFactory::instance().getAllCases()) {

		printf("[----------] %d test%s from %s\n",
				kase->tests.size(),
				(kase->tests.size() > 1 ? "s" : ""),
				kase->casename);

		for (auto test : kase->tests) {
			printf("[ RUN      ] %s.%s\n", kase->casename, test->testname);
			if (test->testbase->run() == TestBase::Result::Success) {
				printf("[       OK ]");
			} else {
				printf("[  FAILED  ]");
				++num_failed;
			}
			printf(" %s.%s\n", kase->casename, test->testname);
		}

		printf("[----------] %d test%s from %s\n",
				kase->tests.size(),
				(kase->tests.size() > 1 ? "s" : ""),
				kase->casename);
	}

	printf("[==========] %d test%s from %d test case%s ran.\n",
			num_tests,
			(num_tests > 1 ? "s" : ""),
			num_cases,
			(num_cases > 1 ? "s" : ""));

	printf("[  PASSED  ] %d test%s.\n",
			num_tests - num_failed,
			(num_tests - num_failed) > 1 ? "s" : "");

	if (num_failed > 0) {
		printf("[  FAILED  ] %d test%s, listed below:\n",
				num_failed,
				(num_failed) > 1 ? "s" : "");

		for (auto kase : TestFactory::instance().getAllCases()) {
			for (auto test : kase->tests) {
				if (test->testbase->getLastResult() != TestBase::Result::Success) {
					printf("[  FAILED  ] %s.%s\n",
						kase->casename, test->testname);
				}
			}
		}
	}

	return (num_failed == 0 ? 0 : 1);
}

void list_tests()
{
	for (auto kase : TestFactory::instance().getAllCases()) {
		printf("%s.\n", kase->casename);
		for (auto test : kase->tests) {
			printf("  %s\n", test->testname);
		}
	}
}
