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

#include "ntestlib.h"
#include <cstring>

TestBase::Result TestBase::run()
{
	_result = Result::Success;

	pthread_t runner_thread;

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, PX4_STACK_ADJUSTED(1000));

	pthread_create(&runner_thread, &attr, TestBase::run_trampoline, this);

	pthread_attr_destroy(&attr);

	pthread_join(runner_thread, NULL);

	return _result;
}

void *TestBase::run_trampoline(void *self)
{
	reinterpret_cast<TestBase *>(self)->run_testbody();
	return nullptr;
}

void TestBase::abort_test()
{
	pthread_exit(NULL);
}

const char *TestBase::extractFilename(const char *path)
{
	const char *pos = strrchr(path, '/');

	if (!pos) {
		return nullptr;
	}

	return pos + 1;
}

const char *str(const TestBase::Result result)
{
	switch (result) {
	case TestBase::Result::Success: return "Success";

	case TestBase::Result::Failed: return "Failed";

	case TestBase::Result::Timeout: return "Timeout";

	case TestBase::Result::NotImplemented: return "Not implemented";

	default: return "Unknown";
	}
}

TestFactory &TestFactory::instance()
{
	static TestFactory instance;
	return instance;
}

void TestFactory::registerTest(ITestRegistrar *registrar, const char *casename, const char *testname)
{
	auto *test = new Test();
	test->testname = testname;
	test->testbase = registrar->instantiateTest();

	Case *kase = findCase(casename);

	if (kase) {
		kase->tests.add(test);

	} else {
		auto *new_case = new Case();
		new_case->casename = casename;
		new_case->tests.add(test);
		_cases.add(new_case);
	}

}

List<TestFactory::Case *> &TestFactory::getAllCases()
{
	return _cases;
}

unsigned TestFactory::getNumTests()
{
	unsigned num = 0;

	for (const auto kase : _cases) {
		num += kase->tests.size();
	}

	return num;
}

unsigned TestFactory::getNumCases()
{
	return _cases.size();
}

TestFactory::Case *TestFactory::findCase(const char *casename)
{
	for (const auto kase : _cases) {
		if (strcmp(kase->casename, casename) == 0) {
			return kase;
		}
	}

	return nullptr;
}
