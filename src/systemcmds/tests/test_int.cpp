#include <unit_test/unit_test.h>

#include <errno.h>
#include <fcntl.h>
#include <float.h>
#include <math.h>
#include <px4_config.h>
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
