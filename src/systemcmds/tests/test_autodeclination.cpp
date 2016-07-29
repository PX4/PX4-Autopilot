#include <unit_test/unit_test.h>

#include <drivers/drv_hrt.h>
#include <geo/geo.h>
#include <px4iofirmware/px4io.h>
#include <systemlib/err.h>
#include <systemlib/mixer/mixer.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

class AutoDeclinationTest : public UnitTest
{
public:
	virtual bool run_tests(void);

private:
	bool autodeclination_check();
};

bool AutoDeclinationTest::autodeclination_check(void)
{
	ut_assert("declination differs more than 1 degree", get_mag_declination(47.0, 8.0) - 0.6f < 0.5f);

	return true;
}

bool AutoDeclinationTest::run_tests(void)
{
	ut_run_test(autodeclination_check);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_autodeclination, AutoDeclinationTest)
