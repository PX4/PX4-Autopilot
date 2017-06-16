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
	virtual bool run_tests();

private:
	bool autodeclination_check();
};

bool AutoDeclinationTest::autodeclination_check()
{
	ut_assert("declination differs more than 0.1 degrees", get_mag_declination(47.0, 8.0) - 1.6f < 0.1f);
	// Test world endpoints
	ut_assert("declination differs more than 0.1 degrees", get_mag_declination(-90.0, 180.0) - 47.0f < 0.1f);
	ut_assert("declination differs more than 0.1 degrees", get_mag_declination(-90.0, -180.0) - 47.0f < 0.1f);
	ut_assert("declination differs more than 0.1 degrees", get_mag_declination(90.0, -180.0) - 3.0f < 0.1f);
	ut_assert("declination differs more than 0.1 degrees", get_mag_declination(90.0, 180.0) - 3.0f < 0.1f);

	return true;
}

bool AutoDeclinationTest::run_tests()
{
	ut_run_test(autodeclination_check);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_autodeclination, AutoDeclinationTest)
