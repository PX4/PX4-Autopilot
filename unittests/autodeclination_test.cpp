#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <geo/geo.h>
#include <px4iofirmware/px4io.h>
#include <systemlib/err.h>
#include <systemlib/mixer/mixer.h>

#include "gtest/gtest.h"

TEST(AutoDeclinationTest, AutoDeclination)
{
	ASSERT_NEAR(get_mag_declination(47.0, 8.0), 0.6, 0.5) << "declination differs more than 1 degree";
}
