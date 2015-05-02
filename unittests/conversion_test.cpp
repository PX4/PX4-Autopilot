#include <systemlib/mixer/mixer.h>
#include <systemlib/err.h>
#include "../../src/systemcmds/tests/tests.h"

#include "gtest/gtest.h"

TEST(ConversionTest, quad_w_main)
{
	ASSERT_EQ(test_conv(0, NULL), 0) << "Conversion test failed";
}
