#include <systemlib/mixer/mixer.h>
#include <systemlib/err.h>
#include "../../src/systemcmds/tests/tests.h"

#include "gtest/gtest.h"

TEST(ConversionTest, FMU_quad_w) {
        ASSERT_EQ(test_conv(0, NULL), 0) << "Conversion test failed";
}
