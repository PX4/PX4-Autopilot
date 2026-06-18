#include <rl_tools/operations/cpu.h>
#include <gtest/gtest.h>
#include <math.h>
namespace rlt = rl_tools;
using DEVICE = rlt::devices::DefaultCPU;

TEST(RL_TOOLS_MATH, MAIN){
    // if msvc
    #if !defined(_MSC_VER)
    float nan_0 = 0.0f / 0.0f;
    #else
    float nan_0 = NAN;
    #endif

    float nan_1 = NAN;
    float nan_2 = std::numeric_limits<float>::quiet_NaN();
    float nan_3 = std::numeric_limits<float>::signaling_NaN();
    float normal_0 = std::numeric_limits<float>::infinity();
    float normal_1 = std::numeric_limits<float>::epsilon();
    float normal_2 = 0;
    float normal_3 = 1;

    DEVICE device;
    using T = float;

#ifdef RL_TOOLS_DISABLE_FAST_MATH
    ASSERT_TRUE(rlt::math::is_nan(device.math, nan_0) == rlt::isnan(device.math, nan_0));
    ASSERT_TRUE(rlt::math::is_nan(device.math, nan_1) == rlt::isnan(device.math, nan_1));
    ASSERT_TRUE(rlt::math::is_nan(device.math, nan_2) == rlt::isnan(device.math, nan_2));
    ASSERT_TRUE(rlt::math::is_nan(device.math, nan_3) == rlt::isnan(device.math, nan_3));
    ASSERT_TRUE(rlt::math::is_nan(device.math, normal_0) == rlt::isnan(device.math, normal_0));
    ASSERT_TRUE(rlt::math::is_nan(device.math, normal_1) == rlt::isnan(device.math, normal_1));
    ASSERT_TRUE(rlt::math::is_nan(device.math, normal_2) == rlt::isnan(device.math, normal_2));
    ASSERT_TRUE(rlt::math::is_nan(device.math, normal_3) == rlt::isnan(device.math, normal_3));
#else
    ASSERT_TRUE(rlt::math::is_nan(device.math, nan_0));
    ASSERT_TRUE(rlt::math::is_nan(device.math, nan_1));
    ASSERT_TRUE(rlt::math::is_nan(device.math, nan_2));
    ASSERT_TRUE(rlt::math::is_nan(device.math, nan_3));
    ASSERT_FALSE(rlt::math::is_nan(device.math, normal_0));
    ASSERT_FALSE(rlt::math::is_nan(device.math, normal_1));
    ASSERT_FALSE(rlt::math::is_nan(device.math, normal_2));
    ASSERT_FALSE(rlt::math::is_nan(device.math, normal_3));
#endif
}

template <typename T>
void test(T EPSILON){
    DEVICE device;

    EXPECT_NEAR(rlt::math::sqrt<T>(device.math, 4.0), 2.0, EPSILON);
    EXPECT_NEAR(rlt::math::sqrt<T>(device.math, 9.0), 3.0, EPSILON);
    EXPECT_NEAR(rlt::math::sqrt<T>(device.math, 0.0), 0.0, EPSILON);
    EXPECT_NEAR(rlt::math::sqrt<T>(device.math, 16.0), 4.0, EPSILON);
    EXPECT_NEAR(rlt::math::sqrt<T>(device.math, 25.0), 5.0, EPSILON);
    // Test cases for cbrt
    EXPECT_NEAR(rlt::math::cbrt<T>(device.math, 8.0), 2.0, EPSILON);
    EXPECT_NEAR(rlt::math::cbrt<T>(device.math, 27.0), 3.0, EPSILON);
    EXPECT_NEAR(rlt::math::cbrt<T>(device.math, 0.0), 0.0, EPSILON);
    EXPECT_NEAR(rlt::math::cbrt<T>(device.math, -8.0), -2.0, EPSILON);
    EXPECT_NEAR(rlt::math::cbrt<T>(device.math, 64.0), 3.9999999999999996, EPSILON);
    // Test cases for tanh
    EXPECT_NEAR(rlt::math::tanh<T>(device.math, 0.0), 0.0, EPSILON);
    EXPECT_NEAR(rlt::math::tanh<T>(device.math, 1.0), 0.7615941559557649, EPSILON);
    EXPECT_NEAR(rlt::math::tanh<T>(device.math, -1.0), -0.7615941559557649, EPSILON);
    EXPECT_NEAR(rlt::math::tanh<T>(device.math, 0.5), 0.46211715726000974, EPSILON);
    EXPECT_NEAR(rlt::math::tanh<T>(device.math, -0.5), -0.46211715726000974, EPSILON);
    // Test cases for exp
    EXPECT_NEAR(rlt::math::exp<T>(device.math, 0.0), 1.0, EPSILON);
    EXPECT_NEAR(rlt::math::exp<T>(device.math, 1.0), 2.718281828459045, EPSILON);
    EXPECT_NEAR(rlt::math::exp<T>(device.math, -1.0), 0.36787944117144233, EPSILON);
    EXPECT_NEAR(rlt::math::exp<T>(device.math, 2.0), 7.38905609893065, EPSILON);
    EXPECT_NEAR(rlt::math::exp<T>(device.math, -2.0), 0.1353352832366127, EPSILON);
    // Test cases for sin
    EXPECT_NEAR(rlt::math::sin<T>(device.math, 0.0), 0.0, EPSILON);
    EXPECT_NEAR(rlt::math::sin<T>(device.math, 1.5707963267948966), 1.0, EPSILON);
    EXPECT_NEAR(rlt::math::sin<T>(device.math, 3.141592653589793), 1.2246467991473532e-16, EPSILON);
    EXPECT_NEAR(rlt::math::sin<T>(device.math, 4.71238898038469), -1.0, EPSILON);
    EXPECT_NEAR(rlt::math::sin<T>(device.math, 6.283185307179586), -2.4492935982947064e-16, EPSILON);
    // Test cases for cos
    EXPECT_NEAR(rlt::math::cos<T>(device.math, 0.0), 1.0, EPSILON);
    EXPECT_NEAR(rlt::math::cos<T>(device.math, 1.5707963267948966), 6.123233995736766e-17, EPSILON);
    EXPECT_NEAR(rlt::math::cos<T>(device.math, 3.141592653589793), -1.0, EPSILON);
    EXPECT_NEAR(rlt::math::cos<T>(device.math, 4.71238898038469), -1.8369701987210297e-16, EPSILON);
    EXPECT_NEAR(rlt::math::cos<T>(device.math, 6.283185307179586), 1.0, EPSILON);
    // Test cases for acos
    EXPECT_NEAR(rlt::math::acos<T>(device.math, 1.0), 0.0, EPSILON);
    EXPECT_NEAR(rlt::math::acos<T>(device.math, 0.5), 1.0471975511965979, EPSILON);
    EXPECT_NEAR(rlt::math::acos<T>(device.math, 0.0), 1.5707963267948966, EPSILON);
    EXPECT_NEAR(rlt::math::acos<T>(device.math, -0.5), 2.0943951023931957, EPSILON);
    EXPECT_NEAR(rlt::math::acos<T>(device.math, -1.0), 3.141592653589793, EPSILON);
    // Test cases for pow
    EXPECT_NEAR(rlt::math::pow<T>(device.math, 2, 3), 8, EPSILON);
    EXPECT_NEAR(rlt::math::pow<T>(device.math, 4, 0.5), 2.0, EPSILON);
    EXPECT_NEAR(rlt::math::pow<T>(device.math, 10, -1), 0.1, EPSILON);
    EXPECT_NEAR(rlt::math::pow<T>(device.math, 1, 5), 1, EPSILON);
    EXPECT_NEAR(rlt::math::pow<T>(device.math, 0, 1), 0, EPSILON);
    // Test cases for log
    EXPECT_NEAR(rlt::math::log<T>(device.math, 1.0), 0.0, EPSILON);
    EXPECT_NEAR(rlt::math::log<T>(device.math, 2.718281828459045), 1.0, EPSILON);
    EXPECT_NEAR(rlt::math::log<T>(device.math, 10.0), 2.302585092994046, EPSILON);
    EXPECT_NEAR(rlt::math::log<T>(device.math, 0.1), -2.3025850929940455, EPSILON);
    EXPECT_NEAR(rlt::math::log<T>(device.math, 100.0), 4.605170185988092, EPSILON);
    // Test cases for floor
    EXPECT_NEAR(rlt::math::floor<T>(device.math, 2.5), 2, EPSILON);
    EXPECT_NEAR(rlt::math::floor<T>(device.math, 3.7), 3, EPSILON);
    EXPECT_NEAR(rlt::math::floor<T>(device.math, -1.2), -2, EPSILON);
    EXPECT_NEAR(rlt::math::floor<T>(device.math, -0.8), -1, EPSILON);
    EXPECT_NEAR(rlt::math::floor<T>(device.math, 0.0), 0, EPSILON);
    // Test cases for is_finite
#ifdef RL_TOOLS_DISABLE_FAST_MATH
    EXPECT_TRUE(rlt::math::is_finite<T>(device.math, 1.0));
    EXPECT_FALSE(rlt::math::is_finite<T>(device.math, rlt::math::infinity<T>(device.math)));
    EXPECT_FALSE(rlt::math::is_finite<T>(device.math, -rlt::math::infinity<T>(device.math)));
    EXPECT_TRUE(rlt::math::is_finite<T>(device.math, 0.0));
    EXPECT_FALSE(rlt::math::is_finite<T>(device.math, rlt::math::nan<T>(device.math)));
#endif
    // Test cases for clamp
    EXPECT_NEAR(rlt::math::clamp<T>(device.math, 5, 1, 10), 5, EPSILON);
    EXPECT_NEAR(rlt::math::clamp<T>(device.math, 15, 1, 10), 10, EPSILON);
    EXPECT_NEAR(rlt::math::clamp<T>(device.math, -5, -10, -1), -5, EPSILON);
    EXPECT_NEAR(rlt::math::clamp<T>(device.math, 0, -1, 1), 0, EPSILON);
    EXPECT_NEAR(rlt::math::clamp<T>(device.math, 10, 10, 10), 10, EPSILON);
    // Test cases for min
    EXPECT_NEAR(rlt::math::min<T>(device.math, 3, 5), 3, EPSILON);
    EXPECT_NEAR(rlt::math::min<T>(device.math, -1, -5), -5, EPSILON);
    EXPECT_NEAR(rlt::math::min<T>(device.math, 0, 0), 0, EPSILON);
    EXPECT_NEAR(rlt::math::min<T>(device.math, 10, 3), 3, EPSILON);
    EXPECT_NEAR(rlt::math::min<T>(device.math, -2, 2), -2, EPSILON);
    // Test cases for max
    EXPECT_NEAR(rlt::math::max<T>(device.math, 3, 5), 5, EPSILON);
    EXPECT_NEAR(rlt::math::max<T>(device.math, -1, -5), -1, EPSILON);
    EXPECT_NEAR(rlt::math::max<T>(device.math, 0, 0), 0, EPSILON);
    EXPECT_NEAR(rlt::math::max<T>(device.math, 10, 3), 10, EPSILON);
    EXPECT_NEAR(rlt::math::max<T>(device.math, -2, 2), 2, EPSILON);
    // Test cases for abs
    EXPECT_NEAR(rlt::math::abs<T>(device.math, 3), 3, EPSILON);
    EXPECT_NEAR(rlt::math::abs<T>(device.math, -3), 3, EPSILON);
    EXPECT_NEAR(rlt::math::abs<T>(device.math, 0), 0, EPSILON);
    EXPECT_NEAR(rlt::math::abs<T>(device.math, -7.5), 7.5, EPSILON);
    EXPECT_NEAR(rlt::math::abs<T>(device.math, 7.5), 7.5, EPSILON);
    // Test cases for atan2
    EXPECT_NEAR(rlt::math::atan2<T>(device.math, 0, 1), 0.0, EPSILON);
    EXPECT_NEAR(rlt::math::atan2<T>(device.math, 1, 1), 0.7853981633974483, EPSILON);
    EXPECT_NEAR(rlt::math::atan2<T>(device.math, 1, 0), 1.5707963267948966, EPSILON);
    EXPECT_NEAR(rlt::math::atan2<T>(device.math, 0, -1), 3.141592653589793, EPSILON);
    EXPECT_NEAR(rlt::math::atan2<T>(device.math, -1, -1), -2.356194490192345, EPSILON);
}

TEST(RL_TOOLS_MATH, FUNCTIONS){
    test<float>(1e-6);
    test<double>(1e-10);
}
