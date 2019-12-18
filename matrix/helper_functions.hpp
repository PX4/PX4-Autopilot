#pragma once

#include "math.hpp"

#if defined (__PX4_NUTTX) || defined (__PX4_QURT)
#include <px4_defines.h>
#endif

namespace matrix
{

template<typename Type>
bool is_finite(Type x) {
#if defined (__PX4_NUTTX)
    return PX4_ISFINITE(x);
#elif defined (__PX4_QURT)
    return __builtin_isfinite(x);
#else
    return std::isfinite(x);
#endif
}

/**
 * Compare if two floating point numbers are equal
 *
 * NAN is considered equal to NAN and -NAN
 * INFINITY is considered equal INFINITY but not -INFINITY
 *
 * @param x right side of equality check
 * @param y left side of equality check
 * @param eps numerical tolerance of the check
 * @return true if the two values are considered equal, false otherwise
 */
template<typename Type>
bool isEqualF(const Type x, const Type y, const Type eps = 1e-4f)
{
    return (matrix::fabs(x - y) <= eps)
           || (isnan(x) && isnan(y))
           || (isinf(x) && isinf(y) && isnan(x - y));
}

/**
 * Wrap value to stay in range [low, high)
 *
 * @param x input possibly outside of the range
 * @param low lower limit of the allowed range
 * @param high upper limit of the allowed range
 * @return wrapped value inside the range
 */
template<typename Type>
Type wrap(Type x, Type low, Type high) {
    // already in range
    if (low <= x && x < high) {
        return x;
    }

    const Type range = high - low;
    const Type inv_range = Type(1) / range; // should evaluate at compile time, multiplies below at runtime
    const Type num_wraps = floor((x - low) * inv_range);
    return x - range * num_wraps;
}

/**
 * Wrap value in range [-π, π)
 */
template<typename Type>
Type wrap_pi(Type x)
{
    return wrap(x, Type(-M_PI), Type(M_PI));
}

/**
 * Wrap value in range [0, 2π)
 */
template<typename Type>
Type wrap_2pi(Type x)
{
    return wrap(x, Type(0), Type(M_TWOPI));
}

template<typename T>
int sign(T val)
{
    return (T(FLT_EPSILON) < val) - (val < T(FLT_EPSILON));
}

} // namespace matrix
