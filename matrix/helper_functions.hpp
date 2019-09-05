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
 * Wrap value to stay in range [low, high)
 *
 * @param x input possibly outside of the range
 * @param low lower limit of the allowed range
 * @param high upper limit of the allowed range
 * @return wrapped value inside the range, or NAN if value is too far away from range.
 */
template<typename Type>
Type wrap(Type x, Type low, Type high) {
    // already in range
    if (low <= x && x < high) {
        return x;
    }

    // close to range
    Type range = high - low;
    if ((high <= x) && (x < high + (range*100))) {
        while (high <= x) {
            x -= range;
        }
        return x;
    }

    if ((low - (range*100) <= x) && (x < low)) {
        while (x < low) {
            x += range;
        }
        return x;
    }

    // very far from the range -> something went terribly wrong
    return NAN;
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

}
