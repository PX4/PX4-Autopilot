#pragma once

#include "math.hpp"

// grody hack - this should go once C++11 is supported
// on all platforms.
#if defined (__PX4_NUTTX) || defined (__PX4_QURT)
#include <math.h>
#else
#include <cmath>
#endif

namespace matrix
{

template<typename Type>
bool isfinite(Type x) {
#if defined (__PX4_NUTTX) || defined (__PX4_QURT)
    return PX4_ISFINITE(x);
#else
    return std::isfinite(x);
#endif
}

template<typename Type>
Type wrap_pi(Type x)
{
    if (!isfinite(x)) {
        return x;
    }

    while (x >= (Type)M_PI) {
        x -= (Type)(2.0 * M_PI);

    }

    while (x < (Type)(-M_PI)) {
        x += (Type)(2.0 * M_PI);

    }

    return x;
}


};
