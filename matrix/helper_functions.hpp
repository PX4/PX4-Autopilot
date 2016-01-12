#pragma once

#include "math.hpp"

// grody hack - this should go once C++11 is supported
// on all platforms.
#ifdef __PX4_NUTTX
#include <math.h>
#else
#include <cmath>
#endif

namespace matrix
{

template<typename Type>
Type wrap_pi(Type x)
{
#ifdef __PX4_NUTTX
    if (!isfinite(x)) {
#else
    if (!std::isfinite(x)) {
#endif
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
