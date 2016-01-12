#pragma once

#include "math.hpp"
#include <math.h>


namespace matrix
{
template<typename Type>
float wrap_pi(Type x)
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