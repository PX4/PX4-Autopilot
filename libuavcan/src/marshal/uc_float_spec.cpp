/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/impl_constants.hpp>
#include <uavcan/marshal/float_spec.hpp>
#include <cmath>

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

#undef signbit
#undef isnan
#undef isinf

namespace uavcan
{
/*
 * IEEE754Converter
 * Float16 conversion algorithm: http://half.sourceforge.net/ (MIT License)
 * TODO: Use conversion tables (conditional compilation - it would require something like 10Kb+ ROM).
 */
template <typename T>
static inline bool signbit(T arg)
{
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
    return std::signbit(arg);
#else
    return arg < T(0) || (arg == T(0) && T(1) / arg < T(0));
#endif
}

template <typename T>
static inline bool isnan(T arg)
{
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
    return std::isnan(arg);
#else
    return arg != arg;
#endif
}

template <typename T>
static inline bool isinf(T arg)
{
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
    return std::isinf(arg);
#else
    return arg == std::numeric_limits<T>::infinity() || arg == -std::numeric_limits<T>::infinity();
#endif
}

uint16_t IEEE754Converter::nativeNonIeeeToHalf(float value)
{
    uint16_t hbits = signbit(value) << 15;
    if (value == 0.0f)
    {
        return hbits;
    }
    if (isnan(value))
    {
        return hbits | 0x7FFF;
    }
    if (isinf(value))
    {
        return hbits | 0x7C00;
    }
    int exp;
    std::frexp(value, &exp);
    if (exp > 16)
    {
        return hbits | 0x7C00;
    }
    if (exp < -13)
    {
        value = std::ldexp(value, 24);
    }
    else
    {
        value = std::ldexp(value, 11 - exp);
        hbits |= ((exp + 14) << 10);
    }
    const int ival = static_cast<int>(value);
    hbits |= static_cast<uint16_t>(std::abs(ival) & 0x3FF);
    float diff = std::abs(value - static_cast<float>(ival));
    hbits += diff >= 0.5f;
    return hbits;
}

float IEEE754Converter::halfToNativeNonIeee(uint16_t value)
{
    float out;
    int abs = value & 0x7FFF;
    if (abs > 0x7C00)
    {
        out = std::numeric_limits<float>::has_quiet_NaN ? std::numeric_limits<float>::quiet_NaN() : 0.0f;
    }
    else if (abs == 0x7C00)
    {
        out = std::numeric_limits<float>::has_infinity ?
              std::numeric_limits<float>::infinity() : std::numeric_limits<float>::max();
    }
    else if (abs > 0x3FF)
    {
        out = std::ldexp(static_cast<float>((value & 0x3FF) | 0x400), (abs >> 10) - 25);
    }
    else
    {
        out = std::ldexp(static_cast<float>(abs), -24);
    }
    return (value & 0x8000) ? -out : out;
}

}
