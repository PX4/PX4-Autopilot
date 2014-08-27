/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/marshal/float_spec.hpp>
#include <uavcan/build_config.hpp>
#include <cmath>

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
# include <limits>
#endif

namespace uavcan
{
/*
 * IEEE754Converter
 * Float16 conversion algorithm: http://half.sourceforge.net/ (MIT License)
 */
uint16_t IEEE754Converter::nativeNonIeeeToHalf(float value)
{
    uint16_t hbits = uint16_t(getSignBit(value) ? 0x8000U : 0);
    if (areFloatsExactlyEqual(value, 0.0F))
    {
        return hbits;
    }
    if (isNaN(value))
    {
        return hbits | 0x7FFFU;
    }
    if (isInfinity(value))
    {
        return hbits | 0x7C00U;
    }
    int exp;
    (void)std::frexp(value, &exp);
    if (exp > 16)
    {
        return hbits | 0x7C00U;
    }
    if (exp < -13)
    {
        value = std::ldexp(value, 24);
    }
    else
    {
        value = std::ldexp(value, 11 - exp);
        hbits |= uint16_t((exp + 14) << 10);
    }
    const int32_t ival = static_cast<int32_t>(value);
    hbits = uint16_t(hbits | (uint32_t((ival < 0) ? (-ival) : ival) & 0x3FFU));
    float diff = std::fabs(value - static_cast<float>(ival));
    hbits = uint16_t(hbits + (diff >= 0.5F));
    return hbits;
}

float IEEE754Converter::halfToNativeNonIeee(uint16_t value)
{
    float out;
    unsigned abs = value & 0x7FFFU;
    if (abs > 0x7C00U)
    {
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
        out = std::numeric_limits<float>::has_quiet_NaN ? std::numeric_limits<float>::quiet_NaN() : 0.0F;
#else
        out = nanf("");
#endif
    }
    else if (abs == 0x7C00U)
    {
#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11
        out = std::numeric_limits<float>::has_infinity ?
              std::numeric_limits<float>::infinity() : std::numeric_limits<float>::max();
#else
        out = NumericTraits<float>::infinity();
#endif
    }
    else if (abs > 0x3FFU)
    {
        out = std::ldexp(static_cast<float>((value & 0x3FFU) | 0x400U), int(abs >> 10) - 25);
    }
    else
    {
        out = std::ldexp(static_cast<float>(abs), -24);
    }
    return (value & 0x8000U) ? -out : out;
}

}
