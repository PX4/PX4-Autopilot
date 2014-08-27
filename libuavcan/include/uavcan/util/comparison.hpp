/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/util/templates.hpp>
#include <uavcan/build_config.hpp>

namespace uavcan
{
/**
 * This function performs fuzzy comparison of two floating point numbers.
 * Type of T can be either float, double or long double.
 * For details refer to http://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
 */
template <typename T>
UAVCAN_EXPORT
inline bool areFloatsClose(T a, T b, const T absolute_epsilon, const T relative_epsilon)
{
    // NAN
    if (isNaN(a) || isNaN(b))
    {
        return false;
    }

    // Infinity
    if (isInfinity(a) || isInfinity(b))
    {
        return a == b;
    }

    // Close numbers near zero
    const T diff = std::fabs(a - b);
    if (diff <= absolute_epsilon)
    {
        return true;
    }

    // General case
    a = std::fabs(a);
    b = std::fabs(b);
    const T largest = (b > a) ? b : a;
    return (diff <= largest * relative_epsilon);
}

/**
 * Generic comparison function.
 * This function properly handles floating point comparison, including mixed floating point type comparison,
 * e.g. float with long double.
 */
template <typename L, typename R>
UAVCAN_EXPORT
inline bool areClose(const L& left, const R& right)
{
    return left == right;
}

/*
 * Float comparison specializations
 */
template <>
UAVCAN_EXPORT
inline bool areClose<float, float>(const float& left, const float& right)
{
    return areFloatsClose(left, right, NumericTraits<float>::epsilon(),
                          NumericTraits<float>::epsilon() * FloatComparisonEpsilonMult);
}

template <>
UAVCAN_EXPORT
inline bool areClose<double, double>(const double& left, const double& right)
{
    return areFloatsClose(left, right, NumericTraits<double>::epsilon(),
                          NumericTraits<double>::epsilon() * FloatComparisonEpsilonMult);
}

template <>
UAVCAN_EXPORT
inline bool areClose<long double, long double>(const long double& left, const long double& right)
{
    return areFloatsClose(left, right, NumericTraits<long double>::epsilon(),
                          NumericTraits<long double>::epsilon() * FloatComparisonEpsilonMult);
}

/*
 * Mixed floating type comparison - coercing larger type to smaller type
 */
template <>
UAVCAN_EXPORT
inline bool areClose<float, double>(const float& left, const double& right)
{
    return areClose(left, static_cast<float>(right));
}

template <>
UAVCAN_EXPORT
inline bool areClose<double, float>(const double& left, const float& right)
{
    return areClose(static_cast<float>(left), right);
}

template <>
UAVCAN_EXPORT
inline bool areClose<float, long double>(const float& left, const long double& right)
{
    return areClose(left, static_cast<float>(right));
}

template <>
UAVCAN_EXPORT
inline bool areClose<long double, float>(const long double& left, const float& right)
{
    return areClose(static_cast<float>(left), right);
}

template <>
UAVCAN_EXPORT
inline bool areClose<double, long double>(const double& left, const long double& right)
{
    return areClose(left, static_cast<double>(right));
}

template <>
UAVCAN_EXPORT
inline bool areClose<long double, double>(const long double& left, const double& right)
{
    return areClose(static_cast<double>(left), right);
}

/**
 * Comparison against zero.
 * Helps to compare a floating point number against zero if the exact type is unknown.
 * For non-floating point types performs exact comparison against integer zero.
 */
template <typename T>
UAVCAN_EXPORT
inline bool isCloseToZero(const T& x)
{
    return x == 0;
}

template <>
UAVCAN_EXPORT
inline bool isCloseToZero<float>(const float& x)
{
    return areClose(x, static_cast<float>(0.0F));
}

template <>
UAVCAN_EXPORT
inline bool isCloseToZero<double>(const double& x)
{
    return areClose(x, static_cast<double>(0.0));
}

template <>
UAVCAN_EXPORT
inline bool isCloseToZero<long double>(const long double& x)
{
    return areClose(x, static_cast<long double>(0.0L));
}

}
