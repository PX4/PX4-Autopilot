/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <climits>
#include <cmath>
#include <uavcan/impl_constants.hpp>

#ifndef UAVCAN_CPP_VERSION
# error UAVCAN_CPP_VERSION
#endif
#if UAVCAN_CPP_VERSION < UAVCAN_CPP11
# include <float.h>      // cfloat may not be available
#else
# include <cfloat>       // C++11 mode assumes that all standard headers are available
#endif

namespace uavcan
{
/**
 * Usage:
 *  StaticAssert<expression>::check();
 */
template <bool Value>
struct UAVCAN_EXPORT StaticAssert;

template <>
struct UAVCAN_EXPORT StaticAssert<true>
{
    static void check() { }
};

/**
 * Usage:
 *  ShowIntegerAsError<integer_expression>::foobar();
 */
template <long N> struct ShowIntegerAsError;

/**
 * Prevents copying when inherited
 */
class UAVCAN_EXPORT Noncopyable
{
    Noncopyable(const Noncopyable&);
    Noncopyable& operator=(const Noncopyable&);
protected:
    Noncopyable() { }
    ~Noncopyable() { }
};

/**
 * Compile time conditions
 */
template <bool B, typename T = void>
struct UAVCAN_EXPORT EnableIf { };

template <typename T>
struct UAVCAN_EXPORT EnableIf<true, T>
{
    typedef T Type;
};


template <typename T, typename R = void>
struct UAVCAN_EXPORT EnableIfType
{
    typedef R Type;
};


template <bool Condition, typename TrueType, typename FalseType>
struct UAVCAN_EXPORT Select;

template <typename TrueType, typename FalseType>
struct UAVCAN_EXPORT Select<true, TrueType, FalseType>
{
    typedef TrueType Result;
};

template <typename TrueType, typename FalseType>
struct UAVCAN_EXPORT Select<false, TrueType, FalseType>
{
    typedef FalseType Result;
};

/**
 * Value types
 */
template <bool> struct UAVCAN_EXPORT BooleanType { };
typedef BooleanType<true> TrueType;
typedef BooleanType<false> FalseType;

/**
 * Relations
 */
template <typename T1, typename T2>
class UAVCAN_EXPORT IsImplicitlyConvertibleFromTo
{
    template <typename U> static U returner();

    struct True_ { char x[2]; };
    struct False_ { };

    static True_ test(const T2 &);
    static False_ test(...);

public:
    enum { Result = sizeof(True_) == sizeof(IsImplicitlyConvertibleFromTo<T1, T2>::test(returner<T1>())) };
};


template <typename From, typename To>
struct UAVCAN_EXPORT TryImplicitCastImpl
{
    static To impl(const From& from, const To&, TrueType) { return To(from); }
    static To impl(const From&, const To& default_, FalseType) { return default_; }
};

template <typename To, typename From>
UAVCAN_EXPORT
To try_implicit_cast(const From& from, const To& default_)
{
    return TryImplicitCastImpl<From, To>::impl(from, default_,
                                               BooleanType<IsImplicitlyConvertibleFromTo<From, To>::Result>());
}

template <typename To, typename From>
UAVCAN_EXPORT
To try_implicit_cast(const From& from)
{
    return TryImplicitCastImpl<From, To>::impl(from, To(),
                                               BooleanType<IsImplicitlyConvertibleFromTo<From, To>::Result>());
}

/**
 * Some arithmetics
 */
template <unsigned Value> struct UAVCAN_EXPORT CompileTimeIntSqrt;
template <> struct UAVCAN_EXPORT CompileTimeIntSqrt<4>  { enum { Result = 2 }; };
template <> struct UAVCAN_EXPORT CompileTimeIntSqrt<9>  { enum { Result = 3 }; };
template <> struct UAVCAN_EXPORT CompileTimeIntSqrt<16> { enum { Result = 4 }; };
template <> struct UAVCAN_EXPORT CompileTimeIntSqrt<25> { enum { Result = 5 }; };
template <> struct UAVCAN_EXPORT CompileTimeIntSqrt<36> { enum { Result = 6 }; };
template <> struct UAVCAN_EXPORT CompileTimeIntSqrt<49> { enum { Result = 7 }; };
template <> struct UAVCAN_EXPORT CompileTimeIntSqrt<64> { enum { Result = 8 }; };
template <> struct UAVCAN_EXPORT CompileTimeIntSqrt<81> { enum { Result = 9 }; };

/**
 * Replacement for std::copy(..)
 */
template <typename InputIt, typename OutputIt>
UAVCAN_EXPORT
OutputIt copy(InputIt first, InputIt last, OutputIt result)
{
    while (first != last)
    {
        *result = *first;
        ++first;
        ++result;
    }
    return result;
}

/**
 * Replacement for std::fill(..)
 */
template <typename ForwardIt, typename T>
UAVCAN_EXPORT
void fill(ForwardIt first, ForwardIt last, const T& value)
{
    while (first != last)
    {
        *first = value;
        ++first;
    }
}

/**
 * Replacement for std::fill_n(..)
 */
template<typename OutputIt, typename T>
UAVCAN_EXPORT
void fill_n(OutputIt first, std::size_t n, const T& value)
{
    while (n--)
    {
        *first++ = value;
    }
}

/**
 * Replacement for std::min(..)
 */
template <typename T>
UAVCAN_EXPORT
const T& min(const T& a, const T& b)
{
    return (b < a) ? b : a;
}

/**
 * Replacement for std::max(..)
 */
template <typename T>
UAVCAN_EXPORT
const T& max(const T& a, const T& b)
{
    return (a < b) ? b : a;
}

/**
 * Replacement for std::lexicographical_compare(..)
 */
template<typename InputIt1, typename InputIt2>
UAVCAN_EXPORT
bool lexicographical_compare(InputIt1 first1, InputIt1 last1, InputIt2 first2, InputIt2 last2)
{
    while ((first1 != last1) && (first2 != last2))
    {
        if (*first1 < *first2)
        {
            return true;
        }
        if (*first2 < *first1)
        {
            return false;
        }
        ++first1;
        ++first2;
    }
    return (first1 == last1) && (first2 != last2);
}

/**
 * Replacement for std::equal(..)
 */
template<typename InputIt1, typename InputIt2>
UAVCAN_EXPORT
bool equal(InputIt1 first1, InputIt1 last1, InputIt2 first2)
{
    while (first1 != last1)
    {
        if (*first1 != *first2)
        {
            return false;
        }
        ++first1;
        ++first2;
    }
    return true;
}

/**
 * Numeric traits, like std::numeric_limits<>
 */
template <typename T>
struct UAVCAN_EXPORT NumericTraits;

/// char
template <>
struct UAVCAN_EXPORT NumericTraits<char>
{
    enum { IsSigned = 1 };
    enum { IsInteger = 1 };
    static char max() { return CHAR_MAX; }
    static char min() { return CHAR_MIN; }
};
template <>
struct UAVCAN_EXPORT NumericTraits<signed char>
{
    enum { IsSigned = 1 };
    enum { IsInteger = 1 };
    static signed char max() { return SCHAR_MAX; }
    static signed char min() { return SCHAR_MIN; }
};
template <>
struct UAVCAN_EXPORT NumericTraits<unsigned char>
{
    enum { IsSigned = 0 };
    enum { IsInteger = 1 };
    static unsigned char max() { return UCHAR_MAX; }
    static unsigned char min() { return 0; }
};

/// short
template <>
struct UAVCAN_EXPORT NumericTraits<short>
{
    enum { IsSigned = 1 };
    enum { IsInteger = 1 };
    static short max() { return SHRT_MAX; }
    static short min() { return SHRT_MIN; }
};
template <>
struct UAVCAN_EXPORT NumericTraits<unsigned short>
{
    enum { IsSigned = 0 };
    enum { IsInteger = 1 };
    static unsigned short max() { return USHRT_MAX; }
    static unsigned short min() { return 0; }
};

/// int
template <>
struct UAVCAN_EXPORT NumericTraits<int>
{
    enum { IsSigned = 1 };
    enum { IsInteger = 1 };
    static int max() { return INT_MAX; }
    static int min() { return INT_MIN; }
};
template <>
struct UAVCAN_EXPORT NumericTraits<unsigned int>
{
    enum { IsSigned = 0 };
    enum { IsInteger = 1 };
    static unsigned int max() { return UINT_MAX; }
    static unsigned int min() { return 0; }
};

/// long
template <>
struct UAVCAN_EXPORT NumericTraits<long>
{
    enum { IsSigned = 1 };
    enum { IsInteger = 1 };
    static long max() { return LONG_MAX; }
    static long min() { return LONG_MIN; }
};
template <>
struct UAVCAN_EXPORT NumericTraits<unsigned long>
{
    enum { IsSigned = 0 };
    enum { IsInteger = 1 };
    static unsigned long max() { return ULONG_MAX; }
    static unsigned long min() { return 0; }
};

/// long long
template <>
struct UAVCAN_EXPORT NumericTraits<long long>
{
    enum { IsSigned = 1 };
    enum { IsInteger = 1 };
    static long long max() { return LLONG_MAX; }
    static long long min() { return LLONG_MIN; }
};
template <>
struct UAVCAN_EXPORT NumericTraits<unsigned long long>
{
    enum { IsSigned = 0 };
    enum { IsInteger = 1 };
    static unsigned long long max() { return ULLONG_MAX; }
    static unsigned long long min() { return 0; }
};

/// float
template <>
struct UAVCAN_EXPORT NumericTraits<float>
{
    enum { IsSigned = 1 };
    enum { IsInteger = 0 };
    static float max() { return FLT_MAX; }
    static float min() { return FLT_MIN; }
    static float infinity() { return INFINITY; }
};

/// double
template <>
struct UAVCAN_EXPORT NumericTraits<double>
{
    enum { IsSigned = 1 };
    enum { IsInteger = 0 };
    static double max() { return DBL_MAX; }
    static double min() { return DBL_MIN; }
    static double infinity() { return static_cast<double>(INFINITY) * static_cast<double>(INFINITY); }
};

/// long double
template <>
struct UAVCAN_EXPORT NumericTraits<long double>
{
    enum { IsSigned = 1 };
    enum { IsInteger = 0 };
    static long double max() { return LDBL_MAX; }
    static long double min() { return LDBL_MIN; }
    static long double infinity() { return static_cast<long double>(INFINITY) * static_cast<long double>(INFINITY); }
};

}
