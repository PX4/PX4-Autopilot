/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/impl_constants.hpp>

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
template<class InputIt1, class InputIt2>
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
template<class InputIt1, class InputIt2>
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

}
