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

}
