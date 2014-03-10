/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>

namespace uavcan
{
/**
 * Usage:
 *  StaticAssert<expression>::check();
 */
template <bool Value>
struct StaticAssert;

template <>
struct StaticAssert<true>
{
    static void check() { }
};

/**
 * Usage:
 *  ShowIntegerAsError<integer_expression>::foobar();
 */
template<long N> struct ShowIntegerAsError;

/**
 * Prevents copying when inherited
 */
class Noncopyable
{
    Noncopyable(const Noncopyable&);
    Noncopyable& operator=(const Noncopyable&);
protected:
    Noncopyable() { }
};

/**
 * Compile time conditions
 */
template<bool B, typename T = void>
struct EnableIf { };

template<typename T>
struct EnableIf<true, T> { typedef T Type; };


template<typename T, typename R = void>
struct EnableIfType { typedef R Type; };


template <bool Condition, typename TrueType, typename FalseType>
struct Select;

template <typename TrueType, typename FalseType>
struct Select<true, TrueType, FalseType>
{
    typedef TrueType Result;
};

template <typename TrueType, typename FalseType>
struct Select<false, TrueType, FalseType>
{
    typedef FalseType Result;
};

/**
 * Value types
 */
template<bool> struct BooleanType { };
typedef BooleanType<true> TrueType;
typedef BooleanType<false> FalseType;

/**
 * Relations
 */
template <typename T1, typename T2>
class IsImplicitlyConvertibleFromTo
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
struct TryImplicitCastImpl
{
    static To impl(const From& from, const To&, TrueType) { return To(from); }
    static To impl(const From&, const To& default_, FalseType) { return default_; }
};

template <typename To, typename From>
To try_implicit_cast(const From& from, const To& default_)
{
    return TryImplicitCastImpl<From, To>::impl(from, default_,
        BooleanType<IsImplicitlyConvertibleFromTo<From, To>::Result>());
}

template <typename To, typename From>
To try_implicit_cast(const From& from)
{
    return TryImplicitCastImpl<From, To>::impl(from, To(),
        BooleanType<IsImplicitlyConvertibleFromTo<From, To>::Result>());
}

}
