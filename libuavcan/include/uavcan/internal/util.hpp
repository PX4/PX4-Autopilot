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
struct StaticIf;

template <typename TrueType, typename FalseType>
struct StaticIf<true, TrueType, FalseType>
{
    typedef TrueType Result;
};

template <typename TrueType, typename FalseType>
struct StaticIf<false, TrueType, FalseType>
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
 * Makeshift binder
 */
template <typename ObjectPtr, typename MemFunPtr>
class MethodBinder
{
    ObjectPtr obj_;
    MemFunPtr fun_;

public:
    MethodBinder(ObjectPtr o, MemFunPtr f) : obj_(o), fun_(f) { }

    void operator()() { (obj_->*fun_)(); }

    template <typename Par1>
    void operator()(Par1 p1) { (obj_->*fun_)(p1); }

    template <typename Par1, typename Par2>
    void operator()(Par1 p1, Par2 p2) { (obj_->*fun_)(p1, p2); }
};

}

/// Ensure that conditional comilation macros are present
#include <uavcan/internal/impl_constants.hpp>
