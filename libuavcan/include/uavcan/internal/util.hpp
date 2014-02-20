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
template <bool VALUE>
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


class Noncopyable
{
    Noncopyable(const Noncopyable&);
    Noncopyable& operator=(const Noncopyable&);
protected:
    Noncopyable() { }
};


template<bool B, typename T = void>
struct EnableIf { };

template<typename T>
struct EnableIf<true, T> { typedef T Type; };

}
