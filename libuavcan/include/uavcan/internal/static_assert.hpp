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
struct StaticAssert
{
#if __CDT_PARSER__
    static void check() { assert(0); }
#endif
};

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

}
