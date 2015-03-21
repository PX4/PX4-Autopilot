/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_STDINT_HPP_INCLUDED
#define UAVCAN_STDINT_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <cstdio>

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

# include <cstdint>
# include <cstdio>

namespace uavcan
{

using std::snprintf;   // Can be used to replace standard snprintf() with a user-provided one.

typedef std::uint8_t uint8_t;
typedef std::uint16_t uint16_t;
typedef std::uint32_t uint32_t;
typedef std::uint64_t uint64_t;

typedef std::int8_t int8_t;
typedef std::int16_t int16_t;
typedef std::int32_t int32_t;
typedef std::int64_t int64_t;

}

#else

# include <stdint.h>  // Standard integer types from C library
# include <stdio.h>   // snprintf() from the C library

namespace uavcan
{

using ::snprintf;

typedef ::uint8_t uint8_t;
typedef ::uint16_t uint16_t;
typedef ::uint32_t uint32_t;
typedef ::uint64_t uint64_t;

typedef ::int8_t int8_t;
typedef ::int16_t int16_t;
typedef ::int32_t int32_t;
typedef ::int64_t int64_t;

}

#endif

#endif // UAVCAN_STDINT_HPP_INCLUDED
