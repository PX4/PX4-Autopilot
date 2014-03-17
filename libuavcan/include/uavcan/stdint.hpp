/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/impl_constants.hpp>

#if !defined(UAVCAN_CPP_VERSION) || !defined(UAVCAN_CPP11)
# error UAVCAN_CPP_VERSION
#endif

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

# include <cstdint>

namespace uavcan
{

using std::uint8_t;
using std::uint16_t;
using std::uint32_t;
using std::uint64_t;

using std::int8_t;
using std::int16_t;
using std::int32_t;
using std::int64_t;

}

#else

# include <stdint.h>

namespace uavcan
{

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
