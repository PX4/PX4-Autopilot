/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/util/compile_time.hpp>

/**
 * UAVCAN version definition
 * // TODO: Use git short hash as build id
 */
#define UAVCAN_VERSION_MAJOR    0
#define UAVCAN_VERSION_MINOR    1
#define UAVCAN_VERSION_BUILD    0

/**
 * UAVCAN can be compiled in C++11 mode.
 * This macro allows to detect which version of the C++ standard is being used.
 */
#define UAVCAN_CPP11    2011
#define UAVCAN_CPP03    2003

#if __cplusplus > 201200
# error Unsupported C++ standard
#elif (__cplusplus > 201100) || defined(__GXX_EXPERIMENTAL_CXX0X__)
# define UAVCAN_CPP_VERSION    UAVCAN_CPP11
#else
# define UAVCAN_CPP_VERSION    UAVCAN_CPP03
#endif

/**
 * UAVCAN can be explicitly told to ignore exceptions, or it can be detected automatically.
 */
#ifndef UAVCAN_EXCEPTIONS
# if __EXCEPTIONS || _HAS_EXCEPTIONS
#  define UAVCAN_EXCEPTIONS  1
# else
#  define UAVCAN_EXCEPTIONS  0
# endif
#endif

/**
 * Struct layout control.
 * Define UAVCAN_PACK_STRUCTS = 1 to reduce memory usage.
 */
#ifndef UAVCAN_PACK_STRUCTS
# define UAVCAN_PACK_STRUCTS    0
#endif
#ifndef UAVCAN_PACKED_BEGIN
# define UAVCAN_PACKED_BEGIN _Pragma("pack(push, 1)")
#endif
#ifndef UAVCAN_PACKED_END
# define UAVCAN_PACKED_END _Pragma("pack(pop)")
#endif


namespace uavcan
{

/**
 * Should be OK for any target arch up to AMD64 and any compiler.
 * The library leverages compile-time checks to ensure that all types that are subject to dynamic allocation
 * fit this size; otherwise compilation fails.
 */
#if UAVCAN_MEM_POOL_BLOCK_SIZE
enum { MemPoolBlockSize = UAVCAN_MEM_POOL_BLOCK_SIZE };
#else
enum { MemPoolBlockSize = 48 };
#endif

enum { MemPoolAlignment = 8 };

typedef char _alignment_check_for_MEM_POOL_BLOCK_SIZE[((MemPoolBlockSize & (MemPoolAlignment - 1)) == 0) ? 1 : -1];

template <typename T>
struct IsDynamicallyAllocatable
{
    static void check() { StaticAssert<sizeof(T) <= MemPoolBlockSize>::check(); }
};

}
