/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

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

#ifndef UAVCAN_CPP_VERSION
# if __cplusplus > 201200
#  error Unsupported C++ standard
# elif (__cplusplus > 201100) || defined(__GXX_EXPERIMENTAL_CXX0X__)
#  define UAVCAN_CPP_VERSION    UAVCAN_CPP11
# else
#  define UAVCAN_CPP_VERSION    UAVCAN_CPP03
# endif
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
# define UAVCAN_PACKED_BEGIN
#endif
#ifndef UAVCAN_PACKED_END
# define UAVCAN_PACKED_END
#endif

/**
 * Declaration visibility
 * http://gcc.gnu.org/wiki/Visibility
 */
#ifndef UAVCAN_EXPORT
# define UAVCAN_EXPORT
#endif

/**
 * Trade-off between ROM/RAM usage and functionality/determinism.
 * Note that this feature is not well tested and should be avoided.
 * Use code search for UAVCAN_TINY to find what functionality will be disabled.
 * This is particularly useful for embedded systems with less than 40kB of ROM.
 */
#ifndef UAVCAN_TINY
# define UAVCAN_TINY 0
#endif

/**
 * It might make sense to remove toString() methods for an embedded system
 */
#ifndef UAVCAN_TOSTRING
# define UAVCAN_TOSTRING 1
#endif

#if UAVCAN_TOSTRING
# include <string>
# include <cstdio>
#endif

/**
 * Some C++ implementations are half-broken and do not implement the placement new operator.
 * Setting this preprocessor symbol to 1 makes libuavcan implement its own operator new (std::size_t, void*),
 * and its delete counterpart.
 * This option may be removed in future.
 */
#ifndef UAVCAN_IMPLEMENT_PLACEMENT_NEW
# define UAVCAN_IMPLEMENT_PLACEMENT_NEW 0
#endif

namespace uavcan
{

/**
 * Should be OK for any target arch up to AMD64 and any compiler.
 * The library leverages compile-time checks to ensure that all types that are subject to dynamic allocation
 * fit this size; otherwise compilation fails.
 */
#if UAVCAN_MEM_POOL_BLOCK_SIZE
static const unsigned MemPoolBlockSize = UAVCAN_MEM_POOL_BLOCK_SIZE;
#else
static const unsigned MemPoolBlockSize = 64;
#endif

#ifdef __BIGGEST_ALIGNMENT__
static const unsigned MemPoolAlignment = __BIGGEST_ALIGNMENT__;
#else
static const unsigned MemPoolAlignment = 16;
#endif

typedef char _alignment_check_for_MEM_POOL_BLOCK_SIZE[((MemPoolBlockSize & (MemPoolAlignment - 1)) == 0) ? 1 : -1];

template <typename T>
struct UAVCAN_EXPORT IsDynamicallyAllocatable
{
    static void check()
    {
        char dummy[(sizeof(T) <= MemPoolBlockSize) ? 1 : -1];
        (void)dummy;
    }
};

}
