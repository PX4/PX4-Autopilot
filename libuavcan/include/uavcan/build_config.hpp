/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

/**
 * UAVCAN version definition
 */
#define UAVCAN_VERSION_MAJOR    0
#define UAVCAN_VERSION_MINOR    1

/**
 * UAVCAN_CPP_VERSION - version of the C++ standard used during compilation.
 * This definition contains the integer year number after which the standard was named:
 *  - 2003 for C++03
 *  - 2011 for C++11
 *
 * This config automatically sets according to the actual C++ standard used by the compiler.
 *
 * In C++03 mode the library has almost zero dependency on the C++ standard library, which allows
 * to use it on platforms with a very limited C++ support. On the other hand, C++11 mode requires
 * many parts of the standard library (e.g. <functional>), thus the user might want to force older
 * standard than used by the compiler, in which case this symbol can be overridden manually via
 * compiler flags.
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
 * Autodetection is not expected to work with all compilers, so it's safer to define it explicitly.
 * If the autodetection fails, exceptions will be disabled by default.
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
 * Set UAVCAN_PACK_STRUCTS=1 and define UAVCAN_PACKED_BEGIN and UAVCAN_PACKED_END to reduce memory usage.
 * THIS MAY BREAK THE CODE.
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
 * It might make sense to remove toString() methods for an embedded system.
 * If the autodetect fails, toString() will be disabled, so it's pretty safe by default.
 */
#ifndef UAVCAN_TOSTRING
// Objective is to make sure that we're NOT on a resource constrained platform
# if __linux__ || __linux || __APPLE__ || _WIN64 || _WIN32
#  define UAVCAN_TOSTRING 1
# else
#  define UAVCAN_TOSTRING 0
# endif
#endif

#if UAVCAN_TOSTRING
# include <string>
# include <cstdio>
#endif

/**
 * Some C++ implementations are half-broken because they don't implement the placement new operator.
 * If UAVCAN_IMPLEMENT_PLACEMENT_NEW is defined, libuavcan will implement its own operator new (std::size_t, void*)
 * and its delete() counterpart, instead of relying on the standard header <new>.
 */
#ifndef UAVCAN_IMPLEMENT_PLACEMENT_NEW
# define UAVCAN_IMPLEMENT_PLACEMENT_NEW 0
#endif

/**
 * Run time checks.
 * Resolves to the standard assert() by default.
 * Disabled completely if UAVCAN_NO_ASSERTIONS is defined.
 */
#ifndef UAVCAN_ASSERT
# if UAVCAN_NO_ASSERTIONS
#  define UAVCAN_ASSERT(x)
# else
#  define UAVCAN_ASSERT(x) assert(x)
# endif
#endif

namespace uavcan
{
/**
 * Memory pool block size.
 *
 * The default of 64 bytes should be OK for any target arch up to AMD64 and any compiler.
 *
 * The library leverages compile-time checks to ensure that all types that are subject to dynamic allocation
 * fit this size, otherwise compilation fails.
 *
 * For platforms featuring small pointer width (16..32 bits), UAVCAN_MEM_POOL_BLOCK_SIZE can often be safely
 * reduced to 56 or even 48 bytes, which leads to lower memory footprint.
 *
 * Note that the pool block size shall be aligned at biggest alignment of the target platform (detected and
 * checked automatically at compile time).
 */
#if UAVCAN_MEM_POOL_BLOCK_SIZE
/// Explicitly specified by the user.
static const unsigned MemPoolBlockSize = UAVCAN_MEM_POOL_BLOCK_SIZE;
#elif defined(__BIGGEST_ALIGNMENT__) && (__BIGGEST_ALIGNMENT__ <= 8)
/// Convenient default for GCC-like compilers - if alignment allows, pool block size can be safely reduced.
static const unsigned MemPoolBlockSize = 56;
#else
/// Safe default that should be OK for any platform.
static const unsigned MemPoolBlockSize = 64;
#endif

#ifdef __BIGGEST_ALIGNMENT__
static const unsigned MemPoolAlignment = __BIGGEST_ALIGNMENT__;
#else
static const unsigned MemPoolAlignment = 16;
#endif

typedef char _alignment_check_for_MEM_POOL_BLOCK_SIZE[((MemPoolBlockSize & (MemPoolAlignment - 1)) == 0) ? 1 : -1];

/**
 * This class that allows to check at compile time whether type T can be allocated using the memory pool.
 * If the check fails, compilation fails.
 */
template <typename T>
struct UAVCAN_EXPORT IsDynamicallyAllocatable
{
    static void check()
    {
        char dummy[(sizeof(T) <= MemPoolBlockSize) ? 1 : -1] = { '0' };
        (void)dummy;
    }
};

/**
 * Float comparison precision.
 * For details refer to:
 *  http://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
 *  https://code.google.com/p/googletest/source/browse/trunk/include/gtest/internal/gtest-internal.h
 */
#ifdef UAVCAN_FLOAT_COMPARISON_EPSILON_MULT
static const unsigned FloatComparisonEpsilonMult = UAVCAN_FLOAT_COMPARISON_EPSILON_MULT;
#else
static const unsigned FloatComparisonEpsilonMult = 10;
#endif

}
