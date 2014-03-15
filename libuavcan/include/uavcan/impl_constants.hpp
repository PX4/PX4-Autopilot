/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/util/compile_time.hpp>


// TODO: Use git short hash as build id
#define UAVCAN_VERSION_MAJOR    0
#define UAVCAN_VERSION_MINOR    1
#define UAVCAN_VERSION_BUILD    0


namespace uavcan
{

/**
 * UAVCAN can be explicitly told to ignore exceptions, or it can be detected automatically.
 */
#ifndef UAVCAN_EXCEPTIONS
# if defined(__EXCEPTIONS) || defined(_HAS_EXCEPTIONS)
#  define UAVCAN_EXCEPTIONS 1
# endif
#endif

/**
 * Struct layout control.
 * Define UAVCAN_PACK_STRUCTS = 1 to reduce memory usage.
 */
#ifndef UAVCAN_PACKED_BEGIN
# define UAVCAN_PACKED_BEGIN _Pragma("pack(push, 1)")
#endif
#ifndef UAVCAN_PACKED_END
# define UAVCAN_PACKED_END _Pragma("pack(pop)")
#endif

/**
 * Should be OK for any target arch up to AMD64 and any compiler.
 * The library leverages compile-time checks to ensure that all types that are subject to dynamic allocation
 * fit this size; otherwise compilation fails.
 */
#if UAVCAN_MEM_POOL_BLOCK_SIZE
enum { MemPoolBlockSize = UAVCAN_MEM_POOL_BLOCK_SIZE };
#else
enum { MemPoolBlockSize = 32 + sizeof(void*) * 2 };
#endif

enum { MemPoolAlignment = 8 };

typedef char _alignment_check_for_MEM_POOL_BLOCK_SIZE[((MemPoolBlockSize & (MemPoolAlignment - 1)) == 0) ? 1 : -1];

template <typename T>
struct IsDynamicallyAllocatable
{
    static void check() { StaticAssert<sizeof(T) <= MemPoolBlockSize>::check(); }
};

}
