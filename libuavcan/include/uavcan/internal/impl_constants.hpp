/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/internal/static_assert.hpp>

namespace uavcan
{

/**
 * Should be OK for any target arch up to AMD64 and any compiler.
 * The library leverages compile-time checks to ensure that all types that are subject to dynamic allocation
 * fit this size; otherwise compilation fails.
 */
#if UAVCAN_MEM_POOL_BLOCK_SIZE
enum { MEM_POOL_BLOCK_SIZE = UAVCAN_MEM_POOL_BLOCK_SIZE };
#else
enum { MEM_POOL_BLOCK_SIZE = 48 };
#endif

enum { MEM_POOL_ALIGNMENT = 8 };

typedef char _alignment_check_for_MEM_POOL_BLOCK_SIZE[((MEM_POOL_BLOCK_SIZE & (MEM_POOL_ALIGNMENT - 1)) == 0) ? 1 : -1];

template <typename T>
struct IsDynamicallyAllocatable
{
    static void check() { StaticAssert<sizeof(T) <= MEM_POOL_BLOCK_SIZE>::check(); }
};

}
