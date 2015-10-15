/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/helpers/heap_based_pool_allocator.hpp>

inline bool atomicCompareAndSwap(void** address, void* expected, void* replacement)
{
    return __sync_bool_compare_and_swap(address, expected, replacement);
}

TEST(ThreadSafeHeapBasedPoolAllocator, Basic)
{
    uavcan::HeapBasedPoolAllocator<uavcan::MemPoolBlockSize, atomicCompareAndSwap> allocator;

    ASSERT_EQ(0, allocator.getNumCachedBlocks());

    void* a = allocator.allocate(10);
    void* b = allocator.allocate(10);
    void* c = allocator.allocate(10);
    void* d = allocator.allocate(10);

    ASSERT_EQ(0, allocator.getNumCachedBlocks());

    allocator.deallocate(a);
    ASSERT_EQ(1, allocator.getNumCachedBlocks());

    allocator.deallocate(b);
    ASSERT_EQ(2, allocator.getNumCachedBlocks());

    allocator.deallocate(c);
    ASSERT_EQ(3, allocator.getNumCachedBlocks());

    a = allocator.allocate(10);
    ASSERT_EQ(2, allocator.getNumCachedBlocks());
    ASSERT_EQ(c, a);

    allocator.deallocate(a);
    ASSERT_EQ(3, allocator.getNumCachedBlocks());

    allocator.shrink();
    ASSERT_EQ(0, allocator.getNumCachedBlocks());

    allocator.deallocate(d);
    ASSERT_EQ(1, allocator.getNumCachedBlocks());

    allocator.shrink();
    ASSERT_EQ(0, allocator.getNumCachedBlocks());
}
