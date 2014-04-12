/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/dynamic_memory.hpp>

TEST(DynamicMemory, Basic)
{
    uavcan::PoolAllocator<128, 32> pool32;
    uavcan::PoolAllocator<256, 64> pool64;
    uavcan::PoolAllocator<512, 128> pool128;

    EXPECT_EQ(4, pool32.getNumFreeBlocks());
    EXPECT_EQ(4, pool64.getNumFreeBlocks());
    EXPECT_EQ(4, pool128.getNumFreeBlocks());

    uavcan::PoolManager<2> poolmgr;
    EXPECT_TRUE(poolmgr.addPool(&pool64));   // Order of insertion shall not matter
    EXPECT_TRUE(poolmgr.addPool(&pool32));
    EXPECT_FALSE(poolmgr.addPool(&pool128));

    EXPECT_EQ(4 * 2, poolmgr.getNumBlocks());

    const void* ptr1 = poolmgr.allocate(16);
    EXPECT_TRUE(ptr1);

    const void* ptr2 = poolmgr.allocate(32);
    EXPECT_TRUE(ptr2);

    const void* ptr3 = poolmgr.allocate(64);
    EXPECT_TRUE(ptr3);

    EXPECT_FALSE(poolmgr.allocate(120));

    EXPECT_EQ(2, pool32.getNumUsedBlocks());
    EXPECT_EQ(1, pool64.getNumUsedBlocks());
    EXPECT_EQ(0, pool128.getNumUsedBlocks());

    poolmgr.deallocate(ptr1);
    EXPECT_EQ(1, pool32.getNumUsedBlocks());
    poolmgr.deallocate(ptr2);
    EXPECT_EQ(0, pool32.getNumUsedBlocks());
    EXPECT_EQ(1, pool64.getNumUsedBlocks());
    poolmgr.deallocate(ptr3);
    EXPECT_EQ(0, pool64.getNumUsedBlocks());
    EXPECT_EQ(0, pool128.getNumUsedBlocks());
}

TEST(DynamicMemory, OutOfMemory)
{
    uavcan::PoolAllocator<64, 32> pool32;
    uavcan::PoolAllocator<128, 64> pool64;

    EXPECT_EQ(2, pool32.getNumFreeBlocks());
    EXPECT_EQ(2, pool64.getNumFreeBlocks());

    uavcan::PoolManager<4> poolmgr;
    EXPECT_TRUE(poolmgr.addPool(&pool64));
    EXPECT_TRUE(poolmgr.addPool(&pool32));

    const void* ptr1 = poolmgr.allocate(32);
    EXPECT_TRUE(ptr1);
    EXPECT_TRUE(pool32.isInPool(ptr1));
    EXPECT_FALSE(pool64.isInPool(ptr1));

    const void* ptr2 = poolmgr.allocate(32);
    EXPECT_TRUE(ptr2);
    EXPECT_TRUE(pool32.isInPool(ptr2));
    EXPECT_FALSE(pool64.isInPool(ptr2));

    const void* ptr3 = poolmgr.allocate(32);
    EXPECT_TRUE(ptr3);
    EXPECT_FALSE(pool32.isInPool(ptr3));
    EXPECT_TRUE(pool64.isInPool(ptr3));        // One block went to the next pool

    EXPECT_EQ(2, pool32.getNumUsedBlocks());
    EXPECT_EQ(1, pool64.getNumUsedBlocks());

    poolmgr.deallocate(ptr2);
    EXPECT_EQ(1, pool32.getNumUsedBlocks());
    EXPECT_EQ(1, pool64.getNumUsedBlocks());

    const void* ptr4 = poolmgr.allocate(64);
    EXPECT_TRUE(ptr4);
    EXPECT_EQ(1, pool32.getNumUsedBlocks());
    EXPECT_EQ(2, pool64.getNumUsedBlocks());   // Top pool is 100% used

    EXPECT_FALSE(poolmgr.allocate(64));        // No free blocks left --> NULL
    EXPECT_EQ(1, pool32.getNumUsedBlocks());
    EXPECT_EQ(2, pool64.getNumUsedBlocks());

    poolmgr.deallocate(ptr3);                  // This was small chunk allocated in big pool
    EXPECT_EQ(1, pool32.getNumUsedBlocks());
    EXPECT_EQ(1, pool64.getNumUsedBlocks());   // Make sure it was properly deallocated
}

TEST(DynamicMemory, LimitedPoolAllocator)
{
    uavcan::PoolAllocator<128, 32> pool32;
    uavcan::LimitedPoolAllocator lim(pool32, 2);

    EXPECT_EQ(2, lim.getNumBlocks());

    const void* ptr1 = lim.allocate(1);
    const void* ptr2 = lim.allocate(1);
    const void* ptr3 = lim.allocate(1);

    EXPECT_TRUE(ptr1);
    EXPECT_TRUE(ptr2);
    EXPECT_FALSE(ptr3);

    lim.deallocate(ptr2);
    const void* ptr4 = lim.allocate(1);
    lim.deallocate(ptr1);
    const void* ptr5 = lim.allocate(1);
    const void* ptr6 = lim.allocate(1);

    EXPECT_TRUE(ptr4);
    EXPECT_TRUE(ptr5);
    EXPECT_FALSE(ptr6);
}
