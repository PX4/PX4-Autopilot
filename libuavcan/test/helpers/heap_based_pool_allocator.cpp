/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/helpers/heap_based_pool_allocator.hpp>
#include <malloc.h>

inline bool atomicCompareAndSwap(void** address, void* expected, void* replacement)
{
    return __sync_bool_compare_and_swap(address, expected, replacement);
}

TEST(HeapBasedPoolAllocator, Basic)
{
    std::cout << ">>> HEAP BEFORE:" << std::endl;
    malloc_stats();

    uavcan::HeapBasedPoolAllocator<uavcan::MemPoolBlockSize, atomicCompareAndSwap> al(64);

    ASSERT_EQ(0, al.getNumCachedBlocks());

    ASSERT_EQ(64, al.getNumBlocks());
    al.setReportedNumBlocks(123);
    ASSERT_EQ(123, al.getNumBlocks());

    void* a = al.allocate(10);
    void* b = al.allocate(10);
    void* c = al.allocate(10);
    void* d = al.allocate(10);

    ASSERT_EQ(0, al.getNumCachedBlocks());

    al.deallocate(a);
    ASSERT_EQ(1, al.getNumCachedBlocks());

    al.deallocate(b);
    ASSERT_EQ(2, al.getNumCachedBlocks());

    al.deallocate(c);
    ASSERT_EQ(3, al.getNumCachedBlocks());

    a = al.allocate(10);
    ASSERT_EQ(2, al.getNumCachedBlocks());
    ASSERT_EQ(c, a);

    al.deallocate(a);
    ASSERT_EQ(3, al.getNumCachedBlocks());

    al.shrink();
    ASSERT_EQ(0, al.getNumCachedBlocks());

    al.deallocate(d);
    ASSERT_EQ(1, al.getNumCachedBlocks());

    al.shrink();
    ASSERT_EQ(0, al.getNumCachedBlocks());

    std::cout << ">>> HEAP AFTER:" << std::endl;
    malloc_stats();
}

#if UAVCAN_CPP_VERSION >= UAVCAN_CPP11

#include <thread>

inline bool atomicCompareAndSwapWithRescheduling(void** address, void* expected, void* replacement)
{
    /*
     * Yield is added for testing reasons, making CAS failure more likely
     */
    if ((float(std::rand()) / float(RAND_MAX)) < 0.05)
    {
        std::this_thread::yield();
    }
    return __sync_bool_compare_and_swap(address, expected, replacement);
}

TEST(HeapBasedPoolAllocator, Concurrency)
{
    std::cout << ">>> HEAP BEFORE:" << std::endl;
    malloc_stats();

    uavcan::HeapBasedPoolAllocator<uavcan::MemPoolBlockSize, atomicCompareAndSwapWithRescheduling> al(1);

    volatile bool terminate = false;

    /*
     * Starting the testing threads
     */
    std::thread threads[3];

    for (auto& x : threads)
    {
        x = std::thread([&al, &terminate]()
        {
            while (!terminate)
            {
                auto a = al.allocate(1);
                auto b = al.allocate(1);
                auto c = al.allocate(1);
                al.deallocate(al.allocate(1));
                al.deallocate(a);
                al.deallocate(b);
                al.deallocate(c);
            }
        });
    }

    /*
     * Running the threads for some time, then terminating
     */
    std::this_thread::sleep_for(std::chrono::seconds(1));

    terminate = true;
    std::cout << "Terminating workers..." << std::endl;

    for (auto& x : threads)
    {
        x.join();
    }
    std::cout << "All workers joined" << std::endl;

    /*
     * Now, there must not be any leaked memory, because the worker threads deallocate everything before completion.
     */
    //std::cout << "Cached blocks: " << al.getNumCachedBlocks() << std::endl;

    std::cout << ">>> HEAP BEFORE SHRINK:" << std::endl;
    malloc_stats();

    al.shrink();

    std::cout << ">>> HEAP AFTER SHRINK:" << std::endl;
    malloc_stats();
}

#endif
