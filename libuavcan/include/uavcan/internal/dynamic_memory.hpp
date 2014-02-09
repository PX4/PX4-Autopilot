/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <limits>
#include <stdint.h>
#include <uavcan/internal/util.hpp>

namespace uavcan
{
/**
 * This interface is used by other library components that need dynamic memory.
 */
class IAllocator
{
public:
    virtual ~IAllocator() { }
    virtual void* allocate(std::size_t size) = 0;
    virtual void deallocate(const void* ptr) = 0;
};


class IPoolAllocator : public IAllocator
{
public:
    virtual bool isInPool(const void* ptr) const = 0;
    virtual std::size_t getBlockSize() const = 0;
};


template <int MAX_POOLS>
class PoolManager : public IAllocator, Noncopyable
{
    IPoolAllocator* pools_[MAX_POOLS];

    static bool sortComparePoolAllocators(const IPoolAllocator* a, const IPoolAllocator* b)
    {
        const std::size_t a_size = a ? a->getBlockSize() : std::numeric_limits<std::size_t>::max();
        const std::size_t b_size = b ? b->getBlockSize() : std::numeric_limits<std::size_t>::max();
        return a_size < b_size;
    }

public:
    PoolManager()
    {
        std::memset(pools_, 0, sizeof(pools_));
    }

    bool addPool(IPoolAllocator* pool)
    {
        assert(pool);
        bool retval = false;
        for (int i = 0; i < MAX_POOLS; i++)
        {
            assert(pools_[i] != pool);
            if (pools_[i] == NULL || pools_[i] == pool)
            {
                pools_[i] = pool;
                retval = true;
                break;
            }
        }
        // We need to keep the pools in order, so that smallest blocks go first
        std::sort(pools_, pools_ + MAX_POOLS, &PoolManager::sortComparePoolAllocators);
        return retval;
    }

    void* allocate(std::size_t size)
    {
        for (int i = 0; i < MAX_POOLS; i++)
        {
            if (pools_[i] == NULL)
                break;
            void* const pmem = pools_[i]->allocate(size);
            if (pmem != NULL)
                return pmem;
        }
        return NULL;
    }

    void deallocate(const void* ptr)
    {
        for (int i = 0; i < MAX_POOLS; i++)
        {
            if (pools_[i] == NULL)
            {
                assert(0);
                break;
            }
            if (pools_[i]->isInPool(ptr))
            {
                pools_[i]->deallocate(ptr);
                break;
            }
        }
    }
};


template <std::size_t POOL_SIZE, std::size_t BLOCK_SIZE>
class PoolAllocator : public IPoolAllocator, Noncopyable
{
    union Node
    {
        uint8_t data[BLOCK_SIZE];
        Node* next;
    };

    Node* free_list_;
    uint8_t pool_[POOL_SIZE] __attribute__((aligned(16)));  // TODO: compiler-independent alignment

    // Noncopyable
    PoolAllocator(const PoolAllocator&);
    PoolAllocator& operator=(const PoolAllocator&);

public:
    static const int NUM_BLOCKS = int(POOL_SIZE / BLOCK_SIZE);

    PoolAllocator()
    : free_list_(reinterpret_cast<Node*>(pool_)) // TODO: alignment
    {
        memset(pool_, 0, POOL_SIZE);
        for (int i = 0; i < NUM_BLOCKS - 1; i++)
            free_list_[i].next = free_list_ + i + 1;
        free_list_[NUM_BLOCKS - 1].next = NULL;
    }

    void* allocate(std::size_t size)
    {
        if (free_list_ == NULL || size > BLOCK_SIZE)
            return NULL;
        void* pmem = free_list_;
        free_list_ = free_list_->next;
        return pmem;
    }

    void deallocate(const void* ptr)
    {
        if (ptr == NULL)
            return;
        Node* p = static_cast<Node*>(const_cast<void*>(ptr));
#if DEBUG || UAVCAN_DEBUG
        std::memset(p, 0, sizeof(Node));
#endif
        p->next = free_list_;
        free_list_ = p;
    }

    bool isInPool(const void* ptr) const
    {
        return
            ptr >= pool_ &&
            ptr < (pool_ + POOL_SIZE);
    }

    std::size_t getBlockSize() const { return BLOCK_SIZE; }

    int getNumFreeBlocks() const
    {
        int num = 0;
        Node* p = free_list_;
        while (p)
        {
            num++;
            assert(num <= NUM_BLOCKS);
            p = p->next;
        }
        return num;
    }

    int getNumUsedBlocks() const
    {
        return NUM_BLOCKS - getNumFreeBlocks();
    }
};

}
