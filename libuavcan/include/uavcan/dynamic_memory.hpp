/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <limits>
#include <uavcan/stdint.hpp>
#include <uavcan/util/compile_time.hpp>
#include <uavcan/impl_constants.hpp>

namespace uavcan
{
/**
 * This interface is used by other library components that need dynamic memory.
 */
class UAVCAN_EXPORT IAllocator
{
public:
    virtual ~IAllocator() { }
    virtual void* allocate(std::size_t size) = 0;
    virtual void deallocate(const void* ptr) = 0;
};


class UAVCAN_EXPORT IPoolAllocator : public IAllocator
{
public:
    virtual bool isInPool(const void* ptr) const = 0;
    virtual std::size_t getBlockSize() const = 0;
};


template <int MaxPools>
class UAVCAN_EXPORT PoolManager : public IAllocator, Noncopyable
{
    IPoolAllocator* pools_[MaxPools];

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

    bool addPool(IPoolAllocator* pool);

    void* allocate(std::size_t size);

    void deallocate(const void* ptr);
};


template <std::size_t PoolSize, std::size_t BlockSize>
class UAVCAN_EXPORT PoolAllocator : public IPoolAllocator, Noncopyable
{
    union Node
    {
        uint8_t data[BlockSize];
        Node* next;
    };

    Node* free_list_;
    union
    {
         uint8_t bytes[PoolSize];
         long double _aligner1;
         long long _aligner2;
         Node _aligner3;
    } pool_;

public:
    static const int NumBlocks = int(PoolSize / BlockSize);

    PoolAllocator();

    void* allocate(std::size_t size);

    void deallocate(const void* ptr);

    bool isInPool(const void* ptr) const;

    std::size_t getBlockSize() const { return BlockSize; }

    int getNumFreeBlocks() const;

    int getNumUsedBlocks() const { return NumBlocks - getNumFreeBlocks(); }
};


class LimitedPoolAllocator : public IAllocator
{
    IAllocator& allocator_;
    const std::size_t max_blocks_;
    std::size_t used_blocks_;

public:
    LimitedPoolAllocator(IAllocator& allocator, std::size_t max_blocks)
        : allocator_(allocator)
        , max_blocks_(max_blocks)
        , used_blocks_(0)
    {
        assert(max_blocks_ > 0);
    }

    void* allocate(std::size_t size);
    void deallocate(const void* ptr);
};

// ----------------------------------------------------------------------------

/*
 * PoolManager<>
 */
template <int MaxPools>
bool PoolManager<MaxPools>::addPool(IPoolAllocator* pool)
{
    assert(pool);
    bool retval = false;
    for (int i = 0; i < MaxPools; i++)
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
    std::sort(pools_, pools_ + MaxPools, &PoolManager::sortComparePoolAllocators);
    return retval;
}

template <int MaxPools>
void* PoolManager<MaxPools>::allocate(std::size_t size)
{
    for (int i = 0; i < MaxPools; i++)
    {
        if (pools_[i] == NULL)
        {
            break;
        }
        void* const pmem = pools_[i]->allocate(size);
        if (pmem != NULL)
        {
            return pmem;
        }
    }
    return NULL;
}

template <int MaxPools>
void PoolManager<MaxPools>::deallocate(const void* ptr)
{
    for (int i = 0; i < MaxPools; i++)
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

/*
 * PoolAllocator<>
 */
template <std::size_t PoolSize, std::size_t BlockSize>
PoolAllocator<PoolSize, BlockSize>::PoolAllocator()
    : free_list_(reinterpret_cast<Node*>(pool_.bytes))
{
    memset(pool_.bytes, 0, PoolSize);
    for (int i = 0; i < NumBlocks - 1; i++)
    {
        free_list_[i].next = free_list_ + i + 1;
    }
    free_list_[NumBlocks - 1].next = NULL;
}

template <std::size_t PoolSize, std::size_t BlockSize>
void* PoolAllocator<PoolSize, BlockSize>::allocate(std::size_t size)
{
    if (free_list_ == NULL || size > BlockSize)
    {
        return NULL;
    }
    void* pmem = free_list_;
    free_list_ = free_list_->next;
    return pmem;
}

template <std::size_t PoolSize, std::size_t BlockSize>
void PoolAllocator<PoolSize, BlockSize>::deallocate(const void* ptr)
{
    if (ptr == NULL)
    {
        return;
    }
    Node* p = static_cast<Node*>(const_cast<void*>(ptr));
#if DEBUG || UAVCAN_DEBUG
    std::memset(p, 0, sizeof(Node));
#endif
    p->next = free_list_;
    free_list_ = p;
}

template <std::size_t PoolSize, std::size_t BlockSize>
bool PoolAllocator<PoolSize, BlockSize>::isInPool(const void* ptr) const
{
    return ptr >= pool_.bytes &&
           ptr < (pool_.bytes + PoolSize);
}

template <std::size_t PoolSize, std::size_t BlockSize>
int PoolAllocator<PoolSize, BlockSize>::getNumFreeBlocks() const
{
    int num = 0;
    Node* p = free_list_;
    while (p)
    {
        num++;
        assert(num <= NumBlocks);
        p = p->next;
    }
    return num;
}

}
