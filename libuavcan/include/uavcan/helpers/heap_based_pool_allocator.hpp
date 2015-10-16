/*
 * Copyright (C) 2015 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_HELPERS_HEAP_BASED_POOL_ALLOCATOR_HPP_INCLUDED
#define UAVCAN_HELPERS_HEAP_BASED_POOL_ALLOCATOR_HPP_INCLUDED

#include <cstdlib>
#include <uavcan/dynamic_memory.hpp>

namespace uavcan
{
/**
 * A special-purpose implementation of a pool allocator that keeps the pool in the heap using malloc()/free().
 * The pool grows dynamically, ad-hoc, thus using as little memory as possible.
 *
 * Allocated blocks will not be freed back automatically, but there are two ways to force their deallocation:
 *  - Call @ref shrink() - this method frees all blocks that are unused at the moment.
 *  - Destroy the object - the desctructor calls @ref shrink().
 *
 * TODO: notes on thread-safety.
 */
template <std::size_t BlockSize, typename RaiiSynchronizer = char>
class UAVCAN_EXPORT HeapBasedPoolAllocator : public IPoolAllocator, Noncopyable
{
    union Node
    {
        Node* next;
    private:
        uint8_t data[BlockSize];
        long double _aligner1;
        long long _aligner2;
    };

    Node* volatile cache_;

    const uint16_t capacity_soft_limit_;
    const uint16_t capacity_hard_limit_;

    Node* popCache()
    {
        RaiiSynchronizer lock;
        (void)lock;
        Node* const p = cache_;
        if (p != NULL)
        {
            cache_ = cache_->next;
        }
        return p;
    }

    void pushCache(Node* node)
    {
        RaiiSynchronizer lock;
        (void)lock;
        node->next = cache_;
        cache_ = node;
    }

public:
    /**
     * The allocator initializes with empty cache, so first allocations will be served from heap.
     *
     * @param block_capacity_soft_limit     Block capacity that will be reported via @ref getBlockCapacity().
     *
     * @param block_capacity_hard_limit     Real block capacity limit; the number of allocated blocks will never
     *                                      exceed this value. Hard limit should be higher than soft limit.
     *                                      Default value is two times the soft limit.
     */
    HeapBasedPoolAllocator(uint16_t block_capacity_soft_limit,
                           uint16_t block_capacity_hard_limit = 0) :
        cache_(NULL),
        capacity_soft_limit_(block_capacity_soft_limit),
        capacity_hard_limit_((block_capacity_hard_limit > 0) ? block_capacity_hard_limit :
                             static_cast<uint16_t>(min(static_cast<uint32_t>(block_capacity_soft_limit) * 2U,
                                                       static_cast<uint32_t>(NumericTraits<uint16_t>::max()))))
    { }

    /**
     * The destructor de-allocates all blocks that are currently in the cache.
     * BLOCKS THAT ARE CURRENTLY HELD BY THE APPLICATION WILL LEAK.
     */
    ~HeapBasedPoolAllocator() { shrink(); }

    /**
     * Takes a block from the cache, unless it's empty.
     * In the latter case, allocates a new block in the heap.
     */
    virtual void* allocate(std::size_t size)
    {
        if (size > BlockSize)
        {
            return NULL;
        }
        if (Node* n = popCache())
        {
            return n;
        }
        else
        {
            return std::malloc(sizeof(Node));
        }
    }

    /**
     * Puts the block back to cache.
     * The block will not be free()d automatically; see @ref shrink().
     */
    virtual void deallocate(const void* ptr)
    {
        if (ptr != NULL)
        {
            pushCache(static_cast<Node*>(const_cast<void*>(ptr)));
        }
    }

    /**
     * The soft limit.
     */
    virtual uint16_t getBlockCapacity() const { return capacity_soft_limit_; }

    /**
     * The hard limit.
     */
    uint16_t getBlockCapacityHardLimit() const { return capacity_hard_limit_; }

    /**
     * Frees all blocks that are not in use at the moment.
     */
    void shrink()
    {
        while (Node* p = popCache())
        {
            std::free(p);
        }
    }

    /**
     * This function naively counts the number of cached blocks.
     * It is not thread-safe and is mostly designed for testing and debugging purposes.
     * Don't use it in production code.
     */
    unsigned getNumCachedBlocks()
    {
        unsigned out = 0;
        Node* p = cache_;
        while (p != NULL)
        {
            out++;
            p = p->next;
        }
        return out;
    }
};

}

#endif
