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
    uint16_t reported_num_blocks_;

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
     */
    HeapBasedPoolAllocator(uint16_t reported_num_blocks) :
        cache_(NULL),
        reported_num_blocks_(reported_num_blocks)
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
     * Heap-based pool is virutally infinite in size, so this method just returns some pre-defined value.
     */
    virtual uint16_t getNumBlocks() const { return reported_num_blocks_; }
    void setReportedNumBlocks(uint16_t x) { reported_num_blocks_ = x; }

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
