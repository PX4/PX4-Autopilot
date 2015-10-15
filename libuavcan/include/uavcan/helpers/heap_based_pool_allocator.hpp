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
 * All operations are thread-safe, the safety is obtained through the standard atomic compare-and-swap operator (CAS).
 * If thread safety is not required, the CAS operator can be replaced with a dummy version that simply performs the
 * assignment without any checks.
 * If thread safety is required, but the hardware does not support atomic exchange operations, CAS can be emulated
 * with a mutex or a critical section roughly as follows:
 *
 *     bool softwareAtomicCompareAndSwap(void** address, void* expected, void* replacement)
 *     {
 *         RaiiSynchronizationPrimitive lock;
 *         if (*address != expected)
 *         {
 *             return false;
 *         }
 *         *address = replacement;
 *         return true;
 *     }
 */
template <std::size_t BlockSize,
          bool (*AtomicCompareAndSwap)(void** address, void* expected, void* replacement)>
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

    Node* popCache()
    {
        // http://www.ibm.com/developerworks/aix/library/au-multithreaded_structures2/
        for (;;)
        {
            Node* volatile const result = cache_;
            if (result == NULL)
            {
                break;
            }
            if ((cache_ != NULL) &&
                AtomicCompareAndSwap(reinterpret_cast<void**>(const_cast<Node**>(&cache_)), result, result->next))
            {
                return result;
            }
        }
        return NULL;
    }

public:
    /**
     * The allocator initializes with empty cache, so first allocations will be served from heap.
     */
    HeapBasedPoolAllocator() : cache_(NULL) { }

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
        if (ptr == NULL)
        {
            return;
        }

        Node* volatile const n = static_cast<Node*>(const_cast<void*>(ptr));
        do
        {
            n->next = cache_;
        }
        while (!AtomicCompareAndSwap(reinterpret_cast<void**>(const_cast<Node**>(&cache_)), n->next, n));
    }

    /**
     * Heap-based pool is virutally infinite in size, so this method just returns maximum possible number of blocks.
     */
    virtual uint16_t getNumBlocks() const { return NumericTraits<uint16_t>::max(); }

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
