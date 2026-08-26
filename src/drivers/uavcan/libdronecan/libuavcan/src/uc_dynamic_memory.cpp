/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/dynamic_memory.hpp>

namespace uavcan
{
/*
 * LimitedPoolAllocator
 */
void* LimitedPoolAllocator::allocate(std::size_t size)
{
    if (used_blocks_ >= max_blocks_)
    {
        return UAVCAN_NULLPTR;
    }

    /*
     * Counting the block only once the underlying allocator has actually produced one. Counting the
     * attempt instead spends quota on a block that deallocate() will never return, so a pool that is
     * momentarily empty permanently shrinks every queue that asked it for memory while it was.
     */
    void* const praw = allocator_.allocate(size);

    if (praw != UAVCAN_NULLPTR)
    {
        used_blocks_++;

        if (used_blocks_ > peak_used_blocks_)
        {
            peak_used_blocks_ = used_blocks_;
        }
    }

    return praw;
}

void LimitedPoolAllocator::deallocate(const void* ptr)
{
    allocator_.deallocate(ptr);

    UAVCAN_ASSERT(used_blocks_ > 0);
    if (used_blocks_ > 0)
    {
        used_blocks_--;
    }
}

uint16_t LimitedPoolAllocator::getBlockCapacity() const
{
    return min(max_blocks_, allocator_.getBlockCapacity());
}

}
