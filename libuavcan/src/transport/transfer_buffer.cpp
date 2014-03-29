/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <uavcan/transport/transfer_buffer.hpp>

namespace uavcan
{
/*
 * TransferBufferManagerKey
 */
std::string TransferBufferManagerKey::toString() const
{
    char buf[24];
    std::snprintf(buf, sizeof(buf), "nid=%i tt=%i", int(node_id_.get()), int(transfer_type_));
    return std::string(buf);
}

/*
 * DynamicTransferBuffer::Block
 */
DynamicTransferBufferManagerEntry::Block* DynamicTransferBufferManagerEntry::Block::instantiate(IAllocator& allocator)
{
    void* const praw = allocator.allocate(sizeof(Block));
    if (praw == NULL)
    {
        return NULL;
    }
    return new (praw) Block;
}

void DynamicTransferBufferManagerEntry::Block::destroy(Block*& obj, IAllocator& allocator)
{
    if (obj != NULL)
    {
        obj->~Block();
        allocator.deallocate(obj);
        obj = NULL;
    }
}

void DynamicTransferBufferManagerEntry::Block::read(uint8_t*& outptr, unsigned target_offset,
                                                    unsigned& total_offset, unsigned& left_to_read)
{
    assert(outptr);
    for (unsigned i = 0; (i < Block::Size) && (left_to_read > 0); i++, total_offset++)
    {
        if (total_offset >= target_offset)
        {
            *outptr++ = data[i];
            left_to_read--;
        }
    }
}

void DynamicTransferBufferManagerEntry::Block::write(const uint8_t*& inptr, unsigned target_offset,
                                                     unsigned& total_offset, unsigned& left_to_write)
{
    assert(inptr);
    for (unsigned i = 0; (i < Block::Size) && (left_to_write > 0); i++, total_offset++)
    {
        if (total_offset >= target_offset)
        {
            data[i] = *inptr++;
            left_to_write--;
        }
    }
}

/*
 * DynamicTransferBuffer
 */
DynamicTransferBufferManagerEntry* DynamicTransferBufferManagerEntry::instantiate(IAllocator& allocator,
                                                                                  unsigned max_size)
{
    void* const praw = allocator.allocate(sizeof(DynamicTransferBufferManagerEntry));
    if (praw == NULL)
    {
        return NULL;
    }
    return new (praw) DynamicTransferBufferManagerEntry(allocator, max_size);
}

void DynamicTransferBufferManagerEntry::destroy(DynamicTransferBufferManagerEntry*& obj, IAllocator& allocator)
{
    if (obj != NULL)
    {
        obj->~DynamicTransferBufferManagerEntry();
        allocator.deallocate(obj);
        obj = NULL;
    }
}

void DynamicTransferBufferManagerEntry::resetImpl()
{
    max_write_pos_ = 0;
    Block* p = blocks_.get();
    while (p)
    {
        Block* const next = p->getNextListNode();
        blocks_.remove(p);
        Block::destroy(p, allocator_);
        p = next;
    }
}

int DynamicTransferBufferManagerEntry::read(unsigned offset, uint8_t* data, unsigned len) const
{
    if (!data)
    {
        assert(0);
        return -ErrInvalidParam;
    }
    if (offset >= max_write_pos_)
    {
        return 0;
    }
    if ((offset + len) > max_write_pos_)
    {
        len = max_write_pos_ - offset;
    }
    assert((offset + len) <= max_write_pos_);

    // This shall be optimized.
    unsigned total_offset = 0;
    unsigned left_to_read = len;
    uint8_t* outptr = data;
    Block* p = blocks_.get();
    while (p)
    {
        p->read(outptr, offset, total_offset, left_to_read);
        if (left_to_read == 0)
        {
            break;
        }
        p = p->getNextListNode();
    }

    assert(left_to_read == 0);
    return len;
}

int DynamicTransferBufferManagerEntry::write(unsigned offset, const uint8_t* data, unsigned len)
{
    if (!data)
    {
        assert(0);
        return -ErrInvalidParam;
    }

    if (offset >= max_size_)
    {
        return 0;
    }
    if ((offset + len) > max_size_)
    {
        len = max_size_ - offset;
    }
    assert((offset + len) <= max_size_);

    unsigned total_offset = 0;
    unsigned left_to_write = len;
    const uint8_t* inptr = data;
    Block* p = blocks_.get();
    Block* last_written_block = NULL;

    // First we need to write the part that is already allocated
    while (p)
    {
        last_written_block = p;
        p->write(inptr, offset, total_offset, left_to_write);
        if (left_to_write == 0)
        {
            break;
        }
        p = p->getNextListNode();
    }

    // Then we need to append new chunks until all data is written
    while (left_to_write > 0)
    {
        // cppcheck-suppress nullPointer
        assert(p == NULL);

        // Allocating the chunk
        Block* new_block = Block::instantiate(allocator_);
        if (new_block == NULL)
        {
            break;                        // We're in deep shit.
        }
        // Appending the chain with the new block
        if (last_written_block != NULL)
        {
            assert(last_written_block->getNextListNode() == NULL);  // Because it is last in the chain
            last_written_block->setNextListNode(new_block);
            new_block->setNextListNode(NULL);
        }
        else
        {
            blocks_.insert(new_block);
        }
        last_written_block = new_block;

        // Writing the data
        new_block->write(inptr, offset, total_offset, left_to_write);
    }

    const int actually_written = len - left_to_write;
    max_write_pos_ = std::max(offset + actually_written, max_write_pos_);
    return actually_written;
}

}
