/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <uavcan/internal/transport/transfer_buffer.hpp>

namespace uavcan
{
/*
 * TransferBufferManagerKey
 */
std::string TransferBufferManagerKey::toString() const
{
    char buf[24];
    std::snprintf(buf, sizeof(buf), "nid:%i tt:%i", int(node_id_), int(transfer_type_));
    return std::string(buf);
}

/*
 * DynamicTransferBuffer::Block
 */
DynamicTransferBuffer::Block* DynamicTransferBuffer::Block::instantiate(IAllocator* allocator)
{
    assert(allocator);
    void* const praw = allocator->allocate(sizeof(Block));
    if (praw == NULL)
        return NULL;
    return new (praw) Block;
}

void DynamicTransferBuffer::Block::destroy(Block*& obj, IAllocator* allocator)
{
    assert(allocator);
    if (obj != NULL)
    {
        obj->~Block();
        allocator->deallocate(obj);
        obj = NULL;
    }
}

void DynamicTransferBuffer::Block::read(uint8_t*& outptr, unsigned int target_offset,
                                        unsigned int& total_offset, unsigned int& left_to_read)
{
    assert(outptr);
    for (int i = 0; (i < Block::SIZE) && (left_to_read > 0); i++, total_offset++)
    {
        if (total_offset >= target_offset)
        {
            *outptr++ = data[i];
            left_to_read--;
        }
    }
}

void DynamicTransferBuffer::Block::write(const uint8_t*& inptr, unsigned int target_offset,
                                         unsigned int& total_offset, unsigned int& left_to_write)
{
    assert(inptr);
    for (int i = 0; (i < Block::SIZE) && (left_to_write > 0); i++, total_offset++)
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
DynamicTransferBuffer* DynamicTransferBuffer::instantiate(IAllocator* allocator, unsigned int max_size)
{
    assert(allocator);
    void* const praw = allocator->allocate(sizeof(DynamicTransferBuffer));
    if (praw == NULL)
        return NULL;
    return new (praw) DynamicTransferBuffer(allocator, max_size);
}

void DynamicTransferBuffer::destroy(DynamicTransferBuffer*& obj, IAllocator* allocator)
{
    assert(allocator);
    if (obj != NULL)
    {
        obj->~DynamicTransferBuffer();
        allocator->deallocate(obj);
        obj = NULL;
    }
}

void DynamicTransferBuffer::resetImpl()
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

int DynamicTransferBuffer::read(unsigned int offset, uint8_t* data, unsigned int len) const
{
    if (!data)
    {
        assert(0);
        return -1;
    }
    if (offset >= max_write_pos_)
        return 0;
    if ((offset + len) > max_write_pos_)
        len = max_write_pos_ - offset;
    assert((offset + len) <= max_write_pos_);

    // This shall be optimized.
    unsigned int total_offset = 0, left_to_read = len;
    uint8_t* outptr = data;
    Block* p = blocks_.get();
    while (p)
    {
        p->read(outptr, offset, total_offset, left_to_read);
        if (left_to_read == 0)
            break;
        p = p->getNextListNode();
    }

    assert(left_to_read == 0);
    return len;
}

int DynamicTransferBuffer::write(unsigned int offset, const uint8_t* data, unsigned int len)
{
    if (!data)
    {
        assert(0);
        return -1;
    }

    if (offset >= max_size_)
        return 0;
    if ((offset + len) > max_size_)
        len = max_size_ - offset;
    assert((offset + len) <= max_size_);

    unsigned int total_offset = 0, left_to_write = len;
    const uint8_t* inptr = data;
    Block* p = blocks_.get(), *last_written_block = NULL;

    // First we need to write the part that is already allocated
    while (p)
    {
        last_written_block = p;
        p->write(inptr, offset, total_offset, left_to_write);
        if (left_to_write == 0)
            break;
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
            break;                        // We're in deep shit.

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
