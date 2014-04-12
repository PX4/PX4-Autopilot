/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <cstdlib>
#include <uavcan/transport/transfer_buffer.hpp>

namespace uavcan
{
/*
 * TransferBufferManagerKey
 */
#if UAVCAN_TOSTRING
std::string TransferBufferManagerKey::toString() const
{
    using namespace std; // For snprintf()
    char buf[24];
    snprintf(buf, sizeof(buf), "nid=%i tt=%i", int(node_id_.get()), int(transfer_type_));
    return std::string(buf);
}
#endif

/*
 * DynamicTransferBuffer::Block
 */
DynamicTransferBufferManagerEntry::Block* DynamicTransferBufferManagerEntry::Block::instantiate(IPoolAllocator& allocator)
{
    void* const praw = allocator.allocate(sizeof(Block));
    if (praw == NULL)
    {
        return NULL;
    }
    return new (praw) Block;
}

void DynamicTransferBufferManagerEntry::Block::destroy(Block*& obj, IPoolAllocator& allocator)
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
DynamicTransferBufferManagerEntry* DynamicTransferBufferManagerEntry::instantiate(IPoolAllocator& allocator,
                                                                                  uint16_t max_size)
{
    void* const praw = allocator.allocate(sizeof(DynamicTransferBufferManagerEntry));
    if (praw == NULL)
    {
        return NULL;
    }
    return new (praw) DynamicTransferBufferManagerEntry(allocator, max_size);
}

void DynamicTransferBufferManagerEntry::destroy(DynamicTransferBufferManagerEntry*& obj, IPoolAllocator& allocator)
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
    max_write_pos_ = std::max(offset + actually_written, unsigned(max_write_pos_));
    return actually_written;
}

/*
 * StaticTransferBufferImpl
 */
int StaticTransferBufferImpl::read(unsigned offset, uint8_t* data, unsigned len) const
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
    std::copy(data_ + offset, data_ + offset + len, data);
    return len;
}

int StaticTransferBufferImpl::write(unsigned offset, const uint8_t* data, unsigned len)
{
    if (!data)
    {
        assert(0);
        return -ErrInvalidParam;
    }
    if (offset >= size_)
    {
        return 0;
    }
    if ((offset + len) > size_)
    {
        len = size_ - offset;
    }
    assert((offset + len) <= size_);
    std::copy(data, data + len, data_ + offset);
    max_write_pos_ = std::max(offset + len, unsigned(max_write_pos_));
    return len;
}

void StaticTransferBufferImpl::reset()
{
    max_write_pos_ = 0;
#if UAVCAN_DEBUG
    std::fill(data_, data_ + size_, 0);
#endif
}

/*
 * StaticTransferBufferManagerEntryImpl
 */
void StaticTransferBufferManagerEntryImpl::resetImpl()
{
    buf_.reset();
}

int StaticTransferBufferManagerEntryImpl::read(unsigned offset, uint8_t* data, unsigned len) const
{
    return buf_.read(offset, data, len);
}

int StaticTransferBufferManagerEntryImpl::write(unsigned offset, const uint8_t* data, unsigned len)
{
    return buf_.write(offset, data, len);
}

bool StaticTransferBufferManagerEntryImpl::migrateFrom(const TransferBufferManagerEntry* tbme)
{
    if (tbme == NULL || tbme->isEmpty())
    {
        assert(0);
        return false;
    }

    // Resetting self and moving all data from the source
    TransferBufferManagerEntry::reset(tbme->getKey());
    const int res = tbme->read(0, buf_.getRawPtr(), buf_.getSize());
    if (res < 0)
    {
        TransferBufferManagerEntry::reset();
        return false;
    }
    buf_.setMaxWritePos(res);
    if (res < int(buf_.getSize()))
    {
        return true;
    }

    // Now we need to make sure that all data can fit the storage
    uint8_t dummy = 0;
    if (tbme->read(buf_.getSize(), &dummy, 1) > 0)
    {
        TransferBufferManagerEntry::reset();            // Damn, the buffer was too large
        return false;
    }
    return true;
}

/*
 * TransferBufferManagerImpl
 */
StaticTransferBufferManagerEntryImpl* TransferBufferManagerImpl::findFirstStatic(const TransferBufferManagerKey& key)
{
    for (unsigned i = 0; true; i++)
    {
        StaticTransferBufferManagerEntryImpl* const sb = getStaticByIndex(i);
        if (sb == NULL)
        {
            break;
        }
        if (sb->getKey() == key)
        {
            return sb;
        }
    }
    return NULL;
}

DynamicTransferBufferManagerEntry* TransferBufferManagerImpl::findFirstDynamic(const TransferBufferManagerKey& key)
{
    DynamicTransferBufferManagerEntry* dyn = dynamic_buffers_.get();
    while (dyn)
    {
        assert(!dyn->isEmpty());
        if (dyn->getKey() == key)
        {
            return dyn;
        }
        dyn = dyn->getNextListNode();
    }
    return NULL;
}

void TransferBufferManagerImpl::optimizeStorage()
{
    while (!dynamic_buffers_.isEmpty())
    {
        StaticTransferBufferManagerEntryImpl* const sb = findFirstStatic(TransferBufferManagerKey());
        if (sb == NULL)
        {
            break;
        }
        DynamicTransferBufferManagerEntry* dyn = dynamic_buffers_.get();
        assert(dyn);
        assert(!dyn->isEmpty());
        if (sb->migrateFrom(dyn))
        {
            assert(!dyn->isEmpty());
            UAVCAN_TRACE("TransferBufferManager", "Storage optimization: Migrated %s",
                         dyn->getKey().toString().c_str());
            dynamic_buffers_.remove(dyn);
            DynamicTransferBufferManagerEntry::destroy(dyn, allocator_);
        }
        else
        {
            /* Migration can fail if a dynamic buffer contains more data than a static buffer can accomodate.
             * This should never happen during normal operation because dynamic buffers are limited in growth.
             */
            UAVCAN_TRACE("TransferBufferManager", "Storage optimization: MIGRATION FAILURE %s MAXSIZE %u",
                         dyn->getKey().toString().c_str(), max_buf_size_);
            assert(0);
            sb->reset();
            break;
        }
    }
}

TransferBufferManagerImpl::~TransferBufferManagerImpl()
{
    DynamicTransferBufferManagerEntry* dyn = dynamic_buffers_.get();
    while (dyn)
    {
        DynamicTransferBufferManagerEntry* const next = dyn->getNextListNode();
        dynamic_buffers_.remove(dyn);
        DynamicTransferBufferManagerEntry::destroy(dyn, allocator_);
        dyn = next;
    }
}

ITransferBuffer* TransferBufferManagerImpl::access(const TransferBufferManagerKey& key)
{
    if (key.isEmpty())
    {
        assert(0);
        return NULL;
    }
    TransferBufferManagerEntry* tbme = findFirstStatic(key);
    if (tbme)
    {
        return tbme;
    }
    return findFirstDynamic(key);
}

ITransferBuffer* TransferBufferManagerImpl::create(const TransferBufferManagerKey& key)
{
    if (key.isEmpty())
    {
        assert(0);
        return NULL;
    }
    remove(key);

    TransferBufferManagerEntry* tbme = findFirstStatic(TransferBufferManagerKey());
    if (tbme == NULL)
    {
        DynamicTransferBufferManagerEntry* dyn =
            DynamicTransferBufferManagerEntry::instantiate(allocator_, max_buf_size_);
        tbme = dyn;
        if (dyn == NULL)
        {
            return NULL;     // Epic fail.
        }
        dynamic_buffers_.insert(dyn);
        UAVCAN_TRACE("TransferBufferManager", "Dynamic buffer created [st=%u, dyn=%u], %s",
                     getNumStaticBuffers(), getNumDynamicBuffers(), key.toString().c_str());
    }
    else
    {
        UAVCAN_TRACE("TransferBufferManager", "Static buffer created [st=%u, dyn=%u], %s",
                     getNumStaticBuffers(), getNumDynamicBuffers(), key.toString().c_str());
    }

    if (tbme)
    {
        assert(tbme->isEmpty());
        tbme->reset(key);
    }
    return tbme;
}

void TransferBufferManagerImpl::remove(const TransferBufferManagerKey& key)
{
    assert(!key.isEmpty());

    TransferBufferManagerEntry* const tbme = findFirstStatic(key);
    if (tbme)
    {
        UAVCAN_TRACE("TransferBufferManager", "Static buffer deleted, %s", key.toString().c_str());
        tbme->reset();
        optimizeStorage();
        return;
    }

    DynamicTransferBufferManagerEntry* dyn = findFirstDynamic(key);
    if (dyn)
    {
        UAVCAN_TRACE("TransferBufferManager", "Dynamic buffer deleted, %s", key.toString().c_str());
        dynamic_buffers_.remove(dyn);
        DynamicTransferBufferManagerEntry::destroy(dyn, allocator_);
    }
}

bool TransferBufferManagerImpl::isEmpty() const
{
    return (getNumStaticBuffers() == 0) && (getNumDynamicBuffers() == 0);
}

unsigned TransferBufferManagerImpl::getNumDynamicBuffers() const
{
    return dynamic_buffers_.getLength();
}

unsigned TransferBufferManagerImpl::getNumStaticBuffers() const
{
    unsigned res = 0;
    for (unsigned i = 0; true; i++)
    {
        StaticTransferBufferManagerEntryImpl* const sb = getStaticByIndex(i);
        if (sb == NULL)
        {
            break;
        }
        if (!sb->isEmpty())
        {
            res++;
        }
    }
    return res;
}

}
