/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <stdint.h>
#include <algorithm>
#include <limits>
#include <uavcan/internal/transport/transfer.hpp>
#include <uavcan/internal/linked_list.hpp>
#include <uavcan/internal/impl_constants.hpp>
#include <uavcan/internal/debug.hpp>

namespace uavcan
{
#pragma pack(push, 1)
/**
 * API for transfer buffer users.
 */
class TransferBufferBase
{
public:
    virtual ~TransferBufferBase() { }

    virtual int read(unsigned int offset, uint8_t* data, unsigned int len) const = 0;
    virtual int write(unsigned int offset, const uint8_t* data, unsigned int len) = 0;
};

/**
 * Internal for TransferBufferManager
 */
class TransferBufferManagerEntry : public TransferBufferBase, Noncopyable
{
    uint8_t node_id_;

protected:
    virtual void resetImpl() = 0;

public:
    TransferBufferManagerEntry(uint8_t node_id = NODE_ID_INVALID)
    : node_id_(node_id)
    { }

    uint8_t getNodeID() const { return node_id_; }
    bool isEmpty() const { return node_id_ == NODE_ID_INVALID; }

    void reset(uint8_t node_id = NODE_ID_INVALID)
    {
        node_id_ = node_id;
        resetImpl();
    }
};

/**
 * Resizable gather/scatter storage.
 * reset() call releases all memory blocks.
 * Supports unordered write operations - from higher to lower offsets
 */
class DynamicTransferBuffer : public TransferBufferManagerEntry, public LinkedListNode<DynamicTransferBuffer>
{
    struct Block : LinkedListNode<Block>
    {
        enum { SIZE = MEM_POOL_BLOCK_SIZE - sizeof(LinkedListNode<Block>) };
        uint8_t data[SIZE];

        static Block* instantiate(IAllocator* allocator);
        static void destroy(Block*& obj, IAllocator* allocator);

        void read(uint8_t*& outptr, unsigned int target_offset,
                  unsigned int& total_offset, unsigned int& left_to_read);
        void write(const uint8_t*& inptr, unsigned int target_offset,
                   unsigned int& total_offset, unsigned int& left_to_write);
    };

    unsigned int max_write_pos_;
    IAllocator* allocator_;
    LinkedListRoot<Block> blocks_;    // Blocks are ordered from lower to higher buffer offset

    void resetImpl();

public:
    DynamicTransferBuffer(IAllocator* allocator)
    : max_write_pos_(0)
    , allocator_(allocator)
    {
        StaticAssert<(Block::SIZE > 8)>::check();
        IsDynamicallyAllocatable<Block>::check();
        IsDynamicallyAllocatable<DynamicTransferBuffer>::check();
    }

    ~DynamicTransferBuffer()
    {
        resetImpl();
    }

    static DynamicTransferBuffer* instantiate(IAllocator* allocator);
    static void destroy(DynamicTransferBuffer*& obj, IAllocator* allocator);

    int read(unsigned int offset, uint8_t* data, unsigned int len) const;
    int write(unsigned int offset, const uint8_t* data, unsigned int len);
};
#pragma pack(pop)

/**
 * Statically allocated storage
 */
template <unsigned int SIZE>
class StaticTransferBuffer : public TransferBufferManagerEntry
{
    uint8_t data_[SIZE];
    unsigned int max_write_pos_;

    void resetImpl()
    {
        max_write_pos_ = 0;
#if UAVCAN_DEBUG
        std::fill(data_, data_ + SIZE, 0);
#endif
    }

public:
    StaticTransferBuffer()
    : max_write_pos_(0)
    {
        StaticAssert<(SIZE > 0)>::check();
        std::fill(data_, data_ + SIZE, 0);
    }

    int read(unsigned int offset, uint8_t* data, unsigned int len) const
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
        std::copy(data_ + offset, data_ + offset + len, data);
        return len;
    }

    int write(unsigned int offset, const uint8_t* data, unsigned int len)
    {
        if (!data)
        {
            assert(0);
            return -1;
        }
        if (offset >= SIZE)
            return 0;
        if ((offset + len) > SIZE)
            len = SIZE - offset;
        assert((offset + len) <= SIZE);
        std::copy(data, data + len, data_ + offset);
        max_write_pos_ = std::max(offset + len, max_write_pos_);
        return len;
    }

    bool migrateFrom(const TransferBufferManagerEntry* tbme)
    {
        if (tbme == NULL || tbme->isEmpty())
        {
            assert(0);
            return false;
        }

        // Resetting self and moving all data from the source
        reset(tbme->getNodeID());
        const int res = tbme->read(0, data_, SIZE);
        if (res < 0)
        {
            reset();
            return false;
        }
        max_write_pos_ = res;
        if (res < int(SIZE))
            return true;

        // Now we need to make sure that all data can fit the storage
        uint8_t dummy = 0;
        if (tbme->read(SIZE, &dummy, 1) > 0)
        {
            reset();            // Damn, the buffer was too large
            return false;
        }
        return true;
    }
};

/**
 * Manages different storage types (static/dynamic) for transfer reception logic.
 */
class ITransferBufferManager
{
public:
    virtual ~ITransferBufferManager() { }
    virtual TransferBufferBase* access(uint8_t node_id) = 0;
    virtual TransferBufferBase* create(uint8_t node_id) = 0;
    virtual void remove(uint8_t node_id) = 0;
};

/**
 * Buffer manager implementation.
 */
template <unsigned int STATIC_BUF_SIZE, unsigned int NUM_STATIC_BUFS>
class TransferBufferManager : public ITransferBufferManager, Noncopyable
{
    typedef StaticTransferBuffer<STATIC_BUF_SIZE> StaticBufferType;

    StaticBufferType static_buffers_[NUM_STATIC_BUFS];
    LinkedListRoot<DynamicTransferBuffer> dynamic_buffers_;
    IAllocator* const allocator_;

    StaticBufferType* findFirstStatic(uint8_t node_id)
    {
        assert((node_id == NODE_ID_INVALID) || (node_id <= NODE_ID_MAX));
        for (unsigned int i = 0; i < NUM_STATIC_BUFS; i++)
        {
            if (static_buffers_[i].getNodeID() == node_id)
                return static_buffers_ + i;
        }
        return NULL;
    }

    DynamicTransferBuffer* findFirstDynamic(uint8_t node_id)
    {
        DynamicTransferBuffer* dyn = dynamic_buffers_.get();
        while (dyn)
        {
            assert(!dyn->isEmpty());
            if (dyn->getNodeID() == node_id)
                return dyn;
            dyn = dyn->getNextListNode();
        }
        return NULL;
    }

    void optimizeStorage()
    {
        while (!dynamic_buffers_.isEmpty())
        {
            StaticBufferType* const sb = findFirstStatic(NODE_ID_INVALID);
            if (sb == NULL)
                break;
            DynamicTransferBuffer* dyn = dynamic_buffers_.get();
            assert(dyn);
            assert(!dyn->isEmpty());
            if (sb->migrateFrom(dyn))
            {
                assert(!dyn->isEmpty());
                UAVCAN_TRACE("TransferBufferManager", "Storage optimization: Migrated NID %i", int(dyn->getNodeID()));
                dynamic_buffers_.remove(dyn);
                DynamicTransferBuffer::destroy(dyn, allocator_);
            }
            else
            {
                /* Migration can fail if a dynamic buffer contains more data than a static buffer can accomodate (more
                 * than STATIC_BUF_SIZE). This means that there is probably something wrong with the network. Logic
                 * that uses this class should explicitly ensure the proper maximum data size.
                 */
                UAVCAN_TRACE("TransferBufferManager", "Storage optimization: MIGRATION FAILURE NID %i BUFSIZE %u",
                    int(dyn->getNodeID()), STATIC_BUF_SIZE);
                sb->reset();
                break;         // Probably we should try to migrate the rest?
            }
        }
    }

public:
    TransferBufferManager(IAllocator* allocator)
    : allocator_(allocator)
    { }

    ~TransferBufferManager()
    {
        DynamicTransferBuffer* dyn = dynamic_buffers_.get();
        while (dyn)
        {
            DynamicTransferBuffer* const next = dyn->getNextListNode();
            dynamic_buffers_.remove(dyn);
            DynamicTransferBuffer::destroy(dyn, allocator_);
            dyn = next;
        }
    }

    unsigned int getNumDynamicBuffers() const { return dynamic_buffers_.length(); }

    unsigned int getNumStaticBuffers() const
    {
        unsigned int res = 0;
        for (unsigned int i = 0; i < NUM_STATIC_BUFS; i++)
        {
            if (!static_buffers_[i].isEmpty())
                res++;
        }
        return res;
    }

    TransferBufferBase* access(uint8_t node_id)
    {
        if (node_id > NODE_ID_MAX || node_id == NODE_ID_INVALID)
        {
            assert(0);
            return NULL;
        }
        TransferBufferManagerEntry* tbme = findFirstStatic(node_id);
        if (tbme)
            return tbme;
        return findFirstDynamic(node_id);
    }

    TransferBufferBase* create(uint8_t node_id)
    {
        if (node_id > NODE_ID_MAX || node_id == NODE_ID_INVALID)
        {
            assert(0);
            return NULL;
        }
        remove(node_id);

        TransferBufferManagerEntry* tbme = findFirstStatic(NODE_ID_INVALID);
        if (tbme == NULL)
        {
            DynamicTransferBuffer* dyn = DynamicTransferBuffer::instantiate(allocator_);
            tbme = dyn;
            if (dyn == NULL)
                return NULL;     // Epic fail.
            dynamic_buffers_.insert(dyn);
        }

        if (tbme)
        {
            assert(tbme->isEmpty());
            tbme->reset(node_id);
        }
        return tbme;
    }

    void remove(uint8_t node_id)
    {
        assert((node_id <= NODE_ID_MAX) && (node_id != NODE_ID_INVALID));

        TransferBufferManagerEntry* const tbme = findFirstStatic(node_id);
        if (tbme)
        {
            tbme->reset();
            optimizeStorage();
            return;
        }

        DynamicTransferBuffer* dyn = findFirstDynamic(node_id);
        if (dyn)
        {
            dynamic_buffers_.remove(dyn);
            DynamicTransferBuffer::destroy(dyn, allocator_);
        }
    }
};

}
