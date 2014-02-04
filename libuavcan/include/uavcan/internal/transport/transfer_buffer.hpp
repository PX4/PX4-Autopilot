/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <stdint.h>
#include <algorithm>
#include <uavcan/internal/transport/transfer.hpp>
#include <uavcan/internal/linked_list.hpp>
#include <uavcan/internal/impl_constants.hpp>

namespace uavcan
{
#pragma pack(push, 1)
/**
 * API for transfer buffer users.
 */
class TransferBufferBase
{
    uint64_t update_timestamp_;

public:
    TransferBufferBase()
    : update_timestamp_(0)
    { }

    virtual ~TransferBufferBase() { }

    uint64_t getUpdateTimestamp() const { return update_timestamp_; }
    void setUpdateTimestamp(uint64_t val) { update_timestamp_ = val; }

    virtual int read(unsigned int offset, uint8_t* data, unsigned int len) const = 0;
    virtual int write(unsigned int offset, const uint8_t* data, unsigned int len) = 0;
};

/**
 * Internal for TransferBufferManager
 */
class TransferBufferManagerEntry : public TransferBufferBase
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
    virtual void cleanup(uint64_t oldest_timestamp) = 0;
};

}
