/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/stdint.hpp>
#include <uavcan/error.hpp>
#include <uavcan/transport/frame.hpp>
#include <uavcan/transport/abstract_transfer_buffer.hpp>
#include <uavcan/util/linked_list.hpp>
#include <uavcan/dynamic_memory.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/debug.hpp>

namespace uavcan
{
UAVCAN_PACKED_BEGIN
/**
 * Internal for TransferBufferManager
 */
class UAVCAN_EXPORT TransferBufferManagerKey
{
    NodeID node_id_;
    uint8_t transfer_type_;

public:
    TransferBufferManagerKey()
        : transfer_type_(TransferType(0))
    {
        UAVCAN_ASSERT(isEmpty());
    }

    TransferBufferManagerKey(NodeID node_id, TransferType ttype)
        : node_id_(node_id)
        , transfer_type_(ttype)
    {
        UAVCAN_ASSERT(!isEmpty());
    }

    bool operator==(const TransferBufferManagerKey& rhs) const
    {
        return node_id_ == rhs.node_id_ && transfer_type_ == rhs.transfer_type_;
    }

    bool isEmpty() const { return !node_id_.isValid(); }

    NodeID getNodeID() const { return node_id_; }
    TransferType getTransferType() const { return TransferType(transfer_type_); }

#if UAVCAN_TOSTRING
    std::string toString() const;
#endif
};

/**
 * Internal for TransferBufferManager
 */
class UAVCAN_EXPORT TransferBufferManagerEntry : public ITransferBuffer, Noncopyable
{
    TransferBufferManagerKey key_;

protected:
    virtual void resetImpl() = 0;

public:
    TransferBufferManagerEntry() { }

    explicit TransferBufferManagerEntry(const TransferBufferManagerKey& key)
        : key_(key)
    { }

    const TransferBufferManagerKey& getKey() const { return key_; }
    bool isEmpty() const { return key_.isEmpty(); }

    void reset(const TransferBufferManagerKey& key = TransferBufferManagerKey())
    {
        key_ = key;
        resetImpl();
    }
};

/**
 * Resizable gather/scatter storage.
 * reset() call releases all memory blocks.
 * Supports unordered write operations - from higher to lower offsets
 */
class UAVCAN_EXPORT DynamicTransferBufferManagerEntry
    : public TransferBufferManagerEntry
    , public LinkedListNode<DynamicTransferBufferManagerEntry>
{
    struct Block : LinkedListNode<Block>
    {
        enum { Size = MemPoolBlockSize - sizeof(LinkedListNode<Block>) };
        uint8_t data[static_cast<unsigned>(Size)];

        static Block* instantiate(IPoolAllocator& allocator);
        static void destroy(Block*& obj, IPoolAllocator& allocator);

        void read(uint8_t*& outptr, unsigned target_offset,
                  unsigned& total_offset, unsigned& left_to_read);
        void write(const uint8_t*& inptr, unsigned target_offset,
                   unsigned& total_offset, unsigned& left_to_write);
    };

    IPoolAllocator& allocator_;
    LinkedListRoot<Block> blocks_;    // Blocks are ordered from lower to higher buffer offset
    uint16_t max_write_pos_;
    const uint16_t max_size_;

    /// Reset functionality must be implemented in a non-virtual method to call it safely from the destructor.
    void doReset();

    virtual void resetImpl();

public:
    DynamicTransferBufferManagerEntry(IPoolAllocator& allocator, uint16_t max_size)
        : allocator_(allocator)
        , max_write_pos_(0)
        , max_size_(max_size)
    {
        StaticAssert<(Block::Size > 8)>::check();
        IsDynamicallyAllocatable<Block>::check();
        IsDynamicallyAllocatable<DynamicTransferBufferManagerEntry>::check();
    }

    virtual ~DynamicTransferBufferManagerEntry()
    {
        doReset();
    }

    static DynamicTransferBufferManagerEntry* instantiate(IPoolAllocator& allocator, uint16_t max_size);
    static void destroy(DynamicTransferBufferManagerEntry*& obj, IPoolAllocator& allocator);

    virtual int read(unsigned offset, uint8_t* data, unsigned len) const;
    virtual int write(unsigned offset, const uint8_t* data, unsigned len);
};
UAVCAN_PACKED_END

/**
 * Standalone static buffer
 */
class StaticTransferBufferImpl : public ITransferBuffer
{
    uint8_t* const data_;
    const uint16_t size_;
    uint16_t max_write_pos_;

public:
    StaticTransferBufferImpl(uint8_t* buf, uint16_t buf_size)
        : data_(buf)
        , size_(buf_size)
        , max_write_pos_(0)
    { }

    virtual int read(unsigned offset, uint8_t* data, unsigned len) const;
    virtual int write(unsigned offset, const uint8_t* data, unsigned len);

    void reset();

    uint16_t getSize() const { return size_; }

    uint8_t* getRawPtr() { return data_; }
    const uint8_t* getRawPtr() const { return data_; }

    uint16_t getMaxWritePos() const { return max_write_pos_; }
    void setMaxWritePos(uint16_t value) { max_write_pos_ = value; }
};

template <uint16_t Size>
class UAVCAN_EXPORT StaticTransferBuffer : public StaticTransferBufferImpl
{
    uint8_t buffer_[Size];
public:
    StaticTransferBuffer()
        : StaticTransferBufferImpl(buffer_, Size)
    {
        StaticAssert<(Size > 0)>::check();
    }
};

/**
 * Statically allocated storage for the buffer manager
 */
class StaticTransferBufferManagerEntryImpl : public TransferBufferManagerEntry
{
    StaticTransferBufferImpl buf_;

    virtual void resetImpl();

public:
    StaticTransferBufferManagerEntryImpl(uint8_t* buf, uint16_t buf_size)
        : buf_(buf, buf_size)
    { }

    virtual int read(unsigned offset, uint8_t* data, unsigned len) const;
    virtual int write(unsigned offset, const uint8_t* data, unsigned len);

    bool migrateFrom(const TransferBufferManagerEntry* tbme);
};

template <uint16_t Size>
class UAVCAN_EXPORT StaticTransferBufferManagerEntry : public StaticTransferBufferManagerEntryImpl
{
    uint8_t buffer_[Size];
public:
    StaticTransferBufferManagerEntry()
        : StaticTransferBufferManagerEntryImpl(buffer_, Size)
    { }
};

/**
 * Manages different storage types (static/dynamic) for transfer reception logic.
 */
class UAVCAN_EXPORT ITransferBufferManager
{
public:
    virtual ~ITransferBufferManager() { }
    virtual ITransferBuffer* access(const TransferBufferManagerKey& key) = 0;
    virtual ITransferBuffer* create(const TransferBufferManagerKey& key) = 0;
    virtual void remove(const TransferBufferManagerKey& key) = 0;
    virtual bool isEmpty() const = 0;
};

/**
 * Convinience class.
 */
class UAVCAN_EXPORT TransferBufferAccessor
{
    ITransferBufferManager& bufmgr_;
    const TransferBufferManagerKey key_;

public:
    TransferBufferAccessor(ITransferBufferManager& bufmgr, TransferBufferManagerKey key)
        : bufmgr_(bufmgr)
        , key_(key)
    {
        UAVCAN_ASSERT(!key.isEmpty());
    }
    ITransferBuffer* access() { return bufmgr_.access(key_); }
    ITransferBuffer* create() { return bufmgr_.create(key_); }
    void remove() { bufmgr_.remove(key_); }
};

/**
 * Buffer manager implementation.
 */
class TransferBufferManagerImpl : public ITransferBufferManager, Noncopyable
{
    LinkedListRoot<DynamicTransferBufferManagerEntry> dynamic_buffers_;
    IPoolAllocator& allocator_;
    const uint16_t max_buf_size_;

    virtual StaticTransferBufferManagerEntryImpl* getStaticByIndex(uint16_t index) const = 0;

    StaticTransferBufferManagerEntryImpl* findFirstStatic(const TransferBufferManagerKey& key);
    DynamicTransferBufferManagerEntry* findFirstDynamic(const TransferBufferManagerKey& key);
    void optimizeStorage();

public:
    TransferBufferManagerImpl(uint16_t max_buf_size, IPoolAllocator& allocator)
        : allocator_(allocator)
        , max_buf_size_(max_buf_size)
    { }

    virtual ~TransferBufferManagerImpl();

    virtual ITransferBuffer* access(const TransferBufferManagerKey& key);
    virtual ITransferBuffer* create(const TransferBufferManagerKey& key);
    virtual void remove(const TransferBufferManagerKey& key);
    virtual bool isEmpty() const;

    unsigned getNumDynamicBuffers() const;
    unsigned getNumStaticBuffers() const;
};

template <uint16_t MaxBufSize, uint8_t NumStaticBufs>
class UAVCAN_EXPORT TransferBufferManager : public TransferBufferManagerImpl
{
    mutable StaticTransferBufferManagerEntry<MaxBufSize> static_buffers_[NumStaticBufs];

    virtual StaticTransferBufferManagerEntry<MaxBufSize>* getStaticByIndex(uint16_t index) const
    {
        return (index < NumStaticBufs) ? &static_buffers_[index] : NULL;
    }

public:
    explicit TransferBufferManager(IPoolAllocator& allocator)
        : TransferBufferManagerImpl(MaxBufSize, allocator)
    {
#if UAVCAN_TINY
        StaticAssert<(NumStaticBufs == 0)>::check();   // Static buffers in UAVCAN_TINY mode are not allowed
#endif
        StaticAssert<(MaxBufSize > 0)>::check();
    }
};

template <uint16_t MaxBufSize>
class UAVCAN_EXPORT TransferBufferManager<MaxBufSize, 0> : public TransferBufferManagerImpl
{
    virtual StaticTransferBufferManagerEntry<MaxBufSize>* getStaticByIndex(uint16_t) const
    {
        return NULL;
    }

public:
    explicit TransferBufferManager(IPoolAllocator& allocator)
        : TransferBufferManagerImpl(MaxBufSize, allocator)
    {
        StaticAssert<(MaxBufSize > 0)>::check();
    }
};

template <>
class UAVCAN_EXPORT TransferBufferManager<0, 0> : public ITransferBufferManager
{
public:
    TransferBufferManager() { }
    TransferBufferManager(IPoolAllocator&) { }
    virtual ITransferBuffer* access(const TransferBufferManagerKey&) { return NULL; }
    virtual ITransferBuffer* create(const TransferBufferManagerKey&) { return NULL; }
    virtual void remove(const TransferBufferManagerKey&) { }
    virtual bool isEmpty() const { return true; }
};

}
