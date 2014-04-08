/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <algorithm>
#include <limits>
#include <uavcan/stdint.hpp>
#include <uavcan/error.hpp>
#include <uavcan/transport/frame.hpp>
#include <uavcan/linked_list.hpp>
#include <uavcan/dynamic_memory.hpp>
#include <uavcan/impl_constants.hpp>
#include <uavcan/debug.hpp>

namespace uavcan
{
UAVCAN_PACKED_BEGIN
/**
 * API for transfer buffer users.
 */
class UAVCAN_EXPORT ITransferBuffer
{
public:
    virtual ~ITransferBuffer() { }

    virtual int read(unsigned offset, uint8_t* data, unsigned len) const = 0;
    virtual int write(unsigned offset, const uint8_t* data, unsigned len) = 0;
};

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
        assert(isEmpty());
    }

    TransferBufferManagerKey(NodeID node_id, TransferType ttype)
        : node_id_(node_id)
        , transfer_type_(ttype)
    {
        assert(!isEmpty());
    }

    bool operator==(const TransferBufferManagerKey& rhs) const
    {
        return node_id_ == rhs.node_id_ && transfer_type_ == rhs.transfer_type_;
    }

    bool isEmpty() const { return !node_id_.isValid(); }

    NodeID getNodeID() const { return node_id_; }
    TransferType getTransferType() const { return TransferType(transfer_type_); }

    std::string toString() const;
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

    TransferBufferManagerEntry(const TransferBufferManagerKey& key)
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
        uint8_t data[Size];

        static Block* instantiate(IAllocator& allocator);
        static void destroy(Block*& obj, IAllocator& allocator);

        void read(uint8_t*& outptr, unsigned target_offset,
                  unsigned& total_offset, unsigned& left_to_read);
        void write(const uint8_t*& inptr, unsigned target_offset,
                   unsigned& total_offset, unsigned& left_to_write);
    };

    IAllocator& allocator_;
    LinkedListRoot<Block> blocks_;    // Blocks are ordered from lower to higher buffer offset
    unsigned max_write_pos_;
    const unsigned max_size_;

    void resetImpl();

public:
    DynamicTransferBufferManagerEntry(IAllocator& allocator, unsigned max_size)
        : allocator_(allocator)
        , max_write_pos_(0)
        , max_size_(max_size)
    {
        StaticAssert<(Block::Size > 8)>::check();
        IsDynamicallyAllocatable<Block>::check();
        IsDynamicallyAllocatable<DynamicTransferBufferManagerEntry>::check();
    }

    ~DynamicTransferBufferManagerEntry()
    {
        resetImpl();
    }

    static DynamicTransferBufferManagerEntry* instantiate(IAllocator& allocator, unsigned max_size);
    static void destroy(DynamicTransferBufferManagerEntry*& obj, IAllocator& allocator);

    int read(unsigned offset, uint8_t* data, unsigned len) const;
    int write(unsigned offset, const uint8_t* data, unsigned len);
};
UAVCAN_PACKED_END

/**
 * Standalone static buffer
 */
class StaticTransferBufferImpl : public ITransferBuffer
{
    uint8_t* const data_;
    const unsigned size_;
    unsigned max_write_pos_;

public:
    StaticTransferBufferImpl(uint8_t* buf, unsigned buf_size)
        : data_(buf)
        , size_(buf_size)
        , max_write_pos_(0)
    { }

    int read(unsigned offset, uint8_t* data, unsigned len) const;

    int write(unsigned offset, const uint8_t* data, unsigned len);

    void reset();

    unsigned getSize() const { return size_; }

    uint8_t* getRawPtr() { return data_; }
    const uint8_t* getRawPtr() const { return data_; }

    unsigned getMaxWritePos() const { return max_write_pos_; }
    void setMaxWritePos(unsigned value) { max_write_pos_ = value; }
};

template <unsigned Size>
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

    void resetImpl();

public:
    StaticTransferBufferManagerEntryImpl(uint8_t* buf, unsigned buf_size)
        : buf_(buf, buf_size)
    { }

    int read(unsigned offset, uint8_t* data, unsigned len) const;

    int write(unsigned offset, const uint8_t* data, unsigned len);

    bool migrateFrom(const TransferBufferManagerEntry* tbme);
};

template <unsigned Size>
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
        assert(!key.isEmpty());
    }
    ITransferBuffer* access() { return bufmgr_.access(key_); }
    ITransferBuffer* create() { return bufmgr_.create(key_); }
    void remove() { bufmgr_.remove(key_); }
};

/**
 * Buffer manager implementation.
 */
template <unsigned MaxBufSize, unsigned NumStaticBufs>
class UAVCAN_EXPORT TransferBufferManager : public ITransferBufferManager, Noncopyable
{
    typedef StaticTransferBufferManagerEntry<MaxBufSize> StaticBufferType;

    StaticBufferType static_buffers_[NumStaticBufs];
    LinkedListRoot<DynamicTransferBufferManagerEntry> dynamic_buffers_;
    IAllocator& allocator_;

    StaticBufferType* findFirstStatic(const TransferBufferManagerKey& key);

    DynamicTransferBufferManagerEntry* findFirstDynamic(const TransferBufferManagerKey& key);

    void optimizeStorage();

public:
    TransferBufferManager(IAllocator& allocator)
        : allocator_(allocator)
    {
        StaticAssert<(MaxBufSize > 0)>::check();
        StaticAssert<(NumStaticBufs > 0)>::check();
    }

    ~TransferBufferManager();

    ITransferBuffer* access(const TransferBufferManagerKey& key);

    ITransferBuffer* create(const TransferBufferManagerKey& key);

    void remove(const TransferBufferManagerKey& key);

    bool isEmpty() const;

    unsigned getNumDynamicBuffers() const;

    unsigned getNumStaticBuffers() const;
};

template <>
class UAVCAN_EXPORT TransferBufferManager<0, 0> : public ITransferBufferManager
{
public:
    TransferBufferManager() { }
    TransferBufferManager(IAllocator& allocator)
    {
        (void)allocator;
    }

    ITransferBuffer* access(const TransferBufferManagerKey& key)
    {
        (void)key;
        return NULL;
    }

    ITransferBuffer* create(const TransferBufferManagerKey& key)
    {
        (void)key;
        return NULL;
    }

    void remove(const TransferBufferManagerKey& key)
    {
        (void)key;
    }

    bool isEmpty() const { return true; }
};

// ----------------------------------------------------------------------------

/*
 * TransferBufferManager<>
 */
template <unsigned MaxBufSize, unsigned NumStaticBufs>
typename TransferBufferManager<MaxBufSize, NumStaticBufs>::StaticBufferType*
TransferBufferManager<MaxBufSize, NumStaticBufs>::findFirstStatic(const TransferBufferManagerKey& key)
{
    for (unsigned i = 0; i < NumStaticBufs; i++)
    {
        if (static_buffers_[i].getKey() == key)
        {
            return static_buffers_ + i;
        }
    }
    return NULL;
}

template <unsigned MaxBufSize, unsigned NumStaticBufs>
DynamicTransferBufferManagerEntry*
TransferBufferManager<MaxBufSize, NumStaticBufs>::findFirstDynamic(const TransferBufferManagerKey& key)
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

template <unsigned MaxBufSize, unsigned NumStaticBufs>
void TransferBufferManager<MaxBufSize, NumStaticBufs>::optimizeStorage()
{
    while (!dynamic_buffers_.isEmpty())
    {
        StaticBufferType* const sb = findFirstStatic(TransferBufferManagerKey());
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
                         dyn->getKey().toString().c_str(), MaxBufSize);
            assert(0);
            sb->reset();
            break;
        }
    }
}

template <unsigned MaxBufSize, unsigned NumStaticBufs>
TransferBufferManager<MaxBufSize, NumStaticBufs>::~TransferBufferManager()
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

template <unsigned MaxBufSize, unsigned NumStaticBufs>
ITransferBuffer* TransferBufferManager<MaxBufSize, NumStaticBufs>::access(const TransferBufferManagerKey& key)
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

template <unsigned MaxBufSize, unsigned NumStaticBufs>
ITransferBuffer* TransferBufferManager<MaxBufSize, NumStaticBufs>::create(const TransferBufferManagerKey& key)
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
            DynamicTransferBufferManagerEntry::instantiate(allocator_, MaxBufSize);
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

template <unsigned MaxBufSize, unsigned NumStaticBufs>
void TransferBufferManager<MaxBufSize, NumStaticBufs>::remove(const TransferBufferManagerKey& key)
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

template <unsigned MaxBufSize, unsigned NumStaticBufs>
bool TransferBufferManager<MaxBufSize, NumStaticBufs>::isEmpty() const
{
    return (getNumStaticBuffers() == 0) && (getNumDynamicBuffers() == 0);
}

template <unsigned MaxBufSize, unsigned NumStaticBufs>
unsigned TransferBufferManager<MaxBufSize, NumStaticBufs>::getNumDynamicBuffers() const
{
    return dynamic_buffers_.getLength();
}

template <unsigned MaxBufSize, unsigned NumStaticBufs>
unsigned TransferBufferManager<MaxBufSize, NumStaticBufs>::getNumStaticBuffers() const
{
    unsigned res = 0;
    for (unsigned i = 0; i < NumStaticBufs; i++)
    {
        if (!static_buffers_[i].isEmpty())
        {
            res++;
        }
    }
    return res;
}

}
