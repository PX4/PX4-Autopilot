/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <algorithm>
#include <stdint.h>
#include <uavcan/internal/transport/transfer_receiver.hpp>
#include <uavcan/internal/linked_list.hpp>
#include <uavcan/internal/map.hpp>
#include <uavcan/internal/debug.hpp>
#include <uavcan/internal/data_type.hpp>
#include <uavcan/internal/transport/crc.hpp>

namespace uavcan
{
/**
 * Container for received transfer.
 */
class IncomingTransfer
{
    uint64_t ts_monotonic_;
    uint64_t ts_utc_;
    TransferType transfer_type_;
    TransferID transfer_id_;
    uint8_t source_node_id_;

protected:
    IncomingTransfer(uint64_t ts_monotonic, uint64_t ts_utc, TransferType transfer_type,
                     TransferID transfer_id, uint8_t source_node_id)
    : ts_monotonic_(ts_monotonic)
    , ts_utc_(ts_utc)
    , transfer_type_(transfer_type)
    , transfer_id_(transfer_id)
    , source_node_id_(source_node_id)
    { }

public:
    virtual ~IncomingTransfer() { }

    /**
     * Read pure payload, no service fields are included (e.g. Target Node ID, Transfer CRC)
     */
    virtual int read(unsigned int offset, uint8_t* data, unsigned int len) const = 0;

    /**
     * Dispose the payload buffer. Further calls to read() will not be possible.
     */
    virtual void release() { }

    uint64_t getMonotonicTimestamp() const { return ts_monotonic_; }
    uint64_t getUtcTimestamp()       const { return ts_utc_; }
    TransferType getTransferType()   const { return transfer_type_; }
    TransferID getTransferID()       const { return transfer_id_; }
    uint8_t getSourceNodeID()        const { return source_node_id_; }
};

/**
 * Internal.
 */
class SingleFrameIncomingTransfer : public IncomingTransfer
{
    const uint8_t* const payload_;
    const uint8_t payload_len_;
public:
    SingleFrameIncomingTransfer(const RxFrame& frm, const uint8_t* payload, unsigned int payload_len);
    int read(unsigned int offset, uint8_t* data, unsigned int len) const;
};

/**
 * Internal.
 */
class MultiFrameIncomingTransfer : public IncomingTransfer, Noncopyable
{
    TransferBufferAccessor& buf_acc_;
public:
    MultiFrameIncomingTransfer(uint64_t ts_monotonic, uint64_t ts_utc, const RxFrame& last_frame,
                               TransferBufferAccessor& tba);
    int read(unsigned int offset, uint8_t* data, unsigned int len) const;
    void release() { buf_acc_.remove(); }
};

/**
 * Internal, refer to transport dispatcher.
 */
class TransferListenerBase : public LinkedListNode<TransferListenerBase>
{
    const Crc16 crc_base_;     ///< Pre-initialized with data type hash, thus constant

    bool checkPayloadCrc(const uint16_t compare_with, const TransferBufferBase& tbb) const;

protected:
    TransferListenerBase(const DataTypeDescriptor* data_type)
    : crc_base_(data_type->hash.value, DataTypeHash::NUM_BYTES)
    {
        assert(data_type);
    }

    virtual ~TransferListenerBase() { }

    void handleReception(TransferReceiver& receiver, const RxFrame& frame, TransferBufferAccessor& tba);

    virtual void handleIncomingTransfer(IncomingTransfer& transfer) = 0;

public:
    virtual void handleFrame(const RxFrame& frame) = 0;
    virtual void cleanup(uint64_t ts_monotonic) = 0;
};

/**
 * This class should be derived by transfer receivers (subscribers, servers, callers).
 */
template <unsigned int MAX_BUF_SIZE, unsigned int NUM_STATIC_BUFS, unsigned int NUM_STATIC_RECEIVERS>
class TransferListener : public TransferListenerBase, Noncopyable
{
    typedef TransferBufferManager<MAX_BUF_SIZE, NUM_STATIC_BUFS> BufferManager;
    BufferManager bufmgr_;
    Map<TransferBufferManagerKey, TransferReceiver, NUM_STATIC_RECEIVERS> receivers_;

    void handleFrame(const RxFrame& frame)
    {
        const TransferBufferManagerKey key(frame.source_node_id, frame.transfer_type);

        TransferReceiver* recv = receivers_.access(key);
        if (recv == NULL)
        {
            if (frame.frame_index != 0)   // We don't want to add registrations mid-transfer, that's pointless
                return;

            TransferReceiver new_recv;
            recv = receivers_.insert(key, new_recv);
            if (recv == NULL)
            {
                UAVCAN_TRACE("TransferListener", "Receiver registration failed; frame %s", frame.toString().c_str());
                return;
            }
        }
        TransferBufferAccessor tba(&bufmgr_, key);
        handleReception(*recv, frame, tba);
    }

    class TimedOutReceiverPredicate
    {
        const uint64_t ts_monotonic_;
        BufferManager& bufmgr_;

    public:
        TimedOutReceiverPredicate(uint64_t ts_monotonic, BufferManager& bufmgr)
        : ts_monotonic_(ts_monotonic)
        , bufmgr_(bufmgr)
        { }

        bool operator()(const TransferBufferManagerKey& key, const TransferReceiver& value) const
        {
            if (value.isTimedOut(ts_monotonic_))
            {
                /*
                 * TransferReceivers do not own their buffers - this helps the Map<> container to copy them
                 * around quickly and safely (using default operator==()). Downside is that we need to destroy
                 * the buffers manually.
                 * Maybe it is not good that the predicate is being using as mapping functor, but I ran out
                 * of better ideas.
                 */
                bufmgr_.remove(key);
                return true;
            }
            return false;
        }
    };

    void cleanup(uint64_t ts_monotonic)
    {
        receivers_.removeWhere(TimedOutReceiverPredicate(ts_monotonic, bufmgr_));
#if UAVCAN_DEBUG
        if (receivers_.isEmpty())
        {
            assert(bufmgr_.isEmpty());
        }
#endif
    }

public:
    TransferListener(const DataTypeDescriptor* data_type, IAllocator* allocator)
    : TransferListenerBase(data_type)
    , bufmgr_(allocator)
    , receivers_(allocator)
    {
        StaticAssert<(NUM_STATIC_RECEIVERS >= NUM_STATIC_BUFS)>::check();  // Otherwise it would be meaningless
    }

    ~TransferListener()
    {
        // Map must be cleared before bufmgr is destructed
        receivers_.removeAll();
    }
};

}
