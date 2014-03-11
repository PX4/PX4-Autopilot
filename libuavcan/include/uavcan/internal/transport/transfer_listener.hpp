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
#include <uavcan/internal/transport/crc.hpp>
#include <uavcan/data_type.hpp>

namespace uavcan
{
/**
 * Container for received transfer.
 */
class IncomingTransfer : public ITransferBuffer
{
    MonotonicTime ts_mono_;
    UtcTime ts_utc_;
    TransferType transfer_type_;
    TransferID transfer_id_;
    NodeID src_node_id_;

    /// That's a no-op, asserts in debug builds
    int write(unsigned int offset, const uint8_t* data, unsigned int len);

protected:
    IncomingTransfer(MonotonicTime ts_mono, UtcTime ts_utc, TransferType transfer_type,
                     TransferID transfer_id, NodeID source_node_id)
    : ts_mono_(ts_mono)
    , ts_utc_(ts_utc)
    , transfer_type_(transfer_type)
    , transfer_id_(transfer_id)
    , src_node_id_(source_node_id)
    { }

public:
    /**
     * Dispose the payload buffer. Further calls to read() will not be possible.
     */
    virtual void release() { }

    MonotonicTime getMonotonicTimestamp() const { return ts_mono_; }
    UtcTime getUtcTimestamp()             const { return ts_utc_; }
    TransferType getTransferType()        const { return transfer_type_; }
    TransferID getTransferID()            const { return transfer_id_; }
    NodeID getSrcNodeID()                 const { return src_node_id_; }
};

/**
 * Internal.
 */
class SingleFrameIncomingTransfer : public IncomingTransfer
{
    const uint8_t* const payload_;
    const uint8_t payload_len_;
public:
    SingleFrameIncomingTransfer(const RxFrame& frm);
    int read(unsigned int offset, uint8_t* data, unsigned int len) const;
};

/**
 * Internal.
 */
class MultiFrameIncomingTransfer : public IncomingTransfer, Noncopyable
{
    TransferBufferAccessor& buf_acc_;
public:
    MultiFrameIncomingTransfer(MonotonicTime ts_mono, UtcTime ts_utc, const RxFrame& last_frame,
                               TransferBufferAccessor& tba);
    int read(unsigned int offset, uint8_t* data, unsigned int len) const;
    void release() { buf_acc_.remove(); }
};

/**
 * Internal, refer to transport dispatcher.
 */
class TransferListenerBase : public LinkedListNode<TransferListenerBase>
{
    const DataTypeDescriptor& data_type_;
    const TransferCRC crc_base_;                      ///< Pre-initialized with data type hash, thus constant

    bool checkPayloadCrc(const uint16_t compare_with, const ITransferBuffer& tbb) const;

protected:
    TransferListenerBase(const DataTypeDescriptor& data_type)
    : data_type_(data_type)
    , crc_base_(data_type.getSignature().toTransferCRC())
    { }

    virtual ~TransferListenerBase() { }

    void handleReception(TransferReceiver& receiver, const RxFrame& frame, TransferBufferAccessor& tba);

    virtual void handleIncomingTransfer(IncomingTransfer& transfer) = 0;

public:
    const DataTypeDescriptor& getDataTypeDescriptor() const { return data_type_; }

    virtual void handleFrame(const RxFrame& frame) = 0;
    virtual void cleanup(MonotonicTime ts) = 0;
};

/**
 * This class should be derived by transfer receivers (subscribers, servers, callers).
 */
template <unsigned int MaxBufSize, unsigned int NumStaticBufs, unsigned int NumStaticReceivers>
class TransferListener : public TransferListenerBase, Noncopyable
{
    typedef TransferBufferManager<MaxBufSize, NumStaticBufs> BufferManager;
    BufferManager bufmgr_;
    Map<TransferBufferManagerKey, TransferReceiver, NumStaticReceivers> receivers_;

    void handleFrame(const RxFrame& frame)
    {
        const TransferBufferManagerKey key(frame.getSrcNodeID(), frame.getTransferType());

        TransferReceiver* recv = receivers_.access(key);
        if (recv == NULL)
        {
            if (!frame.isFirst())
                return;

            TransferReceiver new_recv;
            recv = receivers_.insert(key, new_recv);
            if (recv == NULL)
            {
                UAVCAN_TRACE("TransferListener", "Receiver registration failed; frame %s", frame.toString().c_str());
                return;
            }
        }
        TransferBufferAccessor tba(bufmgr_, key);
        handleReception(*recv, frame, tba);
    }

    class TimedOutReceiverPredicate
    {
        const MonotonicTime ts_;
        BufferManager& bufmgr_;

    public:
        TimedOutReceiverPredicate(MonotonicTime ts, BufferManager& bufmgr)
        : ts_(ts)
        , bufmgr_(bufmgr)
        { }

        bool operator()(const TransferBufferManagerKey& key, const TransferReceiver& value) const
        {
            if (value.isTimedOut(ts_))
            {
                UAVCAN_TRACE("TransferListener", "Timed out receiver: %s", key.toString().c_str());
                /*
                 * TransferReceivers do not own their buffers - this helps the Map<> container to copy them
                 * around quickly and safely (using default assignment operator). Downside is that we need to
                 * destroy the buffers manually.
                 * Maybe it is not good that the predicate has side effects, but I ran out of better ideas.
                 */
                bufmgr_.remove(key);
                return true;
            }
            return false;
        }
    };

    void cleanup(MonotonicTime ts)
    {
        receivers_.removeWhere(TimedOutReceiverPredicate(ts, bufmgr_));
        assert(receivers_.isEmpty() ? bufmgr_.isEmpty() : 1);
    }

public:
    TransferListener(const DataTypeDescriptor& data_type, IAllocator& allocator)
    : TransferListenerBase(data_type)
    , bufmgr_(allocator)
    , receivers_(allocator)
    {
        StaticAssert<(NumStaticReceivers >= NumStaticBufs)>::check();  // Otherwise it would be meaningless
    }

    ~TransferListener()
    {
        // Map must be cleared before bufmgr is destructed
        receivers_.removeAll();
    }
};

}
