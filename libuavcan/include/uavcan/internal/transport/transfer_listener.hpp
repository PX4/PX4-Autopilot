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
    uint8_t payload_[Frame::PAYLOAD_LEN_MAX];
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
    ITransferBufferManager* const bufmgr_;
    const TransferBufferManagerKey bufmgr_key_;
    const uint8_t buffer_offset_;               ///< Number of bytes to skip from the beginning of the buffer space
public:
    MultiFrameIncomingTransfer(const RxFrame& last_frame, uint8_t buffer_offset, ITransferBufferManager* bufmgr,
                               const TransferBufferManagerKey& bufmgr_key);
    int read(unsigned int offset, uint8_t* data, unsigned int len) const;
    void release() { bufmgr_->remove(bufmgr_key_); }
};

/**
 * Transport internal, for dispatcher.
 */
class TransferListenerBase : public LinkedListNode<TransferListenerBase>
{
    const Crc16 crc_base_;     ///< Pre-initialized with data type hash, thus constant

protected:
    TransferListenerBase(const DataTypeDescriptor* data_type)
    : crc_base_(data_type->hash.value, DataTypeHash::NUM_BYTES)
    {
        assert(data_type);
    }

    virtual ~TransferListenerBase() { }

    void handleReception(TransferReceiver& receiver, const RxFrame& frame);

    virtual void handleIncomingTransfer(IncomingTransfer& transfer) = 0;

public:
    virtual void handleFrame(const RxFrame& frame) = 0;
    virtual void cleanup(uint64_t ts_monotonic) = 0;
};

/**
 * This class should be derived by transfer receivers (subscribers, servers, callers).
 */
template <unsigned int STATIC_BUF_SIZE, unsigned int NUM_STATIC_BUFS>
class TransferListener : protected TransferListenerBase, Noncopyable
{
    TransferBufferManager<STATIC_BUF_SIZE, NUM_STATIC_BUFS> bufmgr_;
    Map<TransferBufferManagerKey, TransferReceiver, NUM_STATIC_BUFS> receivers_;

    void handleFrame(const RxFrame& frame)
    {
        // TODO
    }

    struct TimedOutReceiverPredicate
    {
        const uint64_t ts_monotonic;
        TimedOutReceiverPredicate(uint64_t ts_monotonic) : ts_monotonic(ts_monotonic) { }
        bool operator()(const TransferBufferManagerKey& key, const TransferReceiver& value) const
        {
            (void)key;
            return value.isTimedOut(ts_monotonic);
        }
    };

    void cleanup(uint64_t ts_monotonic)
    {
        receivers_.removeWhere(TimedOutReceiverPredicate(ts_monotonic));
    }

public:
    TransferListener(const DataTypeDescriptor* data_type)
    : TransferListenerBase(data_type)
    { }

    ~TransferListener()
    {
        // Map must be cleared before bufmgr is destructed
        receivers_.removeAll();
    }
};

}
