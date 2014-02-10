/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/internal/transport/transfer.hpp>
#include <uavcan/internal/transport/transfer_buffer.hpp>

namespace uavcan
{

class TransferReceiver
{
public:
    enum ResultCode { RESULT_NOT_COMPLETE, RESULT_COMPLETE, RESULT_SINGLE_FRAME };

    static const uint64_t DEFAULT_TRANSFER_INTERVAL = 500 * 1000;
    static const uint64_t MIN_TRANSFER_INTERVAL     = 1   * 1000;
    static const uint64_t MAX_TRANSFER_INTERVAL     = 10  * 1000 * 1000;

private:
    enum TidRelation { TID_SAME, TID_REPEAT, TID_FUTURE };
    enum { IFACE_INDEX_NOTSET = 0xFF };

    uint64_t prev_transfer_timestamp_;
    uint64_t this_transfer_timestamp_;
    uint64_t transfer_interval_;
    ITransferBufferManager* bufmgr_;
    TransferID tid_;
    uint8_t node_id_;
    uint8_t iface_index_;
    uint8_t next_frame_index_;

    bool isInitialized() const { return iface_index_ != IFACE_INDEX_NOTSET; }

    void cleanup();

    TidRelation getTidRelation(const RxFrame& frame) const;

    void updateTransferTimings();
    void prepareForNextTransfer();

    bool validate(const RxFrame& frame) const;
    ResultCode receive(const RxFrame& frame);

    TransferReceiver(const TransferReceiver&); // = delete (not needed)

public:
    TransferReceiver()
    : prev_transfer_timestamp_(0)
    , this_transfer_timestamp_(0)
    , transfer_interval_(DEFAULT_TRANSFER_INTERVAL)
    , bufmgr_(NULL)
    , node_id_(NODE_ID_INVALID)
    , iface_index_(IFACE_INDEX_NOTSET)
    , next_frame_index_(0)
    { }

    TransferReceiver(ITransferBufferManager* bufmgr, uint8_t node_id)
    : prev_transfer_timestamp_(0)
    , this_transfer_timestamp_(0)
    , transfer_interval_(DEFAULT_TRANSFER_INTERVAL)
    , bufmgr_(bufmgr)
    , node_id_(node_id)
    , iface_index_(IFACE_INDEX_NOTSET)
    , next_frame_index_(0)
    {
        assert(bufmgr);
        assert(node_id <= NODE_ID_MAX);
        assert(node_id != NODE_ID_INVALID);
        assert(node_id != NODE_ID_BROADCAST);
    }

    ~TransferReceiver() { cleanup(); }

    TransferReceiver& operator=(const TransferReceiver& rhs)
    {
        cleanup();
        prev_transfer_timestamp_ = rhs.prev_transfer_timestamp_;
        this_transfer_timestamp_ = rhs.this_transfer_timestamp_;
        transfer_interval_ = rhs.transfer_interval_;
        bufmgr_ = rhs.bufmgr_;
        tid_ = rhs.tid_;
        node_id_ = rhs.node_id_;
        iface_index_ = rhs.iface_index_;
        next_frame_index_ = rhs.next_frame_index_;
        return *this;
    }

    bool isTimedOut(uint64_t timestamp) const;

    ResultCode addFrame(const RxFrame& frame);

    uint64_t getLastTransferTimestamp() const { return prev_transfer_timestamp_; }

    uint64_t getInterval() const { return transfer_interval_; }
};

}
