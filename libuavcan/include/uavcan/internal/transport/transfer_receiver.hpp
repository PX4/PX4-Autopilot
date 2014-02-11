/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cstdlib>
#include <uavcan/internal/transport/transfer.hpp>
#include <uavcan/internal/transport/transfer_buffer.hpp>

namespace uavcan
{

#pragma pack(push, 1)
class TransferReceiver
{
public:
    enum ResultCode { RESULT_NOT_COMPLETE, RESULT_COMPLETE, RESULT_SINGLE_FRAME };

    static const uint32_t DEFAULT_TRANSFER_INTERVAL = 500 * 1000UL;
    static const uint32_t MIN_TRANSFER_INTERVAL     = 1   * 1000UL;
    static const uint32_t MAX_TRANSFER_INTERVAL     = 10  * 1000 * 1000UL;

private:
    enum TidRelation { TID_SAME, TID_REPEAT, TID_FUTURE };
    enum { IFACE_INDEX_NOTSET = 0xFF };

    uint64_t prev_transfer_ts_monotonic_;
    uint64_t this_transfer_ts_monotonic_;
    uint64_t first_frame_ts_utc_;
    uint32_t transfer_interval_;
    TransferID tid_;
    uint8_t iface_index_;
    uint8_t next_frame_index_;

    bool isInitialized() const { return iface_index_ != IFACE_INDEX_NOTSET; }

    TidRelation getTidRelation(const RxFrame& frame) const;

    void updateTransferTimings();
    void prepareForNextTransfer();

    bool validate(const RxFrame& frame) const;
    ResultCode receive(const RxFrame& frame, TransferBufferAccessor& tba);

    TransferReceiver(const TransferReceiver&); // = delete (not needed)

public:
    TransferReceiver()
    : prev_transfer_ts_monotonic_(0)
    , this_transfer_ts_monotonic_(0)
    , first_frame_ts_utc_(0)
    , transfer_interval_(DEFAULT_TRANSFER_INTERVAL)
    , iface_index_(IFACE_INDEX_NOTSET)
    , next_frame_index_(0)
    { }

    bool isTimedOut(uint64_t ts_monotonic) const;

    ResultCode addFrame(const RxFrame& frame, TransferBufferAccessor& tba);

    uint64_t getLastTransferTimestampMonotonic() const { return prev_transfer_ts_monotonic_; }
    uint64_t getLastTransferTimestampUtc() const { return first_frame_ts_utc_; }

    uint32_t getInterval() const { return transfer_interval_; }
};
#pragma pack(pop)

}
