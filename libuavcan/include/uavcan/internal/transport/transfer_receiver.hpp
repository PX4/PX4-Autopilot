/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cstdlib>
#include <uavcan/internal/transport/transfer.hpp>
#include <uavcan/internal/transport/transfer_buffer.hpp>

namespace uavcan
{

UAVCAN_PACKED_BEGIN
class TransferReceiver
{
public:
    enum ResultCode { ResultNotComplete, ResultComplete, ResultSingleFrame };

    static const uint32_t DefaultTransferInterval = 500 * 1000UL;
    static const uint32_t MinTransferInterval     = 1   * 1000UL;
    static const uint32_t MaxTransferInterval     = 10  * 1000 * 1000UL;

private:
    enum TidRelation { TidSame, TidRepeat, TidFuture };
    enum { IfaceIndexNotSet = 0xFF };

    uint64_t prev_transfer_ts_monotonic_;
    uint64_t this_transfer_ts_monotonic_;
    uint64_t first_frame_ts_utc_;
    uint32_t transfer_interval_;
    uint16_t this_transfer_crc_;
    uint16_t buffer_write_pos_;
    TransferID tid_;
    uint8_t iface_index_;
    uint8_t next_frame_index_;

    bool isInitialized() const { return iface_index_ != IfaceIndexNotSet; }

    TidRelation getTidRelation(const RxFrame& frame) const;

    void updateTransferTimings();
    void prepareForNextTransfer();

    bool validate(const RxFrame& frame) const;
    bool writePayload(const RxFrame& frame, ITransferBuffer& buf);
    ResultCode receive(const RxFrame& frame, TransferBufferAccessor& tba);

public:
    TransferReceiver()
    : prev_transfer_ts_monotonic_(0)
    , this_transfer_ts_monotonic_(0)
    , first_frame_ts_utc_(0)
    , transfer_interval_(DefaultTransferInterval)
    , this_transfer_crc_(0)
    , buffer_write_pos_(0)
    , iface_index_(IfaceIndexNotSet)
    , next_frame_index_(0)
    { }

    bool isTimedOut(uint64_t ts_monotonic) const;

    ResultCode addFrame(const RxFrame& frame, TransferBufferAccessor& tba);

    uint64_t getLastTransferTimestampMonotonic() const { return prev_transfer_ts_monotonic_; }
    uint64_t getLastTransferTimestampUtc() const { return first_frame_ts_utc_; }

    uint16_t getLastTransferCrc() const { return this_transfer_crc_; }

    uint32_t getInterval() const { return transfer_interval_; }
};
UAVCAN_PACKED_END

}
