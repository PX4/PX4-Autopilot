/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cstdlib>
#include <uavcan/build_config.hpp>
#include <uavcan/transport/frame.hpp>
#include <uavcan/transport/transfer_buffer.hpp>

namespace uavcan
{

UAVCAN_PACKED_BEGIN
class UAVCAN_EXPORT TransferReceiver
{
public:
    enum ResultCode { ResultNotComplete, ResultComplete, ResultSingleFrame };

    static const uint32_t MinTransferIntervalUSec     = 1   * 1000UL;
    static const uint32_t MaxTransferIntervalUSec     = 10  * 1000 * 1000UL;
    static const uint32_t DefaultTransferIntervalUSec = 1   * 1000 * 1000UL;

    static MonotonicDuration getDefaultTransferInterval()
    {
        return MonotonicDuration::fromUSec(DefaultTransferIntervalUSec);
    }
    static MonotonicDuration getMinTransferInterval() { return MonotonicDuration::fromUSec(MinTransferIntervalUSec); }
    static MonotonicDuration getMaxTransferInterval() { return MonotonicDuration::fromUSec(MaxTransferIntervalUSec); }

private:
    enum TidRelation { TidSame, TidRepeat, TidFuture };
    static const uint8_t IfaceIndexNotSet = 0xFF;

    MonotonicTime prev_transfer_ts_;
    MonotonicTime this_transfer_ts_;
    UtcTime first_frame_ts_;
    uint32_t transfer_interval_usec_;
    uint16_t this_transfer_crc_;
    uint16_t buffer_write_pos_;
    TransferID tid_;
    uint8_t iface_index_;
    uint8_t next_frame_index_;
    mutable uint8_t error_cnt_;

    bool isInitialized() const { return iface_index_ != IfaceIndexNotSet; }

    void registerError() const;

    TidRelation getTidRelation(const RxFrame& frame) const;

    void updateTransferTimings();
    void prepareForNextTransfer();

    bool validate(const RxFrame& frame) const;
    bool writePayload(const RxFrame& frame, ITransferBuffer& buf);
    ResultCode receive(const RxFrame& frame, TransferBufferAccessor& tba);

public:
    TransferReceiver()
        : transfer_interval_usec_(DefaultTransferIntervalUSec)
        , this_transfer_crc_(0)
        , buffer_write_pos_(0)
        , iface_index_(IfaceIndexNotSet)
        , next_frame_index_(0)
        , error_cnt_(0)
    { }

    bool isTimedOut(MonotonicTime current_ts) const;

    ResultCode addFrame(const RxFrame& frame, TransferBufferAccessor& tba);

    uint8_t yieldErrorCount();

    MonotonicTime getLastTransferTimestampMonotonic() const { return prev_transfer_ts_; }
    UtcTime getLastTransferTimestampUtc() const { return first_frame_ts_; }

    uint16_t getLastTransferCrc() const { return this_transfer_crc_; }

    MonotonicDuration getInterval() const { return MonotonicDuration::fromUSec(transfer_interval_usec_); }
};
UAVCAN_PACKED_END

}
