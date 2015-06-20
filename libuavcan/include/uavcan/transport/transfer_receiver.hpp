/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_TRANSPORT_TRANSFER_RECEIVER_HPP_INCLUDED
#define UAVCAN_TRANSPORT_TRANSFER_RECEIVER_HPP_INCLUDED

#include <cstdlib>
#include <uavcan/build_config.hpp>
#include <uavcan/transport/frame.hpp>
#include <uavcan/transport/transfer_buffer.hpp>

namespace uavcan
{

class UAVCAN_EXPORT TransferReceiver
{
public:
    enum ResultCode { ResultNotComplete, ResultComplete, ResultSingleFrame };

    static const uint16_t MinTransferIntervalMSec     = 1;
    static const uint16_t MaxTransferIntervalMSec     = 0xFFFF;
    static const uint16_t DefaultTransferIntervalMSec = 1000;

    static MonotonicDuration getDefaultTransferInterval()
    {
        return MonotonicDuration::fromMSec(DefaultTransferIntervalMSec);
    }
    static MonotonicDuration getMinTransferInterval() { return MonotonicDuration::fromMSec(MinTransferIntervalMSec); }
    static MonotonicDuration getMaxTransferInterval() { return MonotonicDuration::fromMSec(MaxTransferIntervalMSec); }

private:
    enum TidRelation { TidSame, TidRepeat, TidFuture };

    enum { IfaceIndexNotSet = MaxCanIfaces };

    enum { BufferWritePosMask = 4095 };
    enum { ErrorCntMask = 15 };
    enum { IfaceIndexMask = MaxCanIfaces };

    MonotonicTime prev_transfer_ts_;
    MonotonicTime this_transfer_ts_;
    UtcTime first_frame_ts_;
    uint16_t transfer_interval_msec_;
    uint16_t this_transfer_crc_;

    // 2 byte aligned bitfields:
    uint16_t buffer_write_pos_  : 12;
    mutable uint16_t error_cnt_ : 4;

    TransferID tid_;    // 1 byte field

    // 1 byte aligned bitfields:
    uint8_t next_frame_index_   : 6;
    uint8_t iface_index_        : 2;

    bool isInitialized() const { return iface_index_ != IfaceIndexNotSet; }

    void registerError() const;

    TidRelation getTidRelation(const RxFrame& frame) const;

    void updateTransferTimings();
    void prepareForNextTransfer();

    bool validate(const RxFrame& frame) const;
    bool writePayload(const RxFrame& frame, ITransferBuffer& buf);
    ResultCode receive(const RxFrame& frame, TransferBufferAccessor& tba);

public:
    TransferReceiver() :
        transfer_interval_msec_(DefaultTransferIntervalMSec),
        this_transfer_crc_(0),
        buffer_write_pos_(0),
        error_cnt_(0),
        next_frame_index_(0),
        iface_index_(IfaceIndexNotSet)
    {
#if UAVCAN_DEBUG
        StaticAssert<sizeof(TransferReceiver) == 32>::check();
#endif
    }

    bool isTimedOut(MonotonicTime current_ts) const;

    ResultCode addFrame(const RxFrame& frame, TransferBufferAccessor& tba);

    uint8_t yieldErrorCount();

    MonotonicTime getLastTransferTimestampMonotonic() const { return prev_transfer_ts_; }
    UtcTime getLastTransferTimestampUtc() const { return first_frame_ts_; }

    uint16_t getLastTransferCrc() const { return this_transfer_crc_; }

    MonotonicDuration getInterval() const { return MonotonicDuration::fromMSec(transfer_interval_msec_); }
};

}

#endif // UAVCAN_TRANSPORT_TRANSFER_RECEIVER_HPP_INCLUDED
