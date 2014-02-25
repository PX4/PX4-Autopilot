/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdlib>
#include <cstdio>
#include <cassert>
#include <algorithm>
#include <uavcan/internal/debug.hpp>
#include <uavcan/internal/transport/transfer_receiver.hpp>
#include <uavcan/internal/transport/crc.hpp>

namespace uavcan
{

const uint32_t TransferReceiver::DefaultTransferInterval;
const uint32_t TransferReceiver::MinTransferInterval;
const uint32_t TransferReceiver::MaxTransferInterval;

TransferReceiver::TidRelation TransferReceiver::getTidRelation(const RxFrame& frame) const
{
    const int distance = tid_.forwardDistance(frame.getTransferID());
    if (distance == 0)
        return TidSame;
    if (distance < ((1 << TransferID::BitLen) / 2))
        return TidFuture;
    return TidRepeat;
}

void TransferReceiver::updateTransferTimings()
{
    assert(this_transfer_ts_monotonic_ > 0);

    const uint64_t prev_prev_ts = prev_transfer_ts_monotonic_;
    prev_transfer_ts_monotonic_ = this_transfer_ts_monotonic_;

    if ((prev_prev_ts != 0) &&
        (prev_transfer_ts_monotonic_ != 0) &&
        (prev_transfer_ts_monotonic_ >= prev_prev_ts))
    {
        uint64_t interval = prev_transfer_ts_monotonic_ - prev_prev_ts;
        interval = std::max(std::min(interval, uint64_t(MaxTransferInterval)), uint64_t(MinTransferInterval));
        transfer_interval_ = static_cast<uint32_t>((uint64_t(transfer_interval_) * 7 + interval) / 8);
    }
}

void TransferReceiver::prepareForNextTransfer()
{
    tid_.increment();
    next_frame_index_ = 0;
    buffer_write_pos_ = 0;
}

bool TransferReceiver::validate(const RxFrame& frame) const
{
    if (iface_index_ != frame.getIfaceIndex())
        return false;

    if (frame.isFirst() && !frame.isLast() && (frame.getPayloadLen() < TransportCRC::NumBytes))
    {
        UAVCAN_TRACE("TransferReceiver", "CRC expected, %s", frame.toString().c_str());
        return false;
    }

    if ((frame.getIndex() == Frame::MaxIndex) && !frame.isLast())
    {
        UAVCAN_TRACE("TransferReceiver", "Unterminated transfer, %s", frame.toString().c_str());
        return false;
    }

    if (frame.getIndex() != next_frame_index_)
    {
        UAVCAN_TRACE("TransferReceiver", "Unexpected frame index (not %i), %s",
                     int(next_frame_index_), frame.toString().c_str());
        return false;
    }

    if (getTidRelation(frame) != TidSame)
    {
        UAVCAN_TRACE("TransferReceiver", "Unexpected TID (current %i), %s", tid_.get(), frame.toString().c_str());
        return false;
    }
    return true;
}

bool TransferReceiver::writePayload(const RxFrame& frame, ITransferBuffer& buf)
{
    const uint8_t* const payload = frame.getPayloadPtr();
    const int payload_len = frame.getPayloadLen();

    if (frame.isFirst())                  // First frame contains CRC, we need to extract it now
    {
        if (frame.getPayloadLen() < TransportCRC::NumBytes) // Must have been validated earlier though. I think I'm paranoid.
            return false;

        this_transfer_crc_ = (payload[0] & 0xFF) | (uint16_t(payload[1] & 0xFF) << 8); // Little endian.

        const int effective_payload_len = payload_len - TransportCRC::NumBytes;
        const int res = buf.write(buffer_write_pos_, payload + TransportCRC::NumBytes, effective_payload_len);
        const bool success = res == effective_payload_len;
        if (success)
            buffer_write_pos_ += effective_payload_len;
        return success;
    }
    else
    {
        const int res = buf.write(buffer_write_pos_, payload, payload_len);
        const bool success = res == payload_len;
        if (success)
            buffer_write_pos_ += payload_len;
        return success;
    }
}

TransferReceiver::ResultCode TransferReceiver::receive(const RxFrame& frame, TransferBufferAccessor& tba)
{
    // Transfer timestamps are derived from the first frame
    if (frame.isFirst())
    {
        this_transfer_ts_monotonic_ = frame.getMonotonicTimestamp();
        first_frame_ts_utc_         = frame.getUtcTimestamp();
    }

    if (frame.isFirst() && frame.isLast())
    {
        tba.remove();
        updateTransferTimings();
        prepareForNextTransfer();
        this_transfer_crc_ = 0;         // SFT has no CRC
        return ResultSingleFrame;
    }

    // Payload write
    ITransferBuffer* buf = tba.access();
    if (buf == NULL)
        buf = tba.create();
    if (buf == NULL)
    {
        UAVCAN_TRACE("TransferReceiver", "Failed to access the buffer, %s", frame.toString().c_str());
        prepareForNextTransfer();
        return ResultNotComplete;
    }
    if (!writePayload(frame, *buf))
    {
        UAVCAN_TRACE("TransferReceiver", "Payload write failed, %s", frame.toString().c_str());
        tba.remove();
        prepareForNextTransfer();
        return ResultNotComplete;
    }
    next_frame_index_++;

    if (frame.isLast())
    {
        updateTransferTimings();
        prepareForNextTransfer();
        return ResultComplete;
    }
    return ResultNotComplete;
}

bool TransferReceiver::isTimedOut(uint64_t ts_monotonic) const
{
    static const uint64_t INTERVAL_MULT = (1 << TransferID::BitLen) / 2 + 1;
    const uint64_t ts = this_transfer_ts_monotonic_;
    if (ts_monotonic <= ts)
        return false;
    return (ts_monotonic - ts) > (uint64_t(transfer_interval_) * INTERVAL_MULT);
}

TransferReceiver::ResultCode TransferReceiver::addFrame(const RxFrame& frame, TransferBufferAccessor& tba)
{
    if ((frame.getMonotonicTimestamp() == 0) ||
        (frame.getMonotonicTimestamp() < prev_transfer_ts_monotonic_) ||
        (frame.getMonotonicTimestamp() < this_transfer_ts_monotonic_))
    {
        return ResultNotComplete;
    }

    const bool not_initialized = !isInitialized();
    const bool receiver_timed_out = isTimedOut(frame.getMonotonicTimestamp());
    const bool same_iface = frame.getIfaceIndex() == iface_index_;
    const bool first_fame = frame.isFirst();
    const TidRelation tid_rel = getTidRelation(frame);
    const bool iface_timed_out =
        (frame.getMonotonicTimestamp() - this_transfer_ts_monotonic_) > (uint64_t(transfer_interval_) * 2);

    const bool need_restart = // FSM, the hard way
        (not_initialized) ||
        (receiver_timed_out) ||
        (same_iface && first_fame && (tid_rel == TidFuture)) ||
        (iface_timed_out && first_fame && (tid_rel == TidFuture));

    if (need_restart)
    {
        UAVCAN_TRACE("TransferReceiver",
            "Restart [not_inited=%i, iface_timeout=%i, recv_timeout=%i, same_iface=%i, first_frame=%i, tid_rel=%i], %s",
            int(not_initialized), int(iface_timed_out), int(receiver_timed_out), int(same_iface), int(first_fame),
            int(tid_rel), frame.toString().c_str());
        tba.remove();
        iface_index_ = frame.getIfaceIndex();
        tid_ = frame.getTransferID();
        next_frame_index_ = 0;
        buffer_write_pos_ = 0;
        this_transfer_crc_ = 0;
        if (!first_fame)
        {
            tid_.increment();
            return ResultNotComplete;
        }
    }

    if (!validate(frame))
        return ResultNotComplete;

    return receive(frame, tba);
}

}
