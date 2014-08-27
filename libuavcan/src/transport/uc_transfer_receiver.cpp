/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/transport/transfer_receiver.hpp>
#include <uavcan/transport/crc.hpp>
#include <uavcan/debug.hpp>
#include <cstdlib>
#include <cassert>

namespace uavcan
{

const uint32_t TransferReceiver::MinTransferIntervalUSec;
const uint32_t TransferReceiver::MaxTransferIntervalUSec;
const uint32_t TransferReceiver::DefaultTransferIntervalUSec;
const uint8_t TransferReceiver::IfaceIndexNotSet;

void TransferReceiver::registerError() const
{
    if (error_cnt_ < 0xFF)
    {
        error_cnt_ = static_cast<uint8_t>(error_cnt_ + 1);
    }
    else
    {
        UAVCAN_ASSERT(0);
    }
}

TransferReceiver::TidRelation TransferReceiver::getTidRelation(const RxFrame& frame) const
{
    const int distance = tid_.computeForwardDistance(frame.getTransferID());
    if (distance == 0)
    {
        return TidSame;
    }
    if (distance < ((1 << TransferID::BitLen) / 2))
    {
        return TidFuture;
    }
    return TidRepeat;
}

void TransferReceiver::updateTransferTimings()
{
    UAVCAN_ASSERT(!this_transfer_ts_.isZero());

    const MonotonicTime prev_prev_ts = prev_transfer_ts_;
    prev_transfer_ts_ = this_transfer_ts_;

    if ((!prev_prev_ts.isZero()) && (!prev_transfer_ts_.isZero()) && (prev_transfer_ts_ >= prev_prev_ts))
    {
        uint64_t interval_usec = uint64_t((prev_transfer_ts_ - prev_prev_ts).toUSec());
        interval_usec = min(interval_usec, uint64_t(MaxTransferIntervalUSec));
        interval_usec = max(interval_usec, uint64_t(MinTransferIntervalUSec));
        transfer_interval_usec_ = static_cast<uint32_t>((uint64_t(transfer_interval_usec_) * 7 + interval_usec) / 8);
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
    {
        return false;
    }
    if (frame.isFirst() && !frame.isLast() && (frame.getPayloadLen() < TransferCRC::NumBytes))
    {
        UAVCAN_TRACE("TransferReceiver", "CRC expected, %s", frame.toString().c_str());
        registerError();
        return false;
    }
    if ((frame.getIndex() == Frame::MaxIndex) && !frame.isLast())
    {
        UAVCAN_TRACE("TransferReceiver", "Unterminated transfer, %s", frame.toString().c_str());
        registerError();
        return false;
    }
    if (frame.getIndex() != next_frame_index_)
    {
        UAVCAN_TRACE("TransferReceiver", "Unexpected frame index (not %i), %s",
                     int(next_frame_index_), frame.toString().c_str());
        registerError();
        return false;
    }
    if (getTidRelation(frame) != TidSame)
    {
        UAVCAN_TRACE("TransferReceiver", "Unexpected TID (current %i), %s", tid_.get(), frame.toString().c_str());
        registerError();
        return false;
    }
    return true;
}

bool TransferReceiver::writePayload(const RxFrame& frame, ITransferBuffer& buf)
{
    const uint8_t* const payload = frame.getPayloadPtr();
    const unsigned payload_len = frame.getPayloadLen();

    if (frame.isFirst())     // First frame contains CRC, we need to extract it now
    {
        if (frame.getPayloadLen() < TransferCRC::NumBytes)
        {
            return false;    // Must have been validated earlier though. I think I'm paranoid.
        }
        this_transfer_crc_ = static_cast<uint16_t>(payload[0] & 0xFF);
        this_transfer_crc_ |= static_cast<uint16_t>(static_cast<uint16_t>(payload[1] & 0xFF) << 8);  // Little endian.

        const unsigned effective_payload_len = payload_len - TransferCRC::NumBytes;
        const int res = buf.write(buffer_write_pos_, payload + TransferCRC::NumBytes, effective_payload_len);
        const bool success = res == static_cast<int>(effective_payload_len);
        if (success)
        {
            buffer_write_pos_ = static_cast<uint16_t>(buffer_write_pos_ + effective_payload_len);
        }
        return success;
    }
    else
    {
        const int res = buf.write(buffer_write_pos_, payload, payload_len);
        const bool success = res == static_cast<int>(payload_len);
        if (success)
        {
            buffer_write_pos_ = static_cast<uint16_t>(buffer_write_pos_ + payload_len);
        }
        return success;
    }
}

TransferReceiver::ResultCode TransferReceiver::receive(const RxFrame& frame, TransferBufferAccessor& tba)
{
    // Transfer timestamps are derived from the first frame
    if (frame.isFirst())
    {
        this_transfer_ts_ = frame.getMonotonicTimestamp();
        first_frame_ts_   = frame.getUtcTimestamp();
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
    {
        buf = tba.create();
    }
    if (buf == NULL)
    {
        UAVCAN_TRACE("TransferReceiver", "Failed to access the buffer, %s", frame.toString().c_str());
        prepareForNextTransfer();
        registerError();
        return ResultNotComplete;
    }
    if (!writePayload(frame, *buf))
    {
        UAVCAN_TRACE("TransferReceiver", "Payload write failed, %s", frame.toString().c_str());
        tba.remove();
        prepareForNextTransfer();
        registerError();
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

bool TransferReceiver::isTimedOut(MonotonicTime current_ts) const
{
    static const int64_t INTERVAL_MULT = (1 << TransferID::BitLen) / 2 + 1;
    if (current_ts <= this_transfer_ts_)
    {
        return false;
    }
    return (current_ts - this_transfer_ts_).toUSec() > (int64_t(transfer_interval_usec_) * INTERVAL_MULT);
}

TransferReceiver::ResultCode TransferReceiver::addFrame(const RxFrame& frame, TransferBufferAccessor& tba)
{
    if ((frame.getMonotonicTimestamp().isZero()) ||
        (frame.getMonotonicTimestamp() < prev_transfer_ts_) ||
        (frame.getMonotonicTimestamp() < this_transfer_ts_))
    {
        return ResultNotComplete;
    }

    const bool not_initialized = !isInitialized();
    const bool receiver_timed_out = isTimedOut(frame.getMonotonicTimestamp());
    const bool same_iface = frame.getIfaceIndex() == iface_index_;
    const bool first_fame = frame.isFirst();
    const TidRelation tid_rel = getTidRelation(frame);
    const bool iface_timed_out =
        (frame.getMonotonicTimestamp() - this_transfer_ts_).toUSec() > (int64_t(transfer_interval_usec_) * 2);

    // FSM, the hard way
    const bool need_restart =
        (not_initialized) ||
        (receiver_timed_out) ||
        (same_iface && first_fame && (tid_rel == TidFuture)) ||
        (iface_timed_out && first_fame && (tid_rel == TidFuture));

    if (need_restart)
    {
        const bool error = !not_initialized && !receiver_timed_out;
        if (error)
        {
            registerError();
        }
        UAVCAN_TRACE("TransferReceiver",
                     "Restart [not_inited=%i, iface_timeout=%i, recv_timeout=%i, same_iface=%i, first_frame=%i, tid_rel=%i], %s",
                     int(not_initialized), int(iface_timed_out), int(receiver_timed_out), int(same_iface),
                     int(first_fame), int(tid_rel), frame.toString().c_str());
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
    {
        return ResultNotComplete;
    }
    return receive(frame, tba);
}

uint8_t TransferReceiver::yieldErrorCount()
{
    const uint8_t ret = error_cnt_;
    error_cnt_ = 0;
    return ret;
}

}
