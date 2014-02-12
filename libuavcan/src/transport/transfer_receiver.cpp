/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cstdlib>
#include <cstdio>
#include <cassert>
#include <algorithm>
#include <uavcan/internal/debug.hpp>
#include <uavcan/internal/transport/transfer_receiver.hpp>

namespace uavcan
{

const uint32_t TransferReceiver::DEFAULT_TRANSFER_INTERVAL;
const uint32_t TransferReceiver::MIN_TRANSFER_INTERVAL;
const uint32_t TransferReceiver::MAX_TRANSFER_INTERVAL;

TransferReceiver::TidRelation TransferReceiver::getTidRelation(const RxFrame& frame) const
{
    const int distance = tid_.forwardDistance(frame.transfer_id);
    if (distance == 0)
        return TID_SAME;
    if (distance < ((1 << TransferID::BITLEN) / 2))
        return TID_FUTURE;
    return TID_REPEAT;
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
        interval = std::max(std::min(interval, uint64_t(MAX_TRANSFER_INTERVAL)), uint64_t(MIN_TRANSFER_INTERVAL));
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
    if (iface_index_ != frame.iface_index)
        return false;

    if (frame.source_node_id == 0)
    {
        UAVCAN_TRACE("TransferReceiver", "Invalid frame NID, %s", frame.toString().c_str());
        return false;
    }

    if (frame.frame_index != next_frame_index_)
    {
        UAVCAN_TRACE("TransferReceiver", "Unexpected frame index, %s", frame.toString().c_str());
        return false;
    }

    if (!frame.last_frame && frame.payload_len != Frame::PAYLOAD_LEN_MAX)
    {
        UAVCAN_TRACE("TransferReceiver", "Unexpected payload len, %s", frame.toString().c_str());
        return false;
    }

    if (!frame.last_frame && frame.frame_index == Frame::FRAME_INDEX_MAX)
    {
        UAVCAN_TRACE("TransferReceiver", "Expected end of transfer, %s", frame.toString().c_str());
        return false;
    }

    if (getTidRelation(frame) != TID_SAME)
    {
        UAVCAN_TRACE("TransferReceiver", "Unexpected TID, %s", frame.toString().c_str());
        return false;
    }
    return true;
}

bool TransferReceiver::writePayload(const RxFrame& frame, TransferBufferBase& buf)
{
    if (frame.frame_index == 0)
    {
        unsigned int payload_offset = 0;
        unsigned int crc_offset = 0;

        switch (frame.transfer_type)
        {
        case TRANSFER_TYPE_MESSAGE_BROADCAST:
            payload_offset = 2;
            crc_offset = 0;
            break;
        case TRANSFER_TYPE_SERVICE_RESPONSE:  // Addressed transfers have 1-byte overhead for Destination Node ID
        case TRANSFER_TYPE_SERVICE_REQUEST:
        case TRANSFER_TYPE_MESSAGE_UNICAST:
            payload_offset = 3;
            crc_offset = 1;
            break;
        default:
            UAVCAN_TRACE("TransferReceiver", "Invalid transfer type, %s", frame.toString().c_str());
            return RESULT_NOT_COMPLETE;
        }

        this_transfer_crc_ =
            (frame.payload[crc_offset] & 0xFF) |
            (uint16_t(frame.payload[crc_offset + 1] & 0xFF) << 8); // little endian

        const int effective_payload_len = frame.payload_len - payload_offset;
        const int res = buf.write(buffer_write_pos_, frame.payload + payload_offset, effective_payload_len);
        const bool success = res == effective_payload_len;
        if (success)
            buffer_write_pos_ += effective_payload_len;
        return success;
    }
    else
    {
        const int res = buf.write(buffer_write_pos_, frame.payload, frame.payload_len);
        const bool success = res == frame.payload_len;
        if (success)
            buffer_write_pos_ += frame.payload_len;
        return success;
    }
}

TransferReceiver::ResultCode TransferReceiver::receive(const RxFrame& frame, TransferBufferAccessor& tba)
{
    // Transfer timestamps are derived from the first frame
    if (frame.frame_index == 0)
    {
        this_transfer_ts_monotonic_ = frame.ts_monotonic;
        first_frame_ts_utc_ = frame.ts_utc;
    }

    // Single-frame transfer
    if ((frame.frame_index == 0) && frame.last_frame)
    {
        tba.remove();
        updateTransferTimings();
        prepareForNextTransfer();
        this_transfer_crc_ = 0;         // SFT has no CRC
        return RESULT_SINGLE_FRAME;
    }

    // Payload write
    TransferBufferBase* buf = tba.access();
    if (buf == NULL)
        buf = tba.create();
    if (buf == NULL)
    {
        UAVCAN_TRACE("TransferReceiver", "Failed to access the buffer, %s", frame.toString().c_str());
        prepareForNextTransfer();
        return RESULT_NOT_COMPLETE;
    }
    if (!writePayload(frame, *buf))
    {
        UAVCAN_TRACE("TransferReceiver", "Payload write failed, %s", frame.toString().c_str());
        tba.remove();
        prepareForNextTransfer();
        return RESULT_NOT_COMPLETE;
    }
    next_frame_index_++;

    if (frame.last_frame)
    {
        updateTransferTimings();
        prepareForNextTransfer();
        return RESULT_COMPLETE;
    }
    return RESULT_NOT_COMPLETE;
}

bool TransferReceiver::isTimedOut(uint64_t ts_monotonic) const
{
    static const uint64_t INTERVAL_MULT = (1 << TransferID::BITLEN) / 2 - 1;
    const uint64_t ts = this_transfer_ts_monotonic_;
    if (ts_monotonic <= ts)
        return false;
    return (ts_monotonic - ts) > (uint64_t(transfer_interval_) * INTERVAL_MULT);
}

TransferReceiver::ResultCode TransferReceiver::addFrame(const RxFrame& frame, TransferBufferAccessor& tba)
{
    if ((frame.ts_monotonic == 0) ||
        (frame.ts_monotonic < prev_transfer_ts_monotonic_) ||
        (frame.ts_monotonic < this_transfer_ts_monotonic_))
    {
        return RESULT_NOT_COMPLETE;
    }

    const bool not_initialized = !isInitialized();
    const bool iface_timed_out = (frame.ts_monotonic - this_transfer_ts_monotonic_) > (uint64_t(transfer_interval_) * 2);
    const bool receiver_timed_out = isTimedOut(frame.ts_monotonic);
    const bool same_iface = frame.iface_index == iface_index_;
    const bool first_fame = frame.frame_index == 0;
    const TidRelation tid_rel = getTidRelation(frame);

    const bool need_restart = // FSM, the hard way
        (not_initialized) ||
        (receiver_timed_out) ||
        (same_iface && first_fame && (tid_rel == TID_FUTURE)) ||
        (iface_timed_out && first_fame && (tid_rel == TID_FUTURE));

    if (need_restart)
    {
        UAVCAN_TRACE("TransferReceiver",
            "Restart [not_inited=%i, iface_timeout=%i, recv_timeout=%i, same_iface=%i, first_frame=%i, tid_rel=%i], %s",
            int(not_initialized), int(iface_timed_out), int(receiver_timed_out), int(same_iface), int(first_fame),
            int(tid_rel), frame.toString().c_str());
        tba.remove();
        iface_index_ = frame.iface_index;
        tid_ = frame.transfer_id;
        next_frame_index_ = 0;
        buffer_write_pos_ = 0;
        this_transfer_crc_ = 0;
        if (!first_fame)
        {
            tid_.increment();
            return RESULT_NOT_COMPLETE;
        }
    }

    if (!validate(frame))
        return RESULT_NOT_COMPLETE;

    return receive(frame, tba);
}

bool TransferReceiver::extractSingleFrameTransferPayload(const RxFrame& frame, uint8_t* out_data,
                                                         unsigned int& out_len)
{
    if (out_data == NULL)
    {
        assert(0);
        return false;
    }

    out_len = 0;
    unsigned int offset = 0;
    switch (frame.transfer_type)
    {
    case TRANSFER_TYPE_MESSAGE_BROADCAST:
        offset = 0;
        break;
    case TRANSFER_TYPE_SERVICE_RESPONSE:  // Addressed transfers have 1-byte overhead for Destination Node ID
    case TRANSFER_TYPE_SERVICE_REQUEST:
    case TRANSFER_TYPE_MESSAGE_UNICAST:
        offset = 1;
        break;
    default:
        return false;
    }

    if (frame.payload_len < offset)
        return false;

    out_len = frame.payload_len - offset;
    std::copy(frame.payload + offset, frame.payload + offset + out_len, out_data);
    return true;
}

}
