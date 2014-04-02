/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <cstdio>
#include <sstream>
#include <uavcan/transport/frame.hpp>
#include <uavcan/transport/can_io.hpp>

namespace uavcan
{
/**
 * Frame
 */
int Frame::getMaxPayloadLen() const
{
    switch (getTransferType())
    {
    case TransferTypeMessageBroadcast:
    {
        return sizeof(payload_);
    }
    case TransferTypeServiceResponse:
    case TransferTypeServiceRequest:
    case TransferTypeMessageUnicast:
    {
        return sizeof(payload_) - 1;
    }
    default:
        assert(0);
        return -ErrLogic;
    }
}

int Frame::setPayload(const uint8_t* data, unsigned len)
{
    const int maxlen = getMaxPayloadLen();
    if (maxlen < 0)
    {
        return maxlen;
    }
    len = std::min(unsigned(maxlen), len);
    std::copy(data, data + len, payload_);
    payload_len_ = len;
    return len;
}

template <int OFFSET, int WIDTH>
inline static uint32_t bitunpack(uint32_t val)
{
    return (val >> OFFSET) & ((1UL << WIDTH) - 1);
}

bool Frame::parse(const CanFrame& can_frame)
{
    if (can_frame.isErrorFrame() || can_frame.isRemoteTransmissionRequest() || !can_frame.isExtended())
    {
        return false;
    }

    if (can_frame.dlc > sizeof(CanFrame::data))
    {
        assert(0);  // This is not a protocol error, so assert() is ok
        return false;
    }

    /*
     * CAN ID parsing
     */
    const uint32_t id = can_frame.id & CanFrame::MaskExtID;
    transfer_id_   = bitunpack<0, 3>(id);
    last_frame_    = bitunpack<3, 1>(id);
    frame_index_   = bitunpack<4, 6>(id);
    src_node_id_   = bitunpack<10, 7>(id);
    transfer_type_ = TransferType(bitunpack<17, 2>(id));
    data_type_id_  = bitunpack<19, 10>(id);

    /*
     * CAN payload parsing
     */
    switch (transfer_type_)
    {
    case TransferTypeMessageBroadcast:
    {
        dst_node_id_ = NodeID::Broadcast;
        payload_len_ = can_frame.dlc;
        std::copy(can_frame.data, can_frame.data + can_frame.dlc, payload_);
        break;
    }

    case TransferTypeServiceResponse:
    case TransferTypeServiceRequest:
    case TransferTypeMessageUnicast:
    {
        if (can_frame.dlc < 1)
        {
            return false;
        }
        if (can_frame.data[0] & 0x80)     // RESERVED, must be zero
        {
            return false;
        }
        dst_node_id_ = can_frame.data[0] & 0x7F;
        payload_len_ = can_frame.dlc - 1;
        std::copy(can_frame.data + 1, can_frame.data + can_frame.dlc, payload_);
        break;
    }

    default:
        return false;
    }

    return isValid();
}

template <int OFFSET, int WIDTH>
inline static uint32_t bitpack(uint32_t field)
{
    return (field & ((1UL << WIDTH) - 1)) << OFFSET;
}

bool Frame::compile(CanFrame& out_can_frame) const
{
    if (!isValid())
    {
        assert(0);        // This is an application error, so we need to maximize it.
        return false;
    }

    out_can_frame.id =
        CanFrame::FlagEFF |
        bitpack<0, 3>(transfer_id_.get()) |
        bitpack<3, 1>(last_frame_) |
        bitpack<4, 6>(frame_index_) |
        bitpack<10, 7>(src_node_id_.get()) |
        bitpack<17, 2>(transfer_type_) |
        bitpack<19, 10>(data_type_id_.get());

    switch (transfer_type_)
    {
    case TransferTypeMessageBroadcast:
    {
        out_can_frame.dlc = payload_len_;
        std::copy(payload_, payload_ + payload_len_, out_can_frame.data);
        break;
    }

    case TransferTypeServiceResponse:
    case TransferTypeServiceRequest:
    case TransferTypeMessageUnicast:
    {
        assert((payload_len_ + 1) <= sizeof(CanFrame::data));
        out_can_frame.data[0] = dst_node_id_.get();
        out_can_frame.dlc = payload_len_ + 1;
        std::copy(payload_, payload_ + payload_len_, out_can_frame.data + 1);
        break;
    }

    default:
        assert(0);
        return false;
    }
    return true;
}

bool Frame::isValid() const
{
    // Refer to the specification for the detailed explanation of the checks
    const bool invalid =
        (frame_index_ > MaxIndex) ||
        ((frame_index_ == MaxIndex) && !last_frame_) ||
        (!src_node_id_.isUnicast()) ||
        (!dst_node_id_.isValid()) ||
        (src_node_id_ == dst_node_id_) ||
        ((transfer_type_ == TransferTypeMessageBroadcast) != dst_node_id_.isBroadcast()) ||
        (transfer_type_ >= NumTransferTypes) ||
        (static_cast<int>(payload_len_) > getMaxPayloadLen()) ||
        (!data_type_id_.isValid());

    return !invalid;
}

bool Frame::operator==(const Frame& rhs) const
{
    return
        (transfer_type_ == rhs.transfer_type_) &&
        (data_type_id_  == rhs.data_type_id_) &&
        (src_node_id_   == rhs.src_node_id_) &&
        (dst_node_id_   == rhs.dst_node_id_) &&
        (frame_index_   == rhs.frame_index_) &&
        (transfer_id_   == rhs.transfer_id_) &&
        (last_frame_    == rhs.last_frame_) &&
        (payload_len_   == rhs.payload_len_) &&
        std::equal(payload_, payload_ + payload_len_, rhs.payload_);
}

std::string Frame::toString() const
{
    using namespace std; // For snprintf()
    /*
     * Frame ID fields, according to UAVCAN specs:
     *  - Data Type ID
     *  - Transfer Type
     *  - Source Node ID
     *  - Frame Index
     *  - Last Frame
     *  - Transfer ID
     */
    static const int BUFLEN = 100;
    char buf[BUFLEN];
    int ofs = snprintf(buf, BUFLEN, "dtid=%i tt=%i snid=%i dnid=%i idx=%i last=%i tid=%i payload=[",
                       int(data_type_id_.get()), int(transfer_type_), int(src_node_id_.get()),
                       int(dst_node_id_.get()), int(frame_index_), int(last_frame_), int(transfer_id_.get()));

    for (unsigned i = 0; i < payload_len_; i++)
    {
        ofs += snprintf(buf + ofs, BUFLEN - ofs, "%02x", payload_[i]);
        if ((i + 1) < payload_len_)
        {
            ofs += snprintf(buf + ofs, BUFLEN - ofs, " ");
        }
    }
    ofs += snprintf(buf + ofs, BUFLEN - ofs, "]");
    return std::string(buf);
}

/**
 * RxFrame
 */
bool RxFrame::parse(const CanRxFrame& can_frame)
{
    if (!Frame::parse(can_frame))
    {
        return false;
    }
    if (can_frame.ts_mono.isZero())  // Monotonic timestamps are mandatory.
    {
        assert(0);                   // If it is not set, it's a driver failure.
        return false;
    }
    ts_mono_ = can_frame.ts_mono;
    ts_utc_ = can_frame.ts_utc;
    iface_index_ = can_frame.iface_index;
    return true;
}

std::string RxFrame::toString() const
{
    std::ostringstream os;  // C++03 doesn't support long long, so we need ostream to print the timestamp
    os << Frame::toString() << " ts_m=" << ts_mono_ << " ts_utc=" << ts_utc_ << " iface=" << int(iface_index_);
    return os.str();
}

}
