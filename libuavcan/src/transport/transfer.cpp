/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <cstdio>
#include <sstream>
#include <uavcan/internal/transport/transfer.hpp>

namespace uavcan
{
/**
 * NodeID
 */
const NodeID NodeID::BROADCAST(VALUE_BROADCAST);

/**
 * TransferID
 */
int TransferID::forwardDistance(TransferID rhs) const
{
    int d = int(rhs.get()) - int(get());
    if (d < 0)
        d += 1 << BITLEN;

    assert(((get() + d) & MAX) == rhs.get());
    return d;
}

/**
 * Frame
 */
int Frame::getMaxPayloadLen() const
{
    switch (getTransferType())
    {
    case TRANSFER_TYPE_MESSAGE_BROADCAST:
        return sizeof(payload_);
        break;

    case TRANSFER_TYPE_SERVICE_RESPONSE:
    case TRANSFER_TYPE_SERVICE_REQUEST:
    case TRANSFER_TYPE_MESSAGE_UNICAST:
        return sizeof(payload_) - 1;
        break;

    default:
        assert(0);
        return -1;
    }
}

int Frame::setPayload(const uint8_t* data, int len)
{
    len = std::min(getMaxPayloadLen(), len);
    if (len >= 0)
    {
        std::copy(data, data + len, payload_);
        payload_len_ = len;
    }
    return len;
}

template <int OFFSET, int WIDTH>
inline static uint32_t bitunpack(uint32_t val)
{
    return (val >> OFFSET) & ((1UL << WIDTH) - 1);
}

bool Frame::parse(const CanFrame& can_frame)
{
    if ((can_frame.id & CanFrame::FLAG_RTR) || !(can_frame.id & CanFrame::FLAG_EFF))
        return false;

    if (can_frame.dlc > sizeof(CanFrame::data))
    {
        assert(0);  // This is not a protocol error, so assert() is ok
        return false;
    }

    /*
     * CAN ID parsing
     */
    const uint32_t id = can_frame.id & CanFrame::MASK_EXTID;
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
    case TRANSFER_TYPE_MESSAGE_BROADCAST:
        dst_node_id_ = NodeID::BROADCAST;
        payload_len_ = can_frame.dlc;
        std::copy(can_frame.data, can_frame.data + can_frame.dlc, payload_);
        break;

    case TRANSFER_TYPE_SERVICE_RESPONSE:
    case TRANSFER_TYPE_SERVICE_REQUEST:
    case TRANSFER_TYPE_MESSAGE_UNICAST:
        if (can_frame.dlc < 1)
            return false;
        if (can_frame.data[0] & 0x80)     // RESERVED, must be zero
            return false;
        dst_node_id_ = can_frame.data[0] & 0x7F;
        payload_len_ = can_frame.dlc - 1;
        std::copy(can_frame.data + 1, can_frame.data + can_frame.dlc, payload_);
        break;

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

    out_can_frame.id = CanFrame::FLAG_EFF |
        bitpack<0, 3>(transfer_id_.get()) |
        bitpack<3, 1>(last_frame_) |
        bitpack<4, 6>(frame_index_) |
        bitpack<10, 7>(src_node_id_.get()) |
        bitpack<17, 2>(transfer_type_) |
        bitpack<19, 10>(data_type_id_);

    switch (transfer_type_)
    {
    case TRANSFER_TYPE_MESSAGE_BROADCAST:
        out_can_frame.dlc = payload_len_;
        std::copy(payload_, payload_ + payload_len_, out_can_frame.data);
        break;

    case TRANSFER_TYPE_SERVICE_RESPONSE:
    case TRANSFER_TYPE_SERVICE_REQUEST:
    case TRANSFER_TYPE_MESSAGE_UNICAST:
        assert((payload_len_ + 1) <= sizeof(CanFrame::data));
        out_can_frame.data[0] = dst_node_id_.get();
        out_can_frame.dlc = payload_len_ + 1;
        std::copy(payload_, payload_ + payload_len_, out_can_frame.data + 1);
        break;

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
        (frame_index_ > INDEX_MAX) ||
        ((frame_index_ == INDEX_MAX) && !last_frame_) ||
        (!src_node_id_.isUnicast()) ||
        (!dst_node_id_.isValid()) ||
        (src_node_id_ == dst_node_id_) ||
        ((transfer_type_ == TRANSFER_TYPE_MESSAGE_BROADCAST) != dst_node_id_.isBroadcast()) ||
        (transfer_type_ >= NUM_TRANSFER_TYPES) ||
        (payload_len_ > getMaxPayloadLen()) ||
        (data_type_id_ > DATA_TYPE_ID_MAX);

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
    int ofs = std::snprintf(buf, BUFLEN, "dtid=%i tt=%i snid=%i dnid=%i idx=%i last=%i tid=%i payload=[",
                            int(data_type_id_), int(transfer_type_), int(src_node_id_.get()), int(dst_node_id_.get()),
                            int(frame_index_), int(last_frame_), int(transfer_id_.get()));

    for (int i = 0; i < payload_len_; i++)
    {
        ofs += std::snprintf(buf + ofs, BUFLEN - ofs, "%02x", payload_[i]);
        if ((i + 1) < payload_len_)
            ofs += std::snprintf(buf + ofs, BUFLEN - ofs, " ");
    }
    ofs += std::snprintf(buf + ofs, BUFLEN - ofs, "]");
    return std::string(buf);
}

/**
 * RxFrame
 */
std::string RxFrame::toString() const
{
    std::ostringstream os;  // C++03 doesn't support long long, so we need ostream to print the timestamp
    os << Frame::toString() << " ts_m=" << ts_monotonic_ << " ts_utc=" << ts_utc_ << " iface=" << int(iface_index_);
    return os.str();
}

}
