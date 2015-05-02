/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/transport/frame.hpp>
#include <uavcan/transport/can_io.hpp>
#include <uavcan/debug.hpp>
#include <cassert>

namespace uavcan
{
/**
 * Frame
 */
const uint8_t Frame::MaxIndexForService;
const uint8_t Frame::MaxIndexForMessage;

uint_fast8_t Frame::getMaxIndexForTransferType(const TransferType type)
{
    if (type == TransferTypeMessageBroadcast ||
        type == TransferTypeMessageUnicast)
    {
        return MaxIndexForMessage;
    }
    else if (type == TransferTypeServiceRequest ||
             type == TransferTypeServiceResponse)
    {
        return MaxIndexForService;
    }
    else
    {
        UAVCAN_ASSERT(0);
        return 0;
    }
}

void Frame::setPriority(const TransferPriority priority)
{
    UAVCAN_ASSERT(priority < NumTransferPriorities);
    if (transfer_type_ == TransferTypeMessageBroadcast ||
        transfer_type_ == TransferTypeMessageUnicast)
    {
        if (priority != TransferPriorityService)
        {
            transfer_priority_ = priority;
        }
        else
        {
            UAVCAN_ASSERT(0);
        }
    }
    else
    {
        UAVCAN_ASSERT(0);
    }
}

int Frame::getMaxPayloadLen() const
{
    switch (getTransferType())
    {
    case TransferTypeMessageBroadcast:
    {
        return int(sizeof(payload_));
    }
    case TransferTypeServiceResponse:
    case TransferTypeServiceRequest:
    case TransferTypeMessageUnicast:
    {
        return int(sizeof(payload_)) - 1;
    }
    default:
    {
        UAVCAN_ASSERT(0);
        return -ErrLogic;
    }
    }
}

int Frame::setPayload(const uint8_t* data, unsigned len)
{
    const int maxlen = getMaxPayloadLen();
    if (maxlen < 0)
    {
        return maxlen;
    }
    len = min(unsigned(maxlen), len);
    (void)copy(data, data + len, payload_);
    payload_len_ = uint_fast8_t(len);
    return int(len);
}

template <int OFFSET, int WIDTH>
inline static uint32_t bitunpack(uint32_t val)
{
    StaticAssert<(OFFSET >= 0)>::check();
    StaticAssert<(WIDTH > 0)>::check();
    StaticAssert<((OFFSET + WIDTH) <= 29)>::check();
    return (val >> OFFSET) & ((1UL << WIDTH) - 1);
}

bool Frame::parse(const CanFrame& can_frame)
{
    if (can_frame.isErrorFrame() || can_frame.isRemoteTransmissionRequest() || !can_frame.isExtended())
    {
        return false;
    }

    if (can_frame.dlc > sizeof(can_frame.data))
    {
        UAVCAN_ASSERT(0);  // This is not a protocol error, so UAVCAN_ASSERT() is ok
        return false;
    }

    /*
     * CAN ID parsing
     */
    const uint32_t id = can_frame.id & CanFrame::MaskExtID;
    transfer_id_       = uint8_t(bitunpack<0, 3>(id));
    last_frame_        = bitunpack<3, 1>(id) != 0;
    // TT-specific fields skipped
    transfer_priority_ = TransferPriority(bitunpack<27, 2>(id));

    if (transfer_priority_ == TransferPriorityService)
    {
        frame_index_   = uint_fast8_t(bitunpack<4, 6>(id));
        src_node_id_   = uint8_t(bitunpack<10, 7>(id));
        data_type_id_  = uint16_t(bitunpack<17, 9>(id));
        // RequestNotResponse
        transfer_type_ = (bitunpack<26, 1>(id) == 1U) ? TransferTypeServiceRequest : TransferTypeServiceResponse;
    }
    else
    {
        frame_index_   = uint_fast8_t(bitunpack<4, 4>(id));
        // BroadcastNotUnicast
        transfer_type_ = (bitunpack<8, 1>(id) == 1U) ? TransferTypeMessageBroadcast : TransferTypeMessageUnicast;
        src_node_id_   = uint8_t(bitunpack<9, 7>(id));
        data_type_id_  = uint16_t(bitunpack<16, 11>(id));
    }

    /*
     * CAN payload parsing
     */
    switch (transfer_type_)
    {
    case TransferTypeMessageBroadcast:
    {
        dst_node_id_ = NodeID::Broadcast;
        payload_len_ = can_frame.dlc;
        (void)copy(can_frame.data, can_frame.data + can_frame.dlc, payload_);
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
        payload_len_ = uint8_t(can_frame.dlc - 1);
        (void)copy(can_frame.data + 1, can_frame.data + can_frame.dlc, payload_);
        break;
    }
    default:
    {
        return false;
    }
    }

    /*
     * Special case for anonymous transfers - trailing 8 bits of CAN ID must be ignored
     *  - Transfer ID (assumed zero)
     *  - Last Frame  (assumed true)
     *  - Frame Index (assumed zero)
     */
    if (src_node_id_.isBroadcast())
    {
        transfer_id_ = TransferID(0);
        last_frame_  = true;
        frame_index_ = 0;
    }

    return isValid();
}

template <int OFFSET, int WIDTH>
inline static uint32_t bitpack(uint32_t field)
{
    StaticAssert<(OFFSET >= 0)>::check();
    StaticAssert<(WIDTH > 0)>::check();
    StaticAssert<((OFFSET + WIDTH) <= 29)>::check();
    UAVCAN_ASSERT((field & ((1UL << WIDTH) - 1)) == field);
    return uint32_t((field & ((1UL << WIDTH) - 1)) << OFFSET);
}

bool Frame::compile(CanFrame& out_can_frame) const
{
    if (!isValid())
    {
        UAVCAN_ASSERT(0);        // This is an application error, so we need to maximize it.
        return false;
    }

    /*
     * Setting CAN ID field
     */
    // Common fields for messages and services
    out_can_frame.id =
        CanFrame::FlagEFF |
        bitpack<0, 3>(transfer_id_.get()) |
        bitpack<3, 1>(last_frame_) |
        /* TT-specific fields skipped */
        bitpack<27, 2>(transfer_priority_);

    if (transfer_type_ == TransferTypeServiceRequest ||
        transfer_type_ == TransferTypeServiceResponse)
    {
        out_can_frame.id |=
            bitpack<4, 6>(frame_index_) |
            bitpack<10, 7>(src_node_id_.get()) |
            bitpack<17, 9>(data_type_id_.get()) |
            bitpack<26, 1>((transfer_type_ == TransferTypeServiceRequest) ? 1U : 0U);
    }
    else if (transfer_type_ == TransferTypeMessageBroadcast ||
             transfer_type_ == TransferTypeMessageUnicast)
    {
        out_can_frame.id |=
            bitpack<4, 4>(frame_index_) |
            bitpack<8, 1>((transfer_type_ == TransferTypeMessageBroadcast) ? 1U : 0U) |
            bitpack<9, 7>(src_node_id_.get()) |
            bitpack<16, 11>(data_type_id_.get());
    }
    else
    {
        UAVCAN_ASSERT(0);
        return false;
    }

    /*
     * Setting payload
     */
    switch (transfer_type_)
    {
    case TransferTypeMessageBroadcast:
    {
        out_can_frame.dlc = uint8_t(payload_len_);
        (void)copy(payload_, payload_ + payload_len_, out_can_frame.data);
        break;
    }
    case TransferTypeServiceResponse:
    case TransferTypeServiceRequest:
    case TransferTypeMessageUnicast:
    {
        UAVCAN_ASSERT((payload_len_ + 1U) <= sizeof(out_can_frame.data));
        out_can_frame.data[0] = dst_node_id_.get();
        out_can_frame.dlc = uint8_t(payload_len_ + 1);
        (void)copy(payload_, payload_ + payload_len_, out_can_frame.data + 1);
        break;
    }
    default:
    {
        UAVCAN_ASSERT(0);
        return false;
    }
    }

    /*
     * Setting trailing bits of CAN ID for anonymous message
     * This overrides the following fields:
     *  - Transfer ID (assumed zero)
     *  - Last Frame  (assumed true)
     *  - Frame Index (assumed zero)
     */
    if (src_node_id_.isBroadcast())
    {
        uint8_t sum = 0;
        out_can_frame.id &= ~bitpack<0, 8>(sum);        // Clearing bits
        for (uint_fast8_t i = 0; i < payload_len_; i++)
        {
            sum =  static_cast<uint8_t>(sum + payload_[i]);
        }
        out_can_frame.id |= bitpack<0, 8>(sum);        // Setting the checksum
    }

    return true;
}

bool Frame::isValid() const
{
    /*
     * Frame index
     */
    if (frame_index_ > getMaxIndex())
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    if ((frame_index_ == getMaxIndex()) && !last_frame_)
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    /*
     * Node ID
     */
    if (!src_node_id_.isValid() ||
        !dst_node_id_.isValid())
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    if (src_node_id_.isUnicast() &&
        (src_node_id_ == dst_node_id_))
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    /*
     * Transfer type
     */
    if (transfer_type_ >= NumTransferTypes)
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    if ((transfer_type_ == TransferTypeMessageBroadcast) != dst_node_id_.isBroadcast())
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    // Anonymous transfers
    if (src_node_id_.isBroadcast() &&
        (!last_frame_ || (frame_index_ > 0) || (transfer_type_ != TransferTypeMessageBroadcast)))
    {
        return false;
    }

    /*
     * Payload
     */
    if (static_cast<int>(payload_len_) > getMaxPayloadLen())
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    /*
     * Data type ID
     */
    if (!data_type_id_.isValidForDataTypeKind(getDataTypeKindForTransferType(transfer_type_)))
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    /*
     * Priority
     */
    if (transfer_priority_ >= NumTransferPriorities)
    {
        UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
        return false;
    }

    if (transfer_type_ == TransferTypeServiceRequest ||
        transfer_type_ == TransferTypeServiceResponse)
    {
        if (transfer_priority_ != TransferPriorityService)
        {
            UAVCAN_TRACE("Frame", "Validness check failed at line %d", __LINE__);
            return false;
        }
    }

    return true;
}

bool Frame::operator==(const Frame& rhs) const
{
    return
        (transfer_priority_ == rhs.transfer_priority_) &&
        (transfer_type_     == rhs.transfer_type_) &&
        (data_type_id_      == rhs.data_type_id_) &&
        (src_node_id_       == rhs.src_node_id_) &&
        (dst_node_id_       == rhs.dst_node_id_) &&
        (frame_index_       == rhs.frame_index_) &&
        (transfer_id_       == rhs.transfer_id_) &&
        (last_frame_        == rhs.last_frame_) &&
        (payload_len_       == rhs.payload_len_) &&
        equal(payload_, payload_ + payload_len_, rhs.payload_);
}

#if UAVCAN_TOSTRING
std::string Frame::toString() const
{
    /*
     *  - Priority
     *  - Data Type ID
     *  - Transfer Type
     *  - Source Node ID
     *  - Frame Index
     *  - Last Frame
     *  - Transfer ID
     */
    static const int BUFLEN = 100;
    char buf[BUFLEN];
    int ofs = snprintf(buf, BUFLEN, "prio=%d dtid=%d tt=%d snid=%d dnid=%d idx=%d last=%d tid=%d payload=[",
                       int(transfer_priority_), int(data_type_id_.get()), int(transfer_type_), int(src_node_id_.get()),
                       int(dst_node_id_.get()), int(frame_index_), int(last_frame_), int(transfer_id_.get()));

    for (unsigned i = 0; i < payload_len_; i++)
    {
        ofs += snprintf(buf + ofs, unsigned(BUFLEN - ofs), "%02x", payload_[i]);
        if ((i + 1) < payload_len_)
        {
            ofs += snprintf(buf + ofs, unsigned(BUFLEN - ofs), " ");
        }
    }
    (void)snprintf(buf + ofs, unsigned(BUFLEN - ofs), "]");
    return std::string(buf);
}
#endif

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
        UAVCAN_ASSERT(0);                   // If it is not set, it's a driver failure.
        return false;
    }
    ts_mono_ = can_frame.ts_mono;
    ts_utc_ = can_frame.ts_utc;
    iface_index_ = can_frame.iface_index;
    return true;
}

#if UAVCAN_TOSTRING
std::string RxFrame::toString() const
{
    std::string out = Frame::toString();
    out.reserve(128);
    out += " ts_m="   + ts_mono_.toString();
    out += " ts_utc=" + ts_utc_.toString();
    out += " iface=";
    out += char('0' + iface_index_);
    return out;
}
#endif

}
