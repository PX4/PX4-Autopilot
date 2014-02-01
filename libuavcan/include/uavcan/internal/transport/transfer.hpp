/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <algorithm>
#include <uavcan/internal/transport/can_io.hpp>

namespace uavcan
{

enum TransferType
{
    SERVICE_RESPONSE  = 0,
    SERVICE_REQUEST   = 1,
    MESSAGE_BROADCAST = 2,
    MESSAGE_UNICAST   = 3
};


class TransferID
{
    uint_fast8_t value_;

public:
    enum { BITLEN = 4 };
    enum { MAX = (1 << BITLEN) - 1 };

    TransferID()
    : value_(0)
    { }

    TransferID(uint_fast8_t value)    // implicit
    : value_(value)
    {
        value_ &= MAX;
        assert(value == value_);
    }

    bool operator!=(TransferID rhs) const { return !operator==(rhs); }
    bool operator==(TransferID rhs) const { return get() == rhs.get(); }

    void increment()
    {
        value_ = (value_ + 1) & MAX;
    }

    uint_fast8_t get() const
    {
        assert(value_ <= MAX);
        return value_;
    }

    /**
     * Amount of increment() calls to reach rhs value.
     */
    int forwardDistance(TransferID rhs) const;
};


struct Frame
{
    uint8_t payload[8];
    TransferType transfer_type;
    uint_fast16_t data_type_id;
    uint_fast8_t payload_len;
    uint_fast8_t source_node_id;
    uint_fast8_t frame_index;
    TransferID transfer_id;
    bool last_frame;

    Frame(const uint8_t* payload, uint_fast8_t payload_len, uint_fast16_t data_type_id, TransferType transfer_type,
          uint_fast8_t source_node_id, uint_fast8_t frame_index, TransferID transfer_id, bool last_frame)
    : transfer_type(transfer_type)
    , data_type_id(data_type_id)
    , payload_len(payload_len)
    , source_node_id(source_node_id)
    , frame_index(frame_index)
    , transfer_id(transfer_id)
    , last_frame(last_frame)
    {
        assert(payload && payload_len <= 8);
        std::copy(payload, payload + payload_len, this->payload);
    }

    static Frame parse(const CanFrame& can_frame);

    CanFrame compile() const;

    bool operator!=(const Frame& rhs) const { return !operator==(rhs); }
    bool operator==(const Frame& rhs) const
    {
        return
            (transfer_type == rhs.transfer_type) &&
            (data_type_id == rhs.data_type_id) &&
            (source_node_id == rhs.source_node_id) &&
            (frame_index == rhs.frame_index) &&
            (transfer_id == rhs.transfer_id) &&
            (last_frame == rhs.last_frame) &&
            (payload_len == rhs.payload_len) &&
            std::equal(payload, payload + payload_len, rhs.payload);
    }
};


struct RxFrame
{
    uint_fast64_t timestamp;
    Frame frame;
    uint_fast8_t iface_index;

    RxFrame(const Frame& frame, uint_fast64_t timestamp, uint_fast8_t iface_index)
    : timestamp(timestamp)
    , frame(frame)
    , iface_index(iface_index)
    { }

    static RxFrame parse(const CanRxFrame& can_frame)
    {
        return RxFrame(Frame::parse(can_frame.frame), can_frame.timestamp, can_frame.iface_index);
    }
};

}
