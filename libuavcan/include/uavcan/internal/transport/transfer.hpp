/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <algorithm>
#include <uavcan/internal/transport/can_io.hpp>

namespace uavcan
{

enum NodeIDConstants
{
    NODE_ID_BROADCAST = 0,
    NODE_ID_MAX       = 127,
    NODE_ID_INVALID   = 255
};

enum TransferType
{
    TRANSFER_TYPE_SERVICE_RESPONSE  = 0,
    TRANSFER_TYPE_SERVICE_REQUEST   = 1,
    TRANSFER_TYPE_MESSAGE_BROADCAST = 2,
    TRANSFER_TYPE_MESSAGE_UNICAST   = 3,
    NUM_TRANSFER_TYPES = 4
};

enum DataTypeKind
{
    DATA_TYPE_KIND_SERVICE,
    DATA_TYPE_KIND_MESSAGE,
    NUM_DATA_TYPE_KINDS
};

class TransferID
{
    uint8_t value_;

public:
    enum { BITLEN = 4 };
    enum { MAX = (1 << BITLEN) - 1 };

    TransferID()
    : value_(0)
    { }

    TransferID(uint8_t value)    // implicit
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

    uint8_t get() const
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
    enum { DATA_TYPE_ID_MAX = 1023 };
    enum { NODE_ID_MAX = 127 };
    enum { FRAME_INDEX_MAX = 31 };

    uint8_t payload[8];
    TransferType transfer_type;
    uint_fast16_t data_type_id;
    uint_fast8_t payload_len;
    uint_fast8_t source_node_id;
    uint_fast8_t frame_index;
    TransferID transfer_id;
    bool last_frame;

    Frame()
    : transfer_type(TransferType(0))
    , data_type_id(0)
    , payload_len(0)
    , source_node_id(0)
    , frame_index(0)
    , transfer_id(0)
    , last_frame(false)
    {
        std::fill(payload, payload + sizeof(payload), 0);
    }

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
        assert(data_type_id <= DATA_TYPE_ID_MAX);
        assert(source_node_id <= NODE_ID_MAX);
        assert(frame_index <= FRAME_INDEX_MAX);
        assert(payload && payload_len <= sizeof(payload));
        std::copy(payload, payload + payload_len, this->payload);
    }

    bool parse(const CanFrame& can_frame);

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


struct RxFrame : public Frame
{
    uint_fast64_t timestamp;
    uint_fast8_t iface_index;

    RxFrame()
    : timestamp(0)
    , iface_index(0)
    { }

    bool parse(const CanRxFrame& can_frame)
    {
        if (!Frame::parse(can_frame))
            return false;
        timestamp = can_frame.timestamp;
        iface_index = can_frame.iface_index;
        return true;
    }
};

}
