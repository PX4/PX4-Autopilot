/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/transport/transfer.hpp>
#include <uavcan/transport/can_io.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/data_type.hpp>

namespace uavcan
{

class UAVCAN_EXPORT Frame
{
    uint8_t payload_[sizeof(CanFrame::data)];
    TransferType transfer_type_;
    DataTypeID data_type_id_;
    uint_fast8_t payload_len_;
    NodeID src_node_id_;
    NodeID dst_node_id_;
    uint_fast8_t frame_index_;
    TransferID transfer_id_;
    bool last_frame_;

public:
    enum { MaxIndex = 62 };        // 63 (or 0b111111) is reserved

    Frame()
        : transfer_type_(TransferType(NumTransferTypes)) // That is invalid value
        , payload_len_(0)
        , frame_index_(0)
        , transfer_id_(0)
        , last_frame_(false)
    { }

    Frame(DataTypeID data_type_id, TransferType transfer_type, NodeID src_node_id, NodeID dst_node_id,
          uint_fast8_t frame_index, TransferID transfer_id, bool last_frame = false)
        : transfer_type_(transfer_type)
        , data_type_id_(data_type_id)
        , payload_len_(0)
        , src_node_id_(src_node_id)
        , dst_node_id_(dst_node_id)
        , frame_index_(frame_index)
        , transfer_id_(transfer_id)
        , last_frame_(last_frame)
    {
        UAVCAN_ASSERT((transfer_type == TransferTypeMessageBroadcast) == dst_node_id.isBroadcast());
        UAVCAN_ASSERT(data_type_id.isValid());
        UAVCAN_ASSERT(src_node_id != dst_node_id);
        UAVCAN_ASSERT(frame_index <= MaxIndex);
    }

    /**
     * Max payload length depends on the transfer type and frame index.
     */
    int getMaxPayloadLen() const;
    int setPayload(const uint8_t* data, unsigned len);

    unsigned getPayloadLen() const { return payload_len_; }
    const uint8_t* getPayloadPtr() const { return payload_; }

    TransferType getTransferType() const { return transfer_type_; }
    DataTypeID getDataTypeID()     const { return data_type_id_; }
    NodeID getSrcNodeID()          const { return src_node_id_; }
    NodeID getDstNodeID()          const { return dst_node_id_; }
    TransferID getTransferID()     const { return transfer_id_; }
    uint_fast8_t getIndex()        const { return frame_index_; }
    bool isLast()                  const { return last_frame_; }

    void makeLast() { last_frame_ = true; }
    void setIndex(int index) { frame_index_ = uint_fast8_t(index); }

    bool isFirst() const { return frame_index_ == 0; }

    bool parse(const CanFrame& can_frame);
    bool compile(CanFrame& can_frame) const;

    bool isValid() const;

    bool operator!=(const Frame& rhs) const { return !operator==(rhs); }
    bool operator==(const Frame& rhs) const;

#if UAVCAN_TOSTRING
    std::string toString() const;
#endif
};


class UAVCAN_EXPORT RxFrame : public Frame
{
    MonotonicTime ts_mono_;
    UtcTime ts_utc_;
    uint8_t iface_index_;

public:
    RxFrame()
        : iface_index_(0)
    { }

    RxFrame(const Frame& frame, MonotonicTime ts_mono, UtcTime ts_utc, uint8_t iface_index)
        : ts_mono_(ts_mono)
        , ts_utc_(ts_utc)
        , iface_index_(iface_index)
    {
        *static_cast<Frame*>(this) = frame;
    }

    bool parse(const CanRxFrame& can_frame);

    /**
     * Can't be zero.
     */
    MonotonicTime getMonotonicTimestamp() const { return ts_mono_; }

    /**
     * Can be zero if not supported by the platform driver.
     */
    UtcTime getUtcTimestamp() const { return ts_utc_; }

    uint8_t getIfaceIndex() const { return iface_index_; }

#if UAVCAN_TOSTRING
    std::string toString() const;
#endif
};

}
