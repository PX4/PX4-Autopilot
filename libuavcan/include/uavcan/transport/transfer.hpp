/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/build_config.hpp>
#include <uavcan/stdint.hpp>

namespace uavcan
{

static const unsigned MaxTransferPayloadLen = 439; ///< According to the specification.

static const unsigned MaxSingleFrameTransferPayloadLen = 7;

enum TransferType
{
    TransferTypeServiceResponse  = 0,
    TransferTypeServiceRequest   = 1,
    TransferTypeMessageBroadcast = 2,
    TransferTypeMessageUnicast   = 3,
    NumTransferTypes = 4
};


class UAVCAN_EXPORT TransferID
{
    uint8_t value_;

public:
    static const uint8_t BitLen = 3U;
    static const uint8_t Max = (1U << BitLen) - 1U;

    TransferID()
        : value_(0)
    { }

    TransferID(uint8_t value)    // implicit
        : value_(value)
    {
        value_ &= Max;
        UAVCAN_ASSERT(value == value_);
    }

    bool operator!=(TransferID rhs) const { return !operator==(rhs); }
    bool operator==(TransferID rhs) const { return get() == rhs.get(); }

    void increment()
    {
        value_ = (value_ + 1) & Max;
    }

    uint8_t get() const
    {
        UAVCAN_ASSERT(value_ <= Max);
        return value_;
    }

    /**
     * Amount of increment() calls to reach rhs value.
     */
    int computeForwardDistance(TransferID rhs) const;
};


class UAVCAN_EXPORT NodeID
{
    static const uint8_t ValueBroadcast = 0;
    static const uint8_t ValueInvalid   = 0xFF;
    uint8_t value_;

public:
    static const uint8_t BitLen = 7U;
    static const uint8_t Max = (1U << BitLen) - 1U;
    static const NodeID Broadcast;

    NodeID() : value_(ValueInvalid) { }

    NodeID(uint8_t value)   // Implicit
        : value_(value)
    {
        UAVCAN_ASSERT(isValid());
    }

    uint8_t get() const { return value_; }

    bool isValid()     const { return value_ <= Max; }
    bool isBroadcast() const { return value_ == ValueBroadcast; }
    bool isUnicast()   const { return (value_ <= Max) && (value_ != ValueBroadcast); }

    bool operator!=(NodeID rhs) const { return !operator==(rhs); }
    bool operator==(NodeID rhs) const { return value_ == rhs.value_; }

    bool operator<(NodeID rhs) const { return value_ < rhs.value_; }
    bool operator>(NodeID rhs) const { return value_ > rhs.value_; }
    bool operator<=(NodeID rhs) const { return value_ <= rhs.value_; }
    bool operator>=(NodeID rhs) const { return value_ >= rhs.value_; }
};

}
