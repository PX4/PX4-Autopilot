/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <cassert>
#include <uavcan/impl_constants.hpp>
#include <uavcan/stdint.hpp>

namespace uavcan
{

enum { MaxTransferPayloadLen = 439 }; ///< According to the standard

enum { MaxSingleFrameTransferPayloadLen = 7 };

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
    enum { BitLen = 3 };
    enum { Max = (1 << BitLen) - 1 };

    TransferID()
        : value_(0)
    { }

    TransferID(uint8_t value)    // implicit
        : value_(value)
    {
        value_ &= Max;
        assert(value == value_);
    }

    bool operator!=(TransferID rhs) const { return !operator==(rhs); }
    bool operator==(TransferID rhs) const { return get() == rhs.get(); }

    void increment()
    {
        value_ = (value_ + 1) & Max;
    }

    uint8_t get() const
    {
        assert(value_ <= Max);
        return value_;
    }

    /**
     * Amount of increment() calls to reach rhs value.
     */
    int computeForwardDistance(TransferID rhs) const;
};


class UAVCAN_EXPORT NodeID
{
    enum
    {
        ValueBroadcast = 0,
        ValueInvalid = 0xFF
    };
    uint8_t value_;

public:
    enum { BitLen = 7 };
    enum { Max = (1 << BitLen) - 1 };

    static const NodeID Broadcast;

    NodeID() : value_(ValueInvalid) { }

    NodeID(uint8_t value)   // Implicit
        : value_(value)
    {
        assert(isValid());
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
