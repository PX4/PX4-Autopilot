/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_TRANSPORT_TRANSFER_HPP_INCLUDED
#define UAVCAN_TRANSPORT_TRANSFER_HPP_INCLUDED

#include <cassert>
#include <uavcan/build_config.hpp>
#include <uavcan/std.hpp>

namespace uavcan
{
/**
 * Refer to the UAVCAN specification for more info about transfers.
 */
static const unsigned MaxMessageBroadcastTransferPayloadLen = 126; ///< 16 frames, 8 bytes per frame, 2 byte CRC
static const unsigned MaxMessageUnicastTransferPayloadLen   = 110; ///< 16 frames, 7 bytes per frame, 2 byte CRC
static const unsigned MaxServiceTransferPayloadLen          = 439; ///< 63 frames, 7 bytes per frame, 2 byte CRC

static const unsigned GuaranteedPayloadLenPerFrame = 7;            ///< Guaranteed for all transfers, all CAN standards

static const unsigned MaxPossibleTransferPayloadLen = MaxServiceTransferPayloadLen;

enum TransferType
{
    TransferTypeServiceResponse  = 0,
    TransferTypeServiceRequest   = 1,
    TransferTypeMessageBroadcast = 2,
    TransferTypeMessageUnicast   = 3,
    NumTransferTypes = 4
};


static inline unsigned getMaxPayloadLenForTransferType(const TransferType type)
{
    static const unsigned lens[NumTransferTypes] =
    {
        MaxServiceTransferPayloadLen,
        MaxServiceTransferPayloadLen,
        MaxMessageBroadcastTransferPayloadLen,
        MaxMessageUnicastTransferPayloadLen
    };
    if (static_cast<int>(type) < NumTransferTypes)
    {
        return lens[static_cast<int>(type)];
    }
    else
    {
        UAVCAN_ASSERT(0);
        return 0;
    }
}


enum TransferPriority
{
    TransferPriorityHigh    = 0,
    TransferPriorityNormal  = 1,
    TransferPriorityService = 2,
    TransferPriorityLow     = 3,
    NumTransferPriorities   = 4
};

static inline TransferPriority getDefaultPriorityForTransferType(const TransferType type)
{
    if (type == TransferTypeServiceResponse || type == TransferTypeServiceRequest)
    {
        return TransferPriorityService;
    }
    else if (type == TransferTypeMessageBroadcast || type == TransferTypeMessageUnicast)
    {
        return TransferPriorityNormal;
    }
    else
    {
        UAVCAN_ASSERT(0);
        return TransferPriority(0); // whatever
    }
}


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
    static const uint8_t MaxRecommendedForRegularNodes = Max - 2;
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

#endif // UAVCAN_TRANSPORT_TRANSFER_HPP_INCLUDED
