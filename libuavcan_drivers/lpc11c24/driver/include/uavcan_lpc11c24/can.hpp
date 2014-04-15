/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/driver/can.hpp>

namespace uavcan_lpc11c24
{

class CanDriver
    : public uavcan::ICanDriver
    , public uavcan::ICanIface
    , uavcan::Noncopyable
{
    static CanDriver self;

    CanDriver() { }

public:
    static CanDriver& instance() { return self; }

    int init(uavcan::uint32_t baudrate);

    bool hasPendingRx() const;
    bool hasEmptyTx() const;

    bool hadActivity();

    virtual uavcan::int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                                 uavcan::CanIOFlags flags);

    virtual uavcan::int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                            uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags);

    virtual uavcan::int16_t select(uavcan::CanSelectMasks& inout_masks, uavcan::MonotonicTime blocking_deadline);

    virtual uavcan::int16_t configureFilters(const uavcan::CanFilterConfig* filter_configs,
                                             uavcan::uint16_t num_configs);

    virtual uavcan::uint64_t getErrorCount() const;

    virtual uavcan::uint16_t getNumFilters() const;

    virtual uavcan::ICanIface* getIface(uavcan::uint8_t iface_index);

    virtual uavcan::uint8_t getNumIfaces() const;
};

}
