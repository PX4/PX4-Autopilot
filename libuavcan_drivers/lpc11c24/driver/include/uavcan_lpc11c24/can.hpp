/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/driver/can.hpp>

namespace uavcan_lpc11c24
{
/**
 * This class implements CAN driver interface for libuavcan.
 * No configuration needed other than CAN baudrate.
 * This class is a singleton.
 */
class CanDriver
    : public uavcan::ICanDriver
    , public uavcan::ICanIface
    , uavcan::Noncopyable
{
    static CanDriver self;

    CanDriver() { }

public:
    /**
     * Returns the singleton reference.
     * No other copies can be created.
     */
    static CanDriver& instance() { return self; }

    /**
     * Returns negative value if the requested baudrate can't be used.
     * Returns zero if OK.
     */
    int init(uavcan::uint32_t baudrate);

    bool hasReadyRx() const;
    bool hasEmptyTx() const;

    /**
     * This method will return true only if there was any CAN bus activity since previous call of this method.
     * This is intended to be used for LED iface activity indicators.
     */
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
