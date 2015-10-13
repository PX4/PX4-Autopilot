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
     * Attempts to detect bit rate of the CAN bus.
     * This function may block for up to X seconds, where X is the number of bit rates to try.
     * This function is NOT guaranteed to reset the CAN controller upon return.
     * @return On success: detected bit rate, in bits per second.
     *         On failure: zero.
     */
    static uavcan::uint32_t detectBitRate(void (*idle_callback)() = nullptr);

    /**
     * Returns negative value if the requested baudrate can't be used.
     * Returns zero if OK.
     */
    int init(uavcan::uint32_t bitrate);

    bool hasReadyRx() const;
    bool hasEmptyTx() const;

    /**
     * This method will return true only if there was any CAN bus activity since previous call of this method.
     * This is intended to be used for LED iface activity indicators.
     */
    bool hadActivity();

    /**
     * Returns the number of times the RX queue was overrun.
     */
    uavcan::uint32_t getRxQueueOverflowCount() const;

    /**
     * Whether the controller is currently in bus off state.
     * Note that the driver recovers the CAN controller from the bus off state automatically!
     * Therefore, this method serves only monitoring purposes and is not necessary to use.
     */
    bool isInBusOffState() const;

    uavcan::int16_t send(const uavcan::CanFrame& frame,
                         uavcan::MonotonicTime tx_deadline,
                         uavcan::CanIOFlags flags) override;

    uavcan::int16_t receive(uavcan::CanFrame& out_frame,
                            uavcan::MonotonicTime& out_ts_monotonic,
                            uavcan::UtcTime& out_ts_utc,
                            uavcan::CanIOFlags& out_flags) override;

    uavcan::int16_t select(uavcan::CanSelectMasks& inout_masks,
                           const uavcan::CanFrame* (&)[uavcan::MaxCanIfaces],
                           uavcan::MonotonicTime blocking_deadline) override;

    uavcan::int16_t configureFilters(const uavcan::CanFilterConfig* filter_configs,
                                     uavcan::uint16_t num_configs) override;

    uavcan::uint64_t getErrorCount() const override;

    uavcan::uint16_t getNumFilters() const override;

    uavcan::ICanIface* getIface(uavcan::uint8_t iface_index) override;

    uavcan::uint8_t getNumIfaces() const override;
};

}
