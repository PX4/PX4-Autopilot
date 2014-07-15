/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan_stm32/thread.hpp>
#include <uavcan/driver/can.hpp>
#include <uavcan_stm32/bxcan.hpp>

namespace uavcan_stm32
{
/**
 * RX queue item.
 * The application shall not use this directly.
 */
struct CanRxItem
{
    uavcan::uint64_t utc_usec;
    uavcan::CanFrame frame;
    uavcan::CanIOFlags flags;
    CanRxItem()
        : utc_usec(0)
        , flags(0)
    { }
};

/**
 * Single CAN iface.
 * The application shall not use this directly.
 */
class CanIface : public uavcan::ICanIface, uavcan::Noncopyable
{
    class RxQueue
    {
        CanRxItem* const buf_;
        const uavcan::uint8_t capacity_;
        uavcan::uint8_t in_;
        uavcan::uint8_t out_;
        uavcan::uint8_t len_;
        uavcan::uint32_t overflow_cnt_;

        void registerOverflow();

    public:
        RxQueue(CanRxItem* buf, uavcan::uint8_t capacity)
            : buf_(buf)
            , capacity_(capacity)
            , in_(0)
            , out_(0)
            , len_(0)
            , overflow_cnt_(0)
        { }

        void push(const uavcan::CanFrame& frame, const uint64_t& utc_usec, uavcan::CanIOFlags flags);
        void pop(uavcan::CanFrame& out_frame, uavcan::uint64_t& out_utc_usec, uavcan::CanIOFlags& out_flags);

        unsigned getLength() const { return len_; }

        uavcan::uint32_t getOverflowCount() const { return overflow_cnt_; }
    };

    struct Timings
    {
        uavcan::uint16_t prescaler;
        uavcan::uint8_t sjw;
        uavcan::uint8_t bs1;
        uavcan::uint8_t bs2;

        Timings()
            : prescaler(0)
            , sjw(0)
            , bs1(0)
            , bs2(0)
        { }
    };

    struct TxItem
    {
        uavcan::CanFrame frame;
        uavcan::MonotonicTime deadline;
        bool pending;
        bool loopback;
        TxItem()
            : pending(false)
            , loopback(false)
        { }
    };

    enum { NumTxMailboxes = 3 };
    enum { NumFilters = 14 };

    RxQueue rx_queue_;
    bxcan::CanType* const can_;
    uavcan::uint64_t error_cnt_;
    BusEvent& update_event_;
    TxItem pending_tx_[NumTxMailboxes];
    uavcan::uint8_t last_hw_error_code_;
    const uavcan::uint8_t self_index_;
    bool had_activity_;

    int computeTimings(uavcan::uint32_t target_bitrate, Timings& out_timings);

    virtual uavcan::int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                                 uavcan::CanIOFlags flags);

    virtual uavcan::int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                                    uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags);

    virtual uavcan::int16_t configureFilters(const uavcan::CanFilterConfig* filter_configs,
                                             uavcan::uint16_t num_configs);

    virtual uavcan::uint16_t getNumFilters() const { return NumFilters; }

    void pollErrorState();

    void handleTxMailboxInterrupt(uavcan::uint8_t mailbox_index, bool txok, uavcan::uint64_t utc_usec);

    bool waitMsrINakBitStateChange(bool target_state);

public:
    enum { MaxRxQueueCapacity = 254 };

    CanIface(bxcan::CanType* can, BusEvent& update_event, uavcan::uint8_t self_index,
             CanRxItem* rx_queue_buffer, uavcan::uint8_t rx_queue_capacity)
        : rx_queue_(rx_queue_buffer, rx_queue_capacity)
        , can_(can)
        , error_cnt_(0)
        , update_event_(update_event)
        , last_hw_error_code_(0)
        , self_index_(self_index)
        , had_activity_(false)
    {
        UAVCAN_ASSERT(self_index_ < UAVCAN_STM32_NUM_IFACES);
    }

    /**
     * Initializes the hardware CAN controller.
     * Assumes:
     *   - Iface clock is enabled
     *   - Iface has been resetted via RCC
     *   - Caller will configure NVIC by itself
     */
    int init(uavcan::uint32_t bitrate);

    void handleTxInterrupt(uavcan::uint64_t utc_usec);
    void handleRxInterrupt(uavcan::uint8_t fifo_index, uavcan::uint64_t utc_usec);

    void discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time);

    bool isTxBufferFull() const;
    bool isRxBufferEmpty() const;

    /**
     * Total number of hardware failures.
     * May increase continuously if the interface is not connected to the bus.
     */
    virtual uavcan::uint64_t getErrorCount() const;

    /**
     * Returns number of frames pending in the RX queue.
     * This is intended for debug use only.
     */
    unsigned getRxQueueLength() const;

    /**
     * Returns last hardware error code (LEC field in the register ESR).
     * The error code will be reset.
     */
    uavcan::uint8_t yieldLastHardwareErrorCode();

    /**
     * Whether this iface had at least one successful IO since previous call of this method.
     * This is designed for use with iface activity LEDs.
     */
    bool hadActivity();
};

/**
 * CAN driver, incorporates all available CAN ifaces.
 * Please avoid direct use, prefer @ref CanInitHelper instead.
 */
class CanDriver : public uavcan::ICanDriver, uavcan::Noncopyable
{
    BusEvent update_event_;
    CanIface if0_;
#if UAVCAN_STM32_NUM_IFACES > 1
    CanIface if1_;
#endif

    virtual uavcan::int16_t select(uavcan::CanSelectMasks& inout_masks, uavcan::MonotonicTime blocking_deadline);

public:
    template <unsigned RxQueueCapacity>
    CanDriver(CanRxItem (&rx_queue_storage)[UAVCAN_STM32_NUM_IFACES][RxQueueCapacity])
        : update_event_(*this)
        , if0_(bxcan::Can[0], update_event_, 0, rx_queue_storage[0], RxQueueCapacity)
#if UAVCAN_STM32_NUM_IFACES > 1
        , if1_(bxcan::Can[1], update_event_, 1, rx_queue_storage[1], RxQueueCapacity)
#endif
    {
        uavcan::StaticAssert<(RxQueueCapacity <= CanIface::MaxRxQueueCapacity)>::check();
    }

    /**
     * This function returns select masks indicating which interfaces are available for read/write.
     */
    uavcan::CanSelectMasks makeSelectMasks() const;

    /**
     * Returns zero if OK.
     * Returns negative value if failed (e.g. invalid bitrate).
     */
    int init(uavcan::uint32_t bitrate);

    virtual CanIface* getIface(uavcan::uint8_t iface_index);

    virtual uavcan::uint8_t getNumIfaces() const { return UAVCAN_STM32_NUM_IFACES; }

    /**
     * Whether at least one iface had at least one successful IO since previous call of this method.
     * This is designed for use with iface activity LEDs.
     */
    bool hadActivity();
};

/**
 * Helper class.
 * Normally only this class should be used by the application.
 * 145 usec per Extended CAN frame @ 1 Mbps, e.g. 32 RX slots * 145 usec --> 4.6 msec before RX queue overruns.
 */
template <unsigned RxQueueCapacity = 32>
class CanInitHelper
{
    CanRxItem queue_storage_[UAVCAN_STM32_NUM_IFACES][RxQueueCapacity];

public:
    CanDriver driver;

    CanInitHelper()
        : driver(queue_storage_)
    { }

    int init(uavcan::uint32_t bitrate)
    {
        return driver.init(bitrate);
    }
};

}
