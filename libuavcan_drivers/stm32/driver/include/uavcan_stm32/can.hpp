/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan_stm32/thread.hpp>
#include <uavcan/driver/can.hpp>

#if UAVCAN_STM32_CHIBIOS
# include <hal.h>
#else
# error "Unknown OS"
#endif

#ifndef UAVCAN_STM32_NUM_IFACES
# if defined(STM32F10X_CL) || defined(STM32F2XX) || defined(STM32F4XX)
#  define UAVCAN_STM32_NUM_IFACES   2
# else
#  define UAVCAN_STM32_NUM_IFACES   1
# endif
#endif

#if UAVCAN_STM32_NUM_IFACES != 1 && UAVCAN_STM32_NUM_IFACES != 2
# error UAVCAN_STM32_NUM_IFACES
#endif

namespace uavcan_stm32
{
/**
 * RX queue item
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
    CAN_TypeDef* const can_;
    uavcan::uint64_t error_cnt_;
    Event& update_event_;
    TxItem pending_tx_[NumTxMailboxes];
    uavcan::uint8_t last_hw_error_code_;
    const uavcan::uint8_t self_index_;

    int computeTimings(uavcan::uint32_t target_bitrate, Timings& out_timings);

    virtual uavcan::int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                                 uavcan::CanIOFlags flags);

    virtual uavcan::int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                                    uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags);

    virtual uavcan::int16_t configureFilters(const uavcan::CanFilterConfig* filter_configs,
                                             uavcan::uint16_t num_configs);

    virtual uavcan::uint16_t getNumFilters() const { return NumFilters; }

    void handleTxMailboxInterrupt(uavcan::uint8_t mailbox_index, bool txok, uavcan::uint64_t utc_usec);

public:
    enum { MaxRxQueueCapacity = 254 };

    CanIface(CAN_TypeDef* can, Event& update_event, uavcan::uint8_t self_index,
             CanRxItem* rx_queue_buffer, uavcan::uint8_t rx_queue_capacity)
        : rx_queue_(rx_queue_buffer, rx_queue_capacity)
        , can_(can)
        , error_cnt_(0)
        , update_event_(update_event)
        , last_hw_error_code_(0)
        , self_index_(self_index)
    {
        assert(self_index_ < UAVCAN_STM32_NUM_IFACES);
    }

    /**
     * Assumes:
     *   - Iface clock is enabled
     *   - Iface has been resetted via RCC
     *   - Interrupts are disabled
     *   - Caller will configure NVIC by itself
     */
    int init(uavcan::uint32_t bitrate);

    void handleTxInterrupt(uavcan::uint64_t utc_usec);
    void handleRxInterrupt(uavcan::uint8_t fifo_index, uavcan::uint64_t utc_usec);
    void handleStatusInterrupt();

    void discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time);

    bool isTxBufferFull() const;
    bool isRxBufferEmpty() const;

    virtual uavcan::uint64_t getErrorCount() const;
    uavcan::uint8_t yieldLastHardwareErrorCode();
};

/**
 * CAN driver, incorporates all available CAN ifaces.
 */
class CanDriver : public uavcan::ICanDriver, uavcan::Noncopyable
{
    Event update_event_;
    CanIface if0_;
#if UAVCAN_STM32_NUM_IFACES > 1
    CanIface if1_;
#endif

    uavcan::CanSelectMasks makeSelectMasks() const;

    virtual uavcan::int16_t select(uavcan::CanSelectMasks& inout_masks, uavcan::MonotonicTime blocking_deadline);

public:
    template <unsigned RxQueueCapacity>
    CanDriver(CanRxItem (&rx_queue_storage)[UAVCAN_STM32_NUM_IFACES][RxQueueCapacity])
        : if0_(CAN1, update_event_, 0, rx_queue_storage[0], RxQueueCapacity)
#if UAVCAN_STM32_NUM_IFACES > 1
        , if1_(CAN2, update_event_, 1, rx_queue_storage[1], RxQueueCapacity)
#endif
    {
        uavcan::StaticAssert<(RxQueueCapacity <= CanIface::MaxRxQueueCapacity)>::check();
    }

    int init(uavcan::uint32_t bitrate);

    virtual CanIface* getIface(uavcan::uint8_t iface_index);

    virtual uavcan::uint8_t getNumIfaces() const { return UAVCAN_STM32_NUM_IFACES; }
};

/**
 * Helper class.
 * Normally only this class should be used by the application.
 * 145 usec per Extended CAN frame @ 1 Mbps, e.g. 16 RX slots * 145 usec --> 2320 usec before RX queue overruns.
 */
template <unsigned RxQueueCapacity = 16>
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
