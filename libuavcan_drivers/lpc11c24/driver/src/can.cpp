/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan_lpc11c24/can.hpp>
#include <uavcan_lpc11c24/clock.hpp>
#include <uavcan/util/templates.hpp>
#include <chip.h>
#include "internal.hpp"

/**
 * The default value should be OK for any use case.
 */
#ifndef UAVCAN_LPC11C24_RX_QUEUE_LEN
# define UAVCAN_LPC11C24_RX_QUEUE_LEN   8
#endif

#if UAVCAN_LPC11C24_RX_QUEUE_LEN > 254
# error UAVCAN_LPC11C24_RX_QUEUE_LEN is too large
#endif

extern "C" void canRxCallback(uint8_t msg_obj_num);
extern "C" void canTxCallback(uint8_t msg_obj_num);
extern "C" void canErrorCallback(uint32_t error_info);

namespace uavcan_lpc11c24
{
namespace
{
/**
 * Hardware message objects are allocated as follows:
 *  - 1 - Single TX object
 *  - 2..32 - RX objects
 * TX priority is defined by the message object number, not by the CAN ID (chapter 16.7.3.5 of the user manual),
 * hence we can't use more than one object because that would cause priority inversion on long transfers.
 */
const unsigned NumMsgObjects = 32;

/**
 * Total number of CAN errors.
 * Does not overflow.
 */
uint32_t error_cnt;

/**
 * True if there's no pending TX frame, i.e. write is possible.
 */
bool tx_free = true;

/**
 * Gets updated every time the CAN IRQ handler is being called.
 */
uint64_t last_irq_utc_timestamp = 0;

bool had_activity;

/**
 * After a received message gets extracted from C_CAN, it will be stored in the RX queue until libuavcan
 * reads it via select()/receive() calls.
 */
class RxQueue
{
    struct Item
    {
        uint64_t utc_usec;
        uavcan::CanFrame frame;
        Item() : utc_usec(0) { }
    };

    Item buf_[UAVCAN_LPC11C24_RX_QUEUE_LEN];
    uint32_t overflow_cnt_;
    uint8_t in_;
    uint8_t out_;
    uint8_t len_;

public:
    RxQueue()
        : overflow_cnt_(0)
        , in_(0)
        , out_(0)
        , len_(0)
    { }

    void push(const uavcan::CanFrame& frame, const uint64_t& utc_usec)
    {
        buf_[in_].frame    = frame;
        buf_[in_].utc_usec = utc_usec;
        in_++;
        if (in_ >= UAVCAN_LPC11C24_RX_QUEUE_LEN)
        {
            in_ = 0;
        }
        len_++;
        if (len_ > UAVCAN_LPC11C24_RX_QUEUE_LEN)
        {
            len_ = UAVCAN_LPC11C24_RX_QUEUE_LEN;
            if (overflow_cnt_ < 0xFFFFFFFF)
            {
                overflow_cnt_++;
            }
            out_++;
            if (out_ >= UAVCAN_LPC11C24_RX_QUEUE_LEN)
            {
                out_ = 0;
            }
        }
    }

    void pop(uavcan::CanFrame& out_frame, uint64_t& out_utc_usec)
    {
        if (len_ > 0)
        {
            out_frame    = buf_[out_].frame;
            out_utc_usec = buf_[out_].utc_usec;
            out_++;
            if (out_ >= UAVCAN_LPC11C24_RX_QUEUE_LEN)
            {
                out_ = 0;
            }
            len_--;
        }
    }

    unsigned getLength() const { return len_; }

    uint32_t getOverflowCount() const { return overflow_cnt_; }
};

RxQueue rx_queue;


int computeBaudrate(uint32_t baud_rate, uint32_t can_api_timing_cfg[2])
{
    const uint32_t pclk = Chip_Clock_GetMainClockRate();
    const uint32_t clk_per_bit = pclk / baud_rate;
    for (uint32_t div = 0; div <= 15; div++)
    {
        for (uint32_t quanta = 1; quanta <= 32; quanta++)
        {
            for (uint32_t segs = 3; segs <= 17; segs++)
            {
                if (clk_per_bit == (segs * quanta * (div + 1)))
                {
                    segs -= 3;
                    const uint32_t seg1 = segs / 2;
                    const uint32_t seg2 = segs - seg1;
                    const uint32_t can_sjw = (seg1 > 3) ? 3 : seg1;
                    can_api_timing_cfg[0] = div;
                    can_api_timing_cfg[1] = ((quanta - 1) & 0x3F) |
                                            (can_sjw & 0x03) << 6 |
                                            (seg1 & 0x0F) << 8 |
                                            (seg2 & 0x07) << 12;
                    return 0;
                }
            }
        }
    }
    return -1;
}

} // namespace

CanDriver CanDriver::self;

int CanDriver::init(uavcan::uint32_t baudrate)
{
    CriticalSectionLocker locker;

    error_cnt = 0;
    tx_free = true;
    last_irq_utc_timestamp = 0;
    had_activity = false;

    /*
     * C_CAN init
     */
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_CAN);
    static uint32_t can_api_init_table[2];
    if (computeBaudrate(baudrate, can_api_init_table) != 0)
    {
        return -1;
    }
    LPC_CCAN_API->init_can(can_api_init_table, true);
    static CCAN_CALLBACKS_T ccan_callbacks =
    {
        canRxCallback,
        canTxCallback,
        canErrorCallback,
        0,
        0,
        0,
        0,
        0
    };
    LPC_CCAN_API->config_calb(&ccan_callbacks);
    NVIC_EnableIRQ(CAN_IRQn);

    /*
     * Default RX msgobj config:
     *  31 - all STD
     *  32 - all EXT
     * RTR ignored
     */
    CCAN_MSG_OBJ_T msg_obj = CCAN_MSG_OBJ_T();
    msg_obj.msgobj = 31;
    LPC_CCAN_API->config_rxmsgobj(&msg_obj);
    msg_obj.mode_id = CAN_MSGOBJ_EXT;
    msg_obj.msgobj = 32;
    LPC_CCAN_API->config_rxmsgobj(&msg_obj);

    return 0;
}

bool CanDriver::hasReadyRx() const
{
    CriticalSectionLocker locker;
    return rx_queue.getLength() > 0;
}

bool CanDriver::hasEmptyTx() const
{
    CriticalSectionLocker locker;
    return tx_free;
}

bool CanDriver::hadActivity()
{
    CriticalSectionLocker locker;
    const bool ret = had_activity;
    had_activity = false;
    return ret;
}

uavcan::int16_t CanDriver::send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                                uavcan::CanIOFlags flags)
{
    if (frame.isErrorFrame() ||
        frame.dlc > 8 ||
        flags != 0)      // Only plain IO is allowed. Loopback, TX timestamping are not supported by this driver.
    {
        return -1;
    }

    /*
     * Frame conversion
     */
    CCAN_MSG_OBJ_T msgobj = CCAN_MSG_OBJ_T();
    msgobj.mode_id = frame.id & uavcan::CanFrame::MaskExtID;
    if (frame.isExtended())
    {
        msgobj.mode_id |= CAN_MSGOBJ_EXT;
    }
    if (frame.isRemoteTransmissionRequest())
    {
        msgobj.mode_id |= CAN_MSGOBJ_RTR;
    }
    msgobj.dlc = frame.dlc;
    uavcan::copy(frame.data, frame.data + frame.dlc, msgobj.data);

    /*
     * Transmission
     */
    (void)tx_deadline;               // TX timeouts are not supported by this driver yet (and hardly going to be).

    CriticalSectionLocker locker;

    if (tx_free)
    {
        tx_free = false;   // Mark as pending - will be released in TX callback
        msgobj.msgobj = 1;
        LPC_CCAN_API->can_transmit(&msgobj);
        return 1;
    }
    return 0;
}

uavcan::int16_t CanDriver::receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                                   uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
{
    out_ts_monotonic = uavcan_lpc11c24::clock::getMonotonic();
    out_flags = 0;                                            // We don't support any IO flags

    CriticalSectionLocker locker;
    if (rx_queue.getLength() == 0)
    {
        return 0;
    }
    uint64_t ts_utc = 0;
    rx_queue.pop(out_frame, ts_utc);
    out_ts_utc = uavcan::UtcTime::fromUSec(ts_utc);
    return 1;
}

uavcan::int16_t CanDriver::select(uavcan::CanSelectMasks& inout_masks, uavcan::MonotonicTime blocking_deadline)
{
    const bool noblock = ((inout_masks.read  == 1) && hasReadyRx()) ||
                         ((inout_masks.write == 1) && hasEmptyTx());

    if (!noblock && (clock::getMonotonic() > blocking_deadline))
    {
#if UAVCAN_LPC11C24_USE_WFE
        /*
         * It's not cool (literally) to burn cycles in a busyloop, and we have no OS to pass control to other
         * tasks, thus solution is to halt the core until a hardware event occurs - e.g. clock timer overflow.
         * Upon such event the select() call will return, even if no requested IO operations became available.
         * It's OK to do that, libuavcan can handle such behavior.
         *
         * Note that it is not possible to precisely control the sleep duration with WFE, since we can't predict when
         * the next hardware event occurs. Worst case conditions:
         *  - WFE gets executed right after the clock timer interrupt;
         *  - CAN bus is completely silent (no traffic);
         *  - User's application has no interrupts and generates no hardware events.
         * In such scenario execution will stuck here for one period of the clock timer interrupt, even if
         * blocking_deadline expires sooner.
         * If the user's application requires higher timing precision, an extra dummy IRQ can be added just to
         * break WFE every once in a while.
         */
        __WFE();
#endif
    }

    inout_masks.read  = hasReadyRx() ? 1 : 0;
    inout_masks.write = hasEmptyTx() ? 1 : 0;
    return 0;  // Return value doesn't matter as long as it is non-negative
}

uavcan::int16_t CanDriver::configureFilters(const uavcan::CanFilterConfig* filter_configs,
                                            uavcan::uint16_t num_configs)
{
    (void)filter_configs;
    (void)num_configs;
    return -1;
}

uavcan::uint64_t CanDriver::getErrorCount() const
{
    CriticalSectionLocker locker;
    return uint64_t(error_cnt) + uint64_t(rx_queue.getOverflowCount());
}

uavcan::uint16_t CanDriver::getNumFilters() const
{
    return NumMsgObjects - 1;  // First msgobj is reserved for TX frame
}

uavcan::ICanIface* CanDriver::getIface(uavcan::uint8_t iface_index)
{
    return (iface_index == 0) ? this : NULL;
}

uavcan::uint8_t CanDriver::getNumIfaces() const
{
    return 1;
}

}

/*
 * C_CAN handlers
 */
extern "C"
{

void canRxCallback(uint8_t msg_obj_num)
{
    CCAN_MSG_OBJ_T msg_obj = CCAN_MSG_OBJ_T();
    msg_obj.msgobj = msg_obj_num;
    LPC_CCAN_API->can_receive(&msg_obj);

    uavcan::CanFrame frame;

    // CAN ID, EXT or not
    if (msg_obj.mode_id & CAN_MSGOBJ_EXT)
    {
        frame.id = msg_obj.mode_id & uavcan::CanFrame::MaskExtID;
        frame.id |= uavcan::CanFrame::FlagEFF;
    }
    else
    {
        frame.id = msg_obj.mode_id & uavcan::CanFrame::MaskStdID;
    }

    // RTR
    if (msg_obj.mode_id & CAN_MSGOBJ_RTR)
    {
        frame.id |= uavcan::CanFrame::FlagRTR;
    }

    // Payload
    frame.dlc = msg_obj.dlc;
    uavcan::copy(msg_obj.data, msg_obj.data + msg_obj.dlc, frame.data);

    uavcan_lpc11c24::rx_queue.push(frame, uavcan_lpc11c24::last_irq_utc_timestamp);
    uavcan_lpc11c24::had_activity = true;
}

void canTxCallback(uint8_t msg_obj_num)
{
    (void)msg_obj_num;
    uavcan_lpc11c24::tx_free = true;
    uavcan_lpc11c24::had_activity = true;
}

void canErrorCallback(uint32_t error_info)
{
    (void)error_info;
    if (uavcan_lpc11c24::error_cnt < 0xFFFFFFFF)
    {
        uavcan_lpc11c24::error_cnt++;
    }
}

void CAN_IRQHandler();

void CAN_IRQHandler()
{
    uavcan_lpc11c24::last_irq_utc_timestamp = uavcan_lpc11c24::clock::getUtcUSecFromCanInterrupt();
    LPC_CCAN_API->isr();
}

}
