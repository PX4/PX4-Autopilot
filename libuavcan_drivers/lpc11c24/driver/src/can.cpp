/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan_lpc11c24/can.hpp>
#include <uavcan_lpc11c24/clock.hpp>
#include <uavcan/util/templates.hpp>
#include <chip.h>
#include "c_can.hpp"
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

extern "C" void canRxCallback(std::uint8_t msg_obj_num);
extern "C" void canTxCallback(std::uint8_t msg_obj_num);
extern "C" void canErrorCallback(std::uint32_t error_info);

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
constexpr unsigned NumberOfMessageObjects   = 32;
constexpr unsigned NumberOfTxMessageObjects = 1;
constexpr unsigned NumberOfRxMessageObjects = NumberOfMessageObjects - NumberOfTxMessageObjects;
constexpr unsigned TxMessageObjectNumber    = 1;

/**
 * Total number of CAN errors.
 * Does not overflow.
 */
std::uint32_t error_cnt = 0;

/**
 * True if there's no pending TX frame, i.e. write is possible.
 */
bool tx_free = true;

/**
 * Gets updated every time the CAN IRQ handler is being called.
 */
std::uint64_t last_irq_utc_timestamp = 0;

bool had_activity;

/**
 * After a received message gets extracted from C_CAN, it will be stored in the RX queue until libuavcan
 * reads it via select()/receive() calls.
 */
class RxQueue
{
    struct Item
    {
        std::uint64_t utc_usec = 0;
        uavcan::CanFrame frame;
    };

    Item buf_[UAVCAN_LPC11C24_RX_QUEUE_LEN];
    std::uint32_t overflow_cnt_ = 0;
    std::uint8_t in_ = 0;
    std::uint8_t out_ = 0;
    std::uint8_t len_ = 0;

public:
    void push(const uavcan::CanFrame& frame, const std::uint64_t& utc_usec)
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

    void pop(uavcan::CanFrame& out_frame, std::uint64_t& out_utc_usec)
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

    std::uint32_t getOverflowCount() const { return overflow_cnt_; }
};

RxQueue rx_queue;


struct BitTimingSettings
{
    std::uint32_t canclkdiv;
    std::uint32_t canbtr;

    bool isValid() const { return canbtr != 0; }
};

/**
 * http://www.bittiming.can-wiki.info
 */
BitTimingSettings computeBitTimings(std::uint32_t bitrate)
{
    if (Chip_Clock_GetSystemClockRate() == 48000000) // 48 MHz is optimal for CAN timings
    {
        switch (bitrate)
        {
        case 1000000: return BitTimingSettings{ 0, 0x0505 }; // 8  quanta, 87.5%
        case 500000:  return BitTimingSettings{ 0, 0x1c05 }; // 16 quanta, 87.5%
        case 250000:  return BitTimingSettings{ 0, 0x1c0b }; // 16 quanta, 87.5%
        case 125000:  return BitTimingSettings{ 0, 0x1c17 }; // 16 quanta, 87.5%
        default:      return BitTimingSettings{ 0, 0 };
        }
    }
    else
    {
        return BitTimingSettings{ 0, 0 };
    }
}

} // namespace

CanDriver CanDriver::self;

uavcan::uint32_t CanDriver::detectBitRate(void (*idle_callback)())
{
    static constexpr uavcan::uint32_t BitRatesToTry[] =
    {
        1000000,
        500000,
        250000,
        125000
    };

    const auto ListeningDuration = uavcan::MonotonicDuration::fromMSec(1050);

    NVIC_DisableIRQ(CAN_IRQn);
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_CAN);

    for (auto bitrate : BitRatesToTry)
    {
        // Computing bit timings
        const auto bit_timings = computeBitTimings(bitrate);
        if (!bit_timings.isValid())
        {
            return 0;
        }

        // Configuring the CAN controller
        {
            CriticalSectionLocker locker;

            LPC_SYSCTL->PRESETCTRL |= (1U << RESET_CAN0);

            // Entering initialization mode
            c_can::CAN.CNTL = c_can::CNTL_INIT | c_can::CNTL_CCE;

            while ((c_can::CAN.CNTL & c_can::CNTL_INIT) == 0)
            {
                ; // Nothing to do
            }

            // Configuring bit rate
            c_can::CAN.CLKDIV = bit_timings.canclkdiv;
            c_can::CAN.BT     = bit_timings.canbtr;
            c_can::CAN.BRPE   = 0;

            // Configuring silent mode
            c_can::CAN.CNTL |= c_can::CNTL_TEST;
            c_can::CAN.TEST = c_can::TEST_SILENT;

            // Configuring status monitor
            c_can::CAN.STAT = (unsigned(c_can::StatLec::Unused) << c_can::STAT_LEC_SHIFT);

            // Leaving initialization mode
            c_can::CAN.CNTL &= ~(c_can::CNTL_INIT | c_can::CNTL_CCE);

            while ((c_can::CAN.CNTL & c_can::CNTL_INIT) != 0)
            {
                ; // Nothing to do
            }
        }

        // Listening
        const auto deadline = clock::getMonotonic() + ListeningDuration;
        bool match_detected = false;
        while (clock::getMonotonic() < deadline)
        {
            if (idle_callback != nullptr)
            {
                idle_callback();
            }

            const auto LastErrorCode = (c_can::CAN.STAT >> c_can::STAT_LEC_SHIFT) & c_can::STAT_LEC_MASK;

            if (LastErrorCode == unsigned(c_can::StatLec::NoError))
            {
                match_detected = true;
                break;
            }
        }

        // De-configuring the CAN controller back to reset state
        {
            CriticalSectionLocker locker;

            c_can::CAN.CNTL = c_can::CNTL_INIT;

            while ((c_can::CAN.CNTL & c_can::CNTL_INIT) == 0)
            {
                ; // Nothing to do
            }

            LPC_SYSCTL->PRESETCTRL &= ~(1U << RESET_CAN0);
        }

        // Termination condition
        if (match_detected)
        {
            return bitrate;
        }
    }

    return 0;   // No match
}

int CanDriver::init(uavcan::uint32_t bitrate)
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
    auto bit_timings = computeBitTimings(bitrate);
    if (!bit_timings.isValid())
    {
        return -1;
    }
    LPC_CCAN_API->init_can(reinterpret_cast<std::uint32_t*>(&bit_timings), true);
    static CCAN_CALLBACKS_T ccan_callbacks =
    {
        canRxCallback,
        canTxCallback,
        canErrorCallback,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr
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
        msgobj.msgobj = TxMessageObjectNumber;
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
    std::uint64_t ts_utc = 0;
    rx_queue.pop(out_frame, ts_utc);
    out_ts_utc = uavcan::UtcTime::fromUSec(ts_utc);
    return 1;
}

uavcan::int16_t CanDriver::select(uavcan::CanSelectMasks& inout_masks,
                                  const uavcan::CanFrame* (&)[uavcan::MaxCanIfaces],
                                  uavcan::MonotonicTime blocking_deadline)
{
    const bool noblock = ((inout_masks.read  == 1) && hasReadyRx()) ||
                         ((inout_masks.write == 1) && hasEmptyTx());

    if (!noblock && (clock::getMonotonic() > blocking_deadline))
    {
#if defined(UAVCAN_LPC11C24_USE_WFE) && UAVCAN_LPC11C24_USE_WFE
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
    return std::uint64_t(error_cnt) + std::uint64_t(rx_queue.getOverflowCount());
}

uavcan::uint16_t CanDriver::getNumFilters() const
{
    return NumberOfRxMessageObjects;
}

uavcan::ICanIface* CanDriver::getIface(uavcan::uint8_t iface_index)
{
    return (iface_index == 0) ? this : nullptr;
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

void canRxCallback(std::uint8_t msg_obj_num)
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

void canTxCallback(std::uint8_t msg_obj_num)
{
    (void)msg_obj_num;
    uavcan_lpc11c24::tx_free = true;
    uavcan_lpc11c24::had_activity = true;
}

void canErrorCallback(std::uint32_t error_info)
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
