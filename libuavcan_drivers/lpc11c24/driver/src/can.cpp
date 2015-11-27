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
volatile std::uint32_t error_cnt = 0;

/**
 * False if there's no pending TX frame, i.e. write is possible.
 */
volatile bool tx_pending = false;

/**
 * Currently pending frame must be aborted on first error.
 */
volatile bool tx_abort_on_error = false;

/**
 * Gets updated every time the CAN IRQ handler is being called.
 */
volatile std::uint64_t last_irq_utc_timestamp = 0;

/**
 * Set by the driver on every successful TX or RX; reset by the application.
 */
volatile bool had_activity = false;

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
    void push(const uavcan::CanFrame& frame, const volatile std::uint64_t& utc_usec)
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
        case 100000:  return BitTimingSettings{ 0, 0x1c1d }; // 16 quanta, 87.5%
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
        125000,
        100000
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
    {
        auto bit_timings = computeBitTimings(bitrate);
        if (!bit_timings.isValid())
        {
            return -1;
        }

        CriticalSectionLocker locker;

        error_cnt = 0;
        tx_abort_on_error = false;
        tx_pending = false;
        last_irq_utc_timestamp = 0;
        had_activity = false;

        /*
         * C_CAN init
         */
        Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_CAN);

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

        /*
         * Interrupts
         */
        c_can::CAN.CNTL |= c_can::CNTL_SIE;         // This is necessary for transmission aborts on error

        NVIC_EnableIRQ(CAN_IRQn);
    }

    /*
     * Applying default filter configuration (accept all)
     */
    if (configureFilters(nullptr, 0) < 0)
    {
        return -1;
    }

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
    return !tx_pending;
}

bool CanDriver::hadActivity()
{
    CriticalSectionLocker locker;
    const bool ret = had_activity;
    had_activity = false;
    return ret;
}

uavcan::uint32_t CanDriver::getRxQueueOverflowCount() const
{
    CriticalSectionLocker locker;
    return rx_queue.getOverflowCount();
}

bool CanDriver::isInBusOffState() const
{
    return (c_can::CAN.STAT & c_can::STAT_BOFF) != 0;
}

uavcan::int16_t CanDriver::send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                                uavcan::CanIOFlags flags)
{
    if (frame.isErrorFrame() ||
        frame.dlc > 8 ||
        (flags & uavcan::CanIOFlagLoopback) != 0)   // TX timestamping is not supported by this driver.
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

    if (!tx_pending)
    {
        tx_pending = true;   // Mark as pending - will be released in TX callback
        tx_abort_on_error = (flags & uavcan::CanIOFlagAbortOnError) != 0;
        msgobj.msgobj = TxMessageObjectNumber;
        LPC_CCAN_API->can_transmit(&msgobj);
        return 1;
    }
    return 0;
}

uavcan::int16_t CanDriver::receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                                   uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
{
    out_ts_monotonic = clock::getMonotonic();
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
    const bool bus_off = isInBusOffState();
    if (bus_off)                                // Recover automatically on bus-off
    {
        CriticalSectionLocker locker;
        if ((c_can::CAN.CNTL & c_can::CNTL_INIT) != 0)
        {
            c_can::CAN.CNTL &= ~c_can::CNTL_INIT;
        }
    }

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

    inout_masks.write = (hasEmptyTx() && !bus_off) ? 1 : 0;     // Disable write while in bus-off

    return 0;           // Return value doesn't matter as long as it is non-negative
}

uavcan::int16_t CanDriver::configureFilters(const uavcan::CanFilterConfig* filter_configs,
                                            uavcan::uint16_t num_configs)
{
    CriticalSectionLocker locker;

    /*
     * If C_CAN is active (INIT=0) and the CAN bus has intensive traffic, RX object configuration may fail.
     * The solution is to disable the controller while configuration is in progress.
     * The documentation, as always, doesn't bother to mention this detail. Shame on you, NXP.
     */
    struct RAIIDisabler
    {
        RAIIDisabler()
        {
            c_can::CAN.CNTL |= c_can::CNTL_INIT;
        }
        ~RAIIDisabler()
        {
            c_can::CAN.CNTL &= ~c_can::CNTL_INIT;
        }
    } can_disabler;    // Must be instantiated AFTER the critical section locker

    if (num_configs == 0)
    {
        auto msg_obj = CCAN_MSG_OBJ_T();
        msg_obj.msgobj = NumberOfTxMessageObjects + 1;
        LPC_CCAN_API->config_rxmsgobj(&msg_obj);    // all STD frames

        msg_obj.mode_id = CAN_MSGOBJ_EXT;
        msg_obj.msgobj = NumberOfTxMessageObjects + 2;
        LPC_CCAN_API->config_rxmsgobj(&msg_obj);    // all EXT frames
    }
    else if (num_configs <= NumberOfRxMessageObjects)
    {
        // Making sure the configs use only EXT frames; otherwise we can't accept them
        for (unsigned i = 0; i < num_configs; i++)
        {
            auto& f = filter_configs[i];
            if ((f.id & f.mask & uavcan::CanFrame::FlagEFF) == 0)
            {
                return -1;
            }
        }

        // Installing the configuration
        for (unsigned i = 0; i < NumberOfRxMessageObjects; i++)
        {
            auto msg_obj = CCAN_MSG_OBJ_T();
            msg_obj.msgobj = std::uint8_t(i + 1U + NumberOfTxMessageObjects);   // Message objects are numbered from 1

            if (i < num_configs)
            {
                msg_obj.mode_id = (filter_configs[i].id & uavcan::CanFrame::MaskExtID) | CAN_MSGOBJ_EXT; // Only EXT
                msg_obj.mask    = filter_configs[i].mask & uavcan::CanFrame::MaskExtID;
            }
            else
            {
                msg_obj.mode_id = CAN_MSGOBJ_RTR;               // Using this configuration to disable the object
                msg_obj.mask    = uavcan::CanFrame::MaskStdID;
            }

            LPC_CCAN_API->config_rxmsgobj(&msg_obj);
        }
    }
    else
    {
        return -1;
    }

    return 0;
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
    using namespace uavcan_lpc11c24;

    auto msg_obj = CCAN_MSG_OBJ_T();
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

    rx_queue.push(frame, last_irq_utc_timestamp);
    had_activity = true;
}

void canTxCallback(std::uint8_t msg_obj_num)
{
    using namespace uavcan_lpc11c24;

    (void)msg_obj_num;

    tx_pending = false;
    had_activity = true;
}

void canErrorCallback(std::uint32_t error_info)
{
    using namespace uavcan_lpc11c24;

    // Updating the error counter
    if ((error_info != 0) && (error_cnt < 0xFFFFFFFFUL))
    {
        error_cnt++;
    }

    // Serving abort requests
    if (tx_pending && tx_abort_on_error)
    {
        tx_pending = false;
        tx_abort_on_error = false;

        // Using the first interface, because this approach seems to be compliant with the BASIC mode (just in case)
        c_can::CAN.IF[0].CMDREQ = TxMessageObjectNumber;
        c_can::CAN.IF[0].CMDMSK.W = c_can::IF_CMDMSK_W_WR_RD;   // Clearing IF_CMDMSK_W_TXRQST
        c_can::CAN.IF[0].MCTRL &= ~c_can::IF_MCTRL_TXRQST;      // Clearing IF_MCTRL_TXRQST
    }
}

void CAN_IRQHandler();

void CAN_IRQHandler()
{
    using namespace uavcan_lpc11c24;

    last_irq_utc_timestamp = clock::getUtcUSecFromCanInterrupt();

    LPC_CCAN_API->isr();
}

}
