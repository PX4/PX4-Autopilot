/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <cstring>
#include <uavcan_stm32/can.hpp>
#include <uavcan_stm32/clock.hpp>
#include "internal.hpp"

#if UAVCAN_STM32_CHIBIOS
# include <hal.h>
#elif UAVCAN_STM32_NUTTX
# include <nuttx/arch.h>
# include <nuttx/irq.h>
# include <arch/board/board.h>
#else
# error "Unknown OS"
#endif

#if !UAVCAN_STM32_NUTTX
# if !(defined(STM32F10X_CL) || defined(STM32F2XX) || defined(STM32F4XX))
// IRQ numbers
#  define CAN1_RX0_IRQn USB_LP_CAN1_RX0_IRQn
#  define CAN1_TX_IRQn USB_HP_CAN1_TX_IRQn
// IRQ vectors
#  if !defined(CAN1_RX0_IRQHandler) || !defined(CAN1_TX_IRQHandler)
#   define CAN1_TX_IRQHandler   USB_HP_CAN1_TX_IRQHandler
#   define CAN1_RX0_IRQHandler  USB_LP_CAN1_RX0_IRQHandler
#  endif
# endif
#endif

#if UAVCAN_STM32_NUTTX
# if !defined(STM32_IRQ_CAN1TX) && !defined(STM32_IRQ_CAN1RX0)
#  define STM32_IRQ_CAN1TX      STM32_IRQ_USBHPCANTX
#  define STM32_IRQ_CAN1RX0     STM32_IRQ_USBLPCANRX0
# endif
extern "C"
{
static int can1_irq(const int irq, void*);
#if UAVCAN_STM32_NUM_IFACES > 1
static int can2_irq(const int irq, void*);
#endif
}
#endif

namespace uavcan_stm32
{
namespace
{

CanIface* ifaces[UAVCAN_STM32_NUM_IFACES] =
{
    NULL
#if UAVCAN_STM32_NUM_IFACES > 1
    , NULL
#endif
};

inline void handleTxInterrupt(uavcan::uint8_t iface_index)
{
    UAVCAN_ASSERT(iface_index < UAVCAN_STM32_NUM_IFACES);
    uavcan::uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();
    if (utc_usec > 0)
    {
        utc_usec--;
    }
    if (ifaces[iface_index] != NULL)
    {
        ifaces[iface_index]->handleTxInterrupt(utc_usec);
    }
    else
    {
        UAVCAN_ASSERT(0);
    }
}

inline void handleRxInterrupt(uavcan::uint8_t iface_index, uavcan::uint8_t fifo_index)
{
    UAVCAN_ASSERT(iface_index < UAVCAN_STM32_NUM_IFACES);
    uavcan::uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();
    if (utc_usec > 0)
    {
        utc_usec--;
    }
    if (ifaces[iface_index] != NULL)
    {
        ifaces[iface_index]->handleRxInterrupt(fifo_index, utc_usec);
    }
    else
    {
        UAVCAN_ASSERT(0);
    }
}

} // namespace

/*
 * CanIface::RxQueue
 */
void CanIface::RxQueue::registerOverflow()
{
    if (overflow_cnt_ < 0xFFFFFFFF)
    {
        overflow_cnt_++;
    }
}

void CanIface::RxQueue::push(const uavcan::CanFrame& frame, const uint64_t& utc_usec, uavcan::CanIOFlags flags)
{
    buf_[in_].frame    = frame;
    buf_[in_].utc_usec = utc_usec;
    buf_[in_].flags    = flags;
    in_++;
    if (in_ >= capacity_)
    {
        in_ = 0;
    }
    len_++;
    if (len_ > capacity_)
    {
        len_ = capacity_;
        registerOverflow();
        out_++;
        if (out_ >= capacity_)
        {
            out_ = 0;
        }
    }
}

void CanIface::RxQueue::pop(uavcan::CanFrame& out_frame, uavcan::uint64_t& out_utc_usec, uavcan::CanIOFlags& out_flags)
{
    if (len_ > 0)
    {
        out_frame    = buf_[out_].frame;
        out_utc_usec = buf_[out_].utc_usec;
        out_flags    = buf_[out_].flags;
        out_++;
        if (out_ >= capacity_)
        {
            out_ = 0;
        }
        len_--;
    }
    else { UAVCAN_ASSERT(0); }
}

/*
 * CanIface
 */
int CanIface::computeTimings(const uavcan::uint32_t target_bitrate, Timings& out_timings)
{
    /*
     *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))
     *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))
     * let:
     *   BS = 1 + BS1 + BS2
     *   PRESCALER_BS = PRESCALER * BS
     * ==>
     *   PRESCALER_BS = PCLK / BITRATE
     */
    if (target_bitrate < 20000 || target_bitrate > 1000000)
    {
        return -1;
    }

#if UAVCAN_STM32_CHIBIOS
    const uavcan::uint32_t pclk = STM32_PCLK1;
#elif UAVCAN_STM32_NUTTX
    const uavcan::uint32_t pclk = STM32_PCLK1_FREQUENCY;
#else
# error "Unknown OS"
#endif
    const uavcan::uint32_t prescaler_bs = pclk / target_bitrate;

    // Initial guess:
    uavcan::int8_t bs1 = 10;  // max 15
    uavcan::int8_t bs2 = 5;   // max 8
    uavcan::uint16_t prescaler = 0;

    while (1)
    {
        prescaler = uavcan::uint16_t(prescaler_bs / unsigned(1 + bs1 + bs2));
        // Check result:
        if ((prescaler >= 1) && (prescaler <= 1024))
        {
            const uavcan::uint32_t current_bitrate = pclk / (prescaler * unsigned(1 + bs1 + bs2));
            if (current_bitrate == target_bitrate)
            {
                break;
            }
        }
        if (bs1 > bs2)
        {
            bs1--;
        }
        else
        {
            bs2--;
        }
        if (bs1 <= 0 || bs2 <= 0)
        {
            return -1;
        }
    }
    if (!((prescaler >= 1) && (prescaler <= 1024)))
    {
        return -1;
    }
    out_timings.prescaler = uavcan::uint16_t(prescaler - 1U);
    out_timings.sjw = 1;
    out_timings.bs1 = uavcan::uint8_t(bs1 - 1);
    out_timings.bs2 = uavcan::uint8_t(bs2 - 1);
    return 0;
}

uavcan::int16_t CanIface::send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                               uavcan::CanIOFlags flags)
{
    if (frame.isErrorFrame() || frame.dlc > 8)
    {
        return -1;  // WTF man how to handle that
    }

    CriticalSectionLocker lock;

    /*
     * Seeking for an empty slot
     */
    uavcan::uint8_t txmailbox = 0xFF;
    if ((can_->TSR & bxcan::TSR_TME0) == bxcan::TSR_TME0)
    {
        txmailbox = 0;
    }
    else if ((can_->TSR & bxcan::TSR_TME1) == bxcan::TSR_TME1)
    {
        txmailbox = 1;
    }
    else if ((can_->TSR & bxcan::TSR_TME2) == bxcan::TSR_TME2)
    {
        txmailbox = 2;
    }
    else { return 0; } // No way

    /*
     * Setting up the mailbox
     */
    bxcan::TxMailboxType& mb = can_->TxMailbox[txmailbox];
    if (frame.isExtended())
    {
        mb.TIR = ((frame.id & uavcan::CanFrame::MaskExtID) << 3) | bxcan::TIR_IDE;
    }
    else
    {
        mb.TIR = ((frame.id & uavcan::CanFrame::MaskStdID) << 21);
    }

    if (frame.isRemoteTransmissionRequest())
    {
        mb.TIR |= bxcan::TIR_RTR;
    }

    mb.TDTR = frame.dlc;

    mb.TDHR = (uavcan::uint32_t(frame.data[7]) << 24) |
              (uavcan::uint32_t(frame.data[6]) << 16) |
              (uavcan::uint32_t(frame.data[5]) << 8)  |
              (uavcan::uint32_t(frame.data[4]) << 0);
    mb.TDLR = (uavcan::uint32_t(frame.data[3]) << 24) |
              (uavcan::uint32_t(frame.data[2]) << 16) |
              (uavcan::uint32_t(frame.data[1]) << 8)  |
              (uavcan::uint32_t(frame.data[0]) << 0);

    mb.TIR |= bxcan::TIR_TXRQ;  // Go.

    /*
     * Registering the pending transmission so we can track its deadline and loopback it as needed
     */
    TxItem& txi = pending_tx_[txmailbox];
    txi.deadline = tx_deadline;
    txi.frame    = frame;
    txi.loopback = (flags & uavcan::CanIOFlagLoopback) == uavcan::CanIOFlagLoopback;
    txi.pending  = true;
    return 1;
}

uavcan::int16_t CanIface::receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                                  uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
{
    out_ts_monotonic = clock::getMonotonic();  // High precision is not required for monotonic timestamps
    uavcan::uint64_t utc_usec = 0;
    {
        CriticalSectionLocker lock;
        if (rx_queue_.getLength() == 0)
        {
            return 0;
        }
        rx_queue_.pop(out_frame, utc_usec, out_flags);
    }
    out_ts_utc = uavcan::UtcTime::fromUSec(utc_usec);
    return 1;
}

uavcan::int16_t CanIface::configureFilters(const uavcan::CanFilterConfig* filter_configs,
                                           uavcan::uint16_t num_configs)
{
    // TODO: Hardware filter support
    CriticalSectionLocker lock;
    (void)filter_configs;
    (void)num_configs;
    return -1;
}

bool CanIface::waitMsrINakBitStateChange(bool target_state)
{
#if UAVCAN_STM32_NUTTX
    const unsigned Timeout = 500;
#else
    const unsigned Timeout = 2000000;
#endif
    for (unsigned wait_ack = 0; wait_ack < Timeout; wait_ack++)
    {
        const bool state = (can_->MSR & bxcan::MSR_INAK) != 0;
        if (state == target_state)
        {
            return true;
        }
#if UAVCAN_STM32_NUTTX
        ::usleep(2000);
#endif
    }
    return false;
}

int CanIface::init(uavcan::uint32_t bitrate)
{
    int res = 0;

    /*
     * CAN timings for this bitrate
     */
    Timings timings;
    res = computeTimings(bitrate, timings);
    if (res < 0)
    {
        goto leave;
    }
    UAVCAN_STM32_LOG("Timings: presc=%u sjw=%u bs1=%u bs2=%u",
                     unsigned(timings.prescaler), unsigned(timings.sjw), unsigned(timings.bs1), unsigned(timings.bs2));

    /*
     * Hardware initialization
     */
    can_->MCR &= ~bxcan::MCR_SLEEP; // Exit sleep mode
    can_->MCR |= bxcan::MCR_INRQ;   // Request init

    if (!waitMsrINakBitStateChange(true))
    {
        UAVCAN_STM32_LOG("MSR INAK not set");
        res = -1;
        goto leave;
    }

    can_->MCR = bxcan::MCR_ABOM | bxcan::MCR_AWUM | bxcan::MCR_INRQ;  // RM page 648

    can_->BTR = ((timings.sjw & 3U)  << 24) |
                ((timings.bs1 & 15U) << 16) |
                ((timings.bs2 & 7U)  << 20) |
                (timings.prescaler & 1023U);

    can_->IER = bxcan::IER_TMEIE |   // TX mailbox empty
                bxcan::IER_FMPIE0 |  // RX FIFO 0 is not empty
                bxcan::IER_FMPIE1;   // RX FIFO 1 is not empty

    can_->MCR &= ~bxcan::MCR_INRQ;  // Leave init mode

    if (!waitMsrINakBitStateChange(false))
    {
        UAVCAN_STM32_LOG("MSR INAK not cleared");
        res = -1;
        goto leave;
    }

    /*
     * Default filter configuration
     */
    if (self_index_ == 0)
    {
        can_->FMR |= bxcan::FMR_FINIT;

        can_->FMR &= 0xFFFFC0F1;
        can_->FMR |= static_cast<uavcan::uint32_t>(NumFilters) << 8;  // Slave (CAN2) gets half of the filters

        can_->FFA1R = 0;                           // All assigned to FIFO0 by default
        can_->FM1R = 0;                            // Indentifier Mask mode

#if UAVCAN_STM32_NUM_IFACES > 1
        can_->FS1R = 0x7ffffff;                    // Single 32-bit for all
        can_->FilterRegister[0].FR1 = 0;          // CAN1 accepts everything
        can_->FilterRegister[0].FR2 = 0;
        can_->FilterRegister[NumFilters].FR1 = 0; // CAN2 accepts everything
        can_->FilterRegister[NumFilters].FR2 = 0;
        can_->FA1R = 1 | (1 << NumFilters);        // One filter per each iface
#else
        can_->FS1R = 0x1fff;
        can_->FilterRegister[0].FR1 = 0;
        can_->FilterRegister[0].FR2 = 0;
        can_->FA1R = 1;
#endif

        can_->FMR &= ~bxcan::FMR_FINIT;
    }

leave:
    return res;
}

void CanIface::pollErrorState()
{
    const uavcan::uint8_t lec = uavcan::uint8_t((can_->ESR & bxcan::ESR_LEC_MASK) >> bxcan::ESR_LEC_SHIFT);
    if (lec != 0)
    {
        last_hw_error_code_ = lec;
        can_->ESR = 0;
        error_cnt_++;
    }
}

void CanIface::handleTxMailboxInterrupt(uavcan::uint8_t mailbox_index, bool txok, const uavcan::uint64_t utc_usec)
{
    UAVCAN_ASSERT(mailbox_index < NumTxMailboxes);

    had_activity_ = had_activity_ || txok;

    TxItem& txi = pending_tx_[mailbox_index];
    if (txi.loopback && txok && txi.pending)
    {
        rx_queue_.push(txi.frame, utc_usec, uavcan::CanIOFlagLoopback);
    }
    if (!txok)
    {
        error_cnt_++;
    }
    txi.pending = false;
}

void CanIface::handleTxInterrupt(const uavcan::uint64_t utc_usec)
{
    // TXOK == false means that there was a hardware failure
    if (can_->TSR & bxcan::TSR_RQCP0)
    {
        const bool txok = can_->TSR & bxcan::TSR_TXOK0;
        can_->TSR = bxcan::TSR_RQCP0;
        handleTxMailboxInterrupt(0, txok, utc_usec);
    }
    if (can_->TSR & bxcan::TSR_RQCP1)
    {
        const bool txok = can_->TSR & bxcan::TSR_TXOK1;
        can_->TSR = bxcan::TSR_RQCP1;
        handleTxMailboxInterrupt(1, txok, utc_usec);
    }
    if (can_->TSR & bxcan::TSR_RQCP2)
    {
        const bool txok = can_->TSR & bxcan::TSR_TXOK2;
        can_->TSR = bxcan::TSR_RQCP2;
        handleTxMailboxInterrupt(2, txok, utc_usec);
    }
    pollErrorState();
    update_event_.signalFromInterrupt();
}

void CanIface::handleRxInterrupt(uavcan::uint8_t fifo_index, uavcan::uint64_t utc_usec)
{
    UAVCAN_ASSERT(fifo_index < 2);

    volatile uavcan::uint32_t* const rfr_reg = (fifo_index == 0) ? &can_->RF0R : &can_->RF1R;
    if ((*rfr_reg & bxcan::RFR_FMP_MASK) == 0)
    {
        UAVCAN_ASSERT(0);  // Weird, IRQ is here but no data to read
        return;
    }

    /*
     * Register overflow as a hardware error
     */
    if ((*rfr_reg & bxcan::RFR_FOVR) != 0)
    {
        error_cnt_++;
    }

    /*
     * Read the frame contents
     */
    uavcan::CanFrame frame;
    const bxcan::RxMailboxType& rf = can_->RxMailbox[fifo_index];

    if ((rf.RIR & bxcan::RIR_IDE) == 0)
    {
        frame.id = uavcan::CanFrame::MaskStdID & (rf.RIR >> 21);
    }
    else
    {
        frame.id = uavcan::CanFrame::MaskExtID & (rf.RIR >> 3);
        frame.id |= uavcan::CanFrame::FlagEFF;
    }

    if ((rf.RIR & bxcan::RIR_RTR) != 0)
    {
        frame.id |= uavcan::CanFrame::FlagRTR;
    }

    frame.dlc = rf.RDTR & 15;

    frame.data[0] = uavcan::uint8_t(0xFF & (rf.RDLR >> 0));
    frame.data[1] = uavcan::uint8_t(0xFF & (rf.RDLR >> 8));
    frame.data[2] = uavcan::uint8_t(0xFF & (rf.RDLR >> 16));
    frame.data[3] = uavcan::uint8_t(0xFF & (rf.RDLR >> 24));
    frame.data[4] = uavcan::uint8_t(0xFF & (rf.RDHR >> 0));
    frame.data[5] = uavcan::uint8_t(0xFF & (rf.RDHR >> 8));
    frame.data[6] = uavcan::uint8_t(0xFF & (rf.RDHR >> 16));
    frame.data[7] = uavcan::uint8_t(0xFF & (rf.RDHR >> 24));

    *rfr_reg = bxcan::RFR_RFOM | bxcan::RFR_FOVR | bxcan::RFR_FULL;  // Release FIFO entry we just read

    /*
     * Store with timeout into the FIFO buffer and signal update event
     */
    rx_queue_.push(frame, utc_usec, 0);
    had_activity_ = true;
    pollErrorState();
    update_event_.signalFromInterrupt();
}

void CanIface::discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time)
{
    static const uavcan::uint32_t AbortFlags[NumTxMailboxes] = { bxcan::TSR_ABRQ0, bxcan::TSR_ABRQ1, bxcan::TSR_ABRQ2 };
    CriticalSectionLocker lock;
    for (int i = 0; i < NumTxMailboxes; i++)
    {
        TxItem& txi = pending_tx_[i];
        if (txi.pending && txi.deadline < current_time)
        {
            can_->TSR = AbortFlags[i];  // Goodnight sweet transmission
            txi.pending = false;
            error_cnt_++;
        }
    }
}

bool CanIface::isTxBufferFull() const
{
    return (can_->TSR & (bxcan::TSR_TME0 | bxcan::TSR_TME1 | bxcan::TSR_TME2)) == 0;  // Interrupts enabled
}

bool CanIface::isRxBufferEmpty() const
{
    CriticalSectionLocker lock;
    return rx_queue_.getLength() == 0;
}

uavcan::uint64_t CanIface::getErrorCount() const
{
    CriticalSectionLocker lock;
    return error_cnt_ + rx_queue_.getOverflowCount();
}

unsigned CanIface::getRxQueueLength() const
{
    CriticalSectionLocker lock;
    return rx_queue_.getLength();
}

uavcan::uint8_t CanIface::yieldLastHardwareErrorCode()
{
    CriticalSectionLocker lock;
    const uavcan::uint8_t val = last_hw_error_code_;
    last_hw_error_code_ = 0;
    return val;
}

bool CanIface::hadActivity()
{
    CriticalSectionLocker lock;
    const bool ret = had_activity_;
    had_activity_ = false;
    return ret;
}

/*
 * CanDriver
 */
uavcan::CanSelectMasks CanDriver::makeSelectMasks() const
{
    uavcan::CanSelectMasks msk;
    // Iface 0
    msk.read  = if0_.isRxBufferEmpty() ? 0 : 1;
    msk.write = if0_.isTxBufferFull()  ? 0 : 1;
    // Iface 1
#if UAVCAN_STM32_NUM_IFACES > 1
    if (!if1_.isRxBufferEmpty())
    {
        msk.read |= 1 << 1;
    }
    if (!if1_.isTxBufferFull())
    {
        msk.write |= 1 << 1;
    }
#endif
    return msk;
}

uavcan::int16_t CanDriver::select(uavcan::CanSelectMasks& inout_masks, const uavcan::MonotonicTime blocking_deadline)
{
    const uavcan::CanSelectMasks in_masks = inout_masks;
    const uavcan::MonotonicTime time = clock::getMonotonic();

    if0_.discardTimedOutTxMailboxes(time);              // Check TX timeouts - this may release some TX slots
#if UAVCAN_STM32_NUM_IFACES > 1
    if1_.discardTimedOutTxMailboxes(time);
#endif

    inout_masks = makeSelectMasks();                    // Check if we already have some of the requested events
    if ((inout_masks.read  & in_masks.read)  != 0 ||
        (inout_masks.write & in_masks.write) != 0)
    {
        return 1;
    }

    (void)update_event_.wait(blocking_deadline - time); // Block until timeout expires or any iface updates
    inout_masks = makeSelectMasks(); // Return what we got even if none of the requested events became signaled
    return 1;                        // Return value doesn't matter as long as it is non-negative
}

int CanDriver::init(uavcan::uint32_t bitrate)
{
    int res = 0;

    UAVCAN_STM32_LOG("Bitrate %lu", static_cast<unsigned long>(bitrate));

    /*
     * CAN1
     */
    {
        CriticalSectionLocker lock;
#if UAVCAN_STM32_NUTTX
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_CAN1EN);
        modifyreg32(STM32_RCC_APB1RSTR, 0, RCC_APB1RSTR_CAN1RST);
        modifyreg32(STM32_RCC_APB1RSTR, RCC_APB1RSTR_CAN1RST, 0);
#else
        RCC->APB1ENR  |=  RCC_APB1ENR_CAN1EN;
        RCC->APB1RSTR |=  RCC_APB1RSTR_CAN1RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;
#endif
    }

    UAVCAN_STM32_LOG("Initing iface 0...");
    res = if0_.init(bitrate);
    if (res < 0)
    {
        UAVCAN_STM32_LOG("Iface 0 init failed %i", res);
        goto fail;
    }
    ifaces[0] = &if0_;

    /*
     * CAN2
     */
#if UAVCAN_STM32_NUM_IFACES > 1
    {
        CriticalSectionLocker lock;
# if UAVCAN_STM32_NUTTX
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_CAN2EN);
        modifyreg32(STM32_RCC_APB1RSTR, 0, RCC_APB1RSTR_CAN2RST);
        modifyreg32(STM32_RCC_APB1RSTR, RCC_APB1RSTR_CAN2RST, 0);
# else
        RCC->APB1ENR  |=  RCC_APB1ENR_CAN2EN;
        RCC->APB1RSTR |=  RCC_APB1RSTR_CAN2RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN2RST;
# endif
    }

    UAVCAN_STM32_LOG("Initing iface 1...");
    res = if1_.init(bitrate);
    if (res < 0)
    {
        UAVCAN_STM32_LOG("Iface 1 init failed %i", res);
        goto fail;
    }
    ifaces[1] = &if1_;
#endif

    /*
     * IRQ
     */
#if UAVCAN_STM32_NUTTX
# define IRQ_ATTACH(irq, handler)                          \
    {                                                      \
        res = irq_attach(irq, handler);                    \
        if (res < 0)                                       \
        {                                                  \
            UAVCAN_STM32_LOG("IRQ attach failed %i", irq); \
            goto fail;                                     \
        }                                                  \
        up_enable_irq(irq);                                \
    }
    IRQ_ATTACH(STM32_IRQ_CAN1TX,  can1_irq);
    IRQ_ATTACH(STM32_IRQ_CAN1RX0, can1_irq);
    IRQ_ATTACH(STM32_IRQ_CAN1RX1, can1_irq);
# if UAVCAN_STM32_NUM_IFACES > 1
    IRQ_ATTACH(STM32_IRQ_CAN2TX,  can2_irq);
    IRQ_ATTACH(STM32_IRQ_CAN2RX0, can2_irq);
    IRQ_ATTACH(STM32_IRQ_CAN2RX1, can2_irq);
# endif
# undef IRQ_ATTACH
#else
    {
        CriticalSectionLocker lock;
        nvicEnableVector(CAN1_TX_IRQn,  UAVCAN_STM32_IRQ_PRIORITY_MASK);
        nvicEnableVector(CAN1_RX0_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
        nvicEnableVector(CAN1_RX1_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
# if UAVCAN_STM32_NUM_IFACES > 1
        nvicEnableVector(CAN2_TX_IRQn,  UAVCAN_STM32_IRQ_PRIORITY_MASK);
        nvicEnableVector(CAN2_RX0_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
        nvicEnableVector(CAN2_RX1_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
# endif
    }
#endif

    UAVCAN_STM32_LOG("CAN drv init OK");
    UAVCAN_ASSERT(res >= 0);
    return res;

fail:
    UAVCAN_STM32_LOG("CAN drv init failed %i", res);
    UAVCAN_ASSERT(res < 0);

    CriticalSectionLocker lock;

#if UAVCAN_STM32_NUTTX
    // TODO: Unattach and disable all IRQs
    modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_CAN1EN, 0);
# if UAVCAN_STM32_NUM_IFACES > 1
    modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_CAN2EN, 0);
# endif
#else
    RCC->APB1ENR &= ~RCC_APB1ENR_CAN1EN;
# if UAVCAN_STM32_NUM_IFACES > 1
    RCC->APB1ENR &= ~RCC_APB1ENR_CAN2EN;
# endif
#endif
    return res;
}

CanIface* CanDriver::getIface(uavcan::uint8_t iface_index)
{
    if (iface_index < UAVCAN_STM32_NUM_IFACES)
    {
        return ifaces[iface_index];
    }
    return NULL;
}

bool CanDriver::hadActivity()
{
    bool ret = if0_.hadActivity();
#if UAVCAN_STM32_NUM_IFACES > 1
    ret |= if1_.hadActivity();
#endif
    return ret;
}

} // namespace uavcan_stm32

/*
 * Interrupt handlers
 */
extern "C"
{

#if UAVCAN_STM32_NUTTX

static int can1_irq(const int irq, void*)
{
    if (irq == STM32_IRQ_CAN1TX)
    {
        uavcan_stm32::handleTxInterrupt(0);
    }
    else if (irq == STM32_IRQ_CAN1RX0)
    {
        uavcan_stm32::handleRxInterrupt(0, 0);
    }
    else if (irq == STM32_IRQ_CAN1RX1)
    {
        uavcan_stm32::handleRxInterrupt(0, 1);
    }
    else
    {
        PANIC();
    }
    return 0;
}

# if UAVCAN_STM32_NUM_IFACES > 1

static int can2_irq(const int irq, void*)
{
    if (irq == STM32_IRQ_CAN2TX)
    {
        uavcan_stm32::handleTxInterrupt(1);
    }
    else if (irq == STM32_IRQ_CAN2RX0)
    {
        uavcan_stm32::handleRxInterrupt(1, 0);
    }
    else if (irq == STM32_IRQ_CAN2RX1)
    {
        uavcan_stm32::handleRxInterrupt(1, 1);
    }
    else
    {
        PANIC();
    }
    return 0;
}

# endif
#else // UAVCAN_STM32_NUTTX

UAVCAN_STM32_IRQ_HANDLER(CAN1_TX_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    uavcan_stm32::handleTxInterrupt(0);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

UAVCAN_STM32_IRQ_HANDLER(CAN1_RX0_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    uavcan_stm32::handleRxInterrupt(0, 0);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

UAVCAN_STM32_IRQ_HANDLER(CAN1_RX1_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    uavcan_stm32::handleRxInterrupt(0, 1);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

# if UAVCAN_STM32_NUM_IFACES > 1

UAVCAN_STM32_IRQ_HANDLER(CAN2_TX_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    uavcan_stm32::handleTxInterrupt(1);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

UAVCAN_STM32_IRQ_HANDLER(CAN2_RX0_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    uavcan_stm32::handleRxInterrupt(1, 0);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

UAVCAN_STM32_IRQ_HANDLER(CAN2_RX1_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    uavcan_stm32::handleRxInterrupt(1, 1);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

# endif
#endif // UAVCAN_STM32_NUTTX

} // extern "C"
