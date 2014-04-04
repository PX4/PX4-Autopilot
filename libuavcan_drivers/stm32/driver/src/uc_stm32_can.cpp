/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <cassert>
#include <cstring>
#include <uavcan_stm32/can.hpp>
#include <uavcan_stm32/clock.hpp>
#include "internal.hpp"

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
    uavcan::uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();
    if (utc_usec > 0)
    {
        utc_usec--;
    }
    if (iface_index < UAVCAN_STM32_NUM_IFACES &&
        ifaces[iface_index] != NULL)
    {
        ifaces[iface_index]->handleTxInterrupt(utc_usec);
    }
    else
    {
        assert(0);
    }
}

inline void handleRxInterrupt(uavcan::uint8_t iface_index, uavcan::uint8_t fifo_index)
{
    uavcan::uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();
    if (utc_usec > 0)
    {
        utc_usec--;
    }
    if (iface_index < UAVCAN_STM32_NUM_IFACES &&
        ifaces[iface_index] != NULL)
    {
        ifaces[iface_index]->handleRxInterrupt(fifo_index, utc_usec);
    }
    else
    {
        assert(0);
    }
}

inline void handleStatusInterrupt(uavcan::uint8_t iface_index)
{
    if (iface_index < UAVCAN_STM32_NUM_IFACES &&
        ifaces[iface_index] != NULL)
    {
        ifaces[iface_index]->handleStatusInterrupt();
    }
    else
    {
        assert(0);
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
    if (len_ >= capacity_)
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
    else { assert(0); }
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

    const uavcan::uint32_t pclk = STM32_PCLK1;
    const uavcan::uint32_t prescaler_bs = pclk / target_bitrate;

    // Initial guess:
    uavcan::int8_t bs1 = 10;  // max 15
    uavcan::int8_t bs2 = 5;   // max 8
    uavcan::uint16_t prescaler = 0;

    while (1)
    {
        prescaler = prescaler_bs / (1 + bs1 + bs2);
        // Check result:
        if ((prescaler >= 1) && (prescaler <= 1024))
        {
            const uavcan::uint32_t current_bitrate = pclk / (prescaler * (1 + bs1 + bs2));
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
    out_timings.prescaler = prescaler - 1;
    out_timings.sjw = 1;
    out_timings.bs1 = bs1 - 1;
    out_timings.bs2 = bs2 - 1;

    UAVCAN_STM32_TRACE("CAN pclk=%lu bitrt=%lu presc=%u sjw=%u bs1=%u bs2=%u",
                       static_cast<unsigned long>(pclk), static_cast<unsigned long>(target_bitrate),
                       static_cast<unsigned>(out_timings.prescaler), static_cast<unsigned>(out_timings.sjw),
                       static_cast<unsigned>(out_timings.bs1), static_cast<unsigned>(out_timings.bs2));
    return 0;
}

uavcan::int16_t CanIface::send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                               uavcan::CanIOFlags flags)
{
    if (frame.isErrorFrame() || frame.dlc > 8)
    {
        return -1;  // WTF man how to handle that
    }

    CriticalSectionLock lock;

    /*
     * Seeking for an empty slot
     */
    uavcan::uint8_t txmailbox = 0xFF;
    if ((can_->TSR & CAN_TSR_TME0) == CAN_TSR_TME0)
    {
        txmailbox = 0;
    }
    else if ((can_->TSR & CAN_TSR_TME1) == CAN_TSR_TME1)
    {
        txmailbox = 1;
    }
    else if ((can_->TSR & CAN_TSR_TME2) == CAN_TSR_TME2)
    {
        txmailbox = 2;
    }
    else { return 0; } // No way

    /*
     * Setting up the mailbox
     */
    CAN_TxMailBox_TypeDef& mb = can_->sTxMailBox[txmailbox];
    if (frame.isExtended())
    {
        mb.TIR = ((frame.id & uavcan::CanFrame::MaskExtID) << 3) | CAN_TI0R_IDE;
    }
    else
    {
        mb.TIR = ((frame.id & uavcan::CanFrame::MaskStdID) << 21);
    }

    if (frame.isRemoteTransmissionRequest())
    {
        mb.TIR |= CAN_TI0R_RTR;
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

    mb.TIR |= CAN_TI0R_TXRQ;  // Go.

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
        CriticalSectionLock lock;
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
    CriticalSectionLock lock;
    (void)filter_configs;
    (void)num_configs;
    return -1;
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

    /*
     * Hardware initialization
     */
    can_->MCR &= ~CAN_MCR_SLEEP; // Exit sleep mode
    can_->MCR |= CAN_MCR_INRQ;   // Request init

    for (unsigned wait_ack = 0; wait_ack < 1000000; wait_ack++)
    {
        if ((can_->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)
        {
            break;
        }
    }
    if ((can_->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)
    {
        res = -1;
        goto leave;
    }

    can_->MCR = CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_INRQ;  // RM page 648

    can_->BTR = ((timings.sjw & 3)  << 24) |
                ((timings.bs1 & 15) << 16) |
                ((timings.bs2 & 7)  << 20) |
                (timings.prescaler & 1023);

    can_->IER = CAN_IER_TMEIE |   // TX mailbox empty
                CAN_IER_ERRIE |   // Error set in ESR register
                CAN_IER_LECIE |   // LEC in ESR updated
                CAN_IER_FMPIE0 |  // RX FIFO 0 is not empty
                CAN_IER_FMPIE1;   // RX FIFO 1 is not empty

    can_->MCR &= ~CAN_MCR_INRQ;  // Leave init mode

    for (unsigned wait_ack = 0; wait_ack < 1000000; wait_ack++)
    {
        if ((can_->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)
        {
            break;
        }
    }
    if ((can_->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)
    {
        res = -1;
        goto leave;
    }

    /*
     * Default filter configuration
     */
    if (self_index_ == 0)
    {
        can_->FMR |= CAN_FMR_FINIT;

        can_->FMR &= 0xFFFFC0F1;
        can_->FMR |= static_cast<uavcan::uint32_t>(NumFilters) << 8;  // Slave (CAN2) gets half of the filters

        can_->FFA1R = 0;                           // All assigned to FIFO0 by default
        can_->FM1R = 0;                            // Indentifier Mask mode

#if UAVCAN_STM32_NUM_IFACES > 1
        can_->FS1R = 0x7ffffff;                    // Single 32-bit for all
        can_->sFilterRegister[0].FR1 = 0;          // CAN1 accepts everything
        can_->sFilterRegister[0].FR2 = 0;
        can_->sFilterRegister[NumFilters].FR1 = 0; // CAN2 accepts everything
        can_->sFilterRegister[NumFilters].FR2 = 0;
        can_->FA1R = 1 | (1 << NumFilters);        // One filter per each iface
#else
        can_->FS1R = 0x1fff;
        can_->sFilterRegister[0].FR1 = 0;
        can_->sFilterRegister[0].FR2 = 0;
        can_->FA1R = 1;
#endif

        can_->FMR &= ~CAN_FMR_FINIT;
    }

leave:
    return res;
}

void CanIface::handleTxMailboxInterrupt(uavcan::uint8_t mailbox_index, bool txok, const uavcan::uint64_t utc_usec)
{
    assert(mailbox_index < NumTxMailboxes);
    TxItem& txi = pending_tx_[mailbox_index];
    if (txi.loopback && txok && txi.pending)
    {
        rx_queue_.push(txi.frame, utc_usec, uavcan::CanIOFlagLoopback);
    }
    txi.pending = false;
}

void CanIface::handleTxInterrupt(const uavcan::uint64_t utc_usec)
{
    // TXOK == false means that there was a hardware failure
    if (can_->TSR & CAN_TSR_RQCP0)
    {
        const bool txok = can_->TSR & CAN_TSR_TXOK0;
        can_->TSR = CAN_TSR_RQCP0;
        handleTxMailboxInterrupt(0, txok, utc_usec);
    }
    if (can_->TSR & CAN_TSR_RQCP1)
    {
        const bool txok = can_->TSR & CAN_TSR_TXOK1;
        can_->TSR = CAN_TSR_RQCP1;
        handleTxMailboxInterrupt(1, txok, utc_usec);
    }
    if (can_->TSR & CAN_TSR_RQCP2)
    {
        const bool txok = can_->TSR & CAN_TSR_TXOK2;
        can_->TSR = CAN_TSR_RQCP2;
        handleTxMailboxInterrupt(2, txok, utc_usec);
    }
    update_event_.signalFromInterrupt();
}

void CanIface::handleRxInterrupt(uavcan::uint8_t fifo_index, uavcan::uint64_t utc_usec)
{
    assert(fifo_index < 2);

    volatile uavcan::uint32_t* const rfr_reg = (fifo_index == 0) ? &can_->RF0R : &can_->RF1R;
    if ((*rfr_reg & CAN_RF0R_FMP0) == 0)
    {
        assert(0);  // Weird, IRQ is here but no data to read
        return;
    }

    /*
     * Register overflow as a hardware error
     */
    if ((*rfr_reg & CAN_RF0R_FOVR0) != 0)
    {
        error_cnt_++;
    }

    /*
     * Read the frame contents
     */
    uavcan::CanFrame frame;
    const CAN_FIFOMailBox_TypeDef& rf = can_->sFIFOMailBox[fifo_index];

    if ((rf.RIR & CAN_RI0R_IDE) == 0)
    {
        frame.id = uavcan::CanFrame::MaskStdID & (rf.RIR >> 21);
    }
    else
    {
        frame.id = uavcan::CanFrame::MaskExtID & (rf.RIR >> 3);
        frame.id |= uavcan::CanFrame::FlagEFF;
    }

    if ((rf.RIR & CAN_RI0R_RTR) != 0)
    {
        frame.id |= uavcan::CanFrame::FlagRTR;
    }

    frame.dlc = rf.RDTR & 15;

    frame.data[0] = 0xFF & (rf.RDLR >> 0);
    frame.data[1] = 0xFF & (rf.RDLR >> 8);
    frame.data[2] = 0xFF & (rf.RDLR >> 16);
    frame.data[3] = 0xFF & (rf.RDLR >> 24);
    frame.data[4] = 0xFF & (rf.RDHR >> 0);
    frame.data[5] = 0xFF & (rf.RDHR >> 8);
    frame.data[6] = 0xFF & (rf.RDHR >> 16);
    frame.data[7] = 0xFF & (rf.RDHR >> 24);

    *rfr_reg = CAN_RF0R_RFOM0 | CAN_RF0R_FOVR0 | CAN_RF0R_FULL0;  // Release FIFO entry we just read

    /*
     * Store with timeout into the FIFO buffer and signal update event
     */
    rx_queue_.push(frame, utc_usec, 0);
    update_event_.signalFromInterrupt();
}

void CanIface::handleStatusInterrupt()
{
    last_hw_error_code_ = (can_->ESR & CAN_ESR_LEC) >> 4;
    can_->ESR = 0;
    can_->MSR = CAN_MSR_ERRI | CAN_MSR_WKUI | CAN_MSR_SLAKI;
    error_cnt_++;
}

void CanIface::discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time)
{
    static const uavcan::uint32_t AbortFlags[NumTxMailboxes] = { CAN_TSR_ABRQ0, CAN_TSR_ABRQ1, CAN_TSR_ABRQ2 };
    CriticalSectionLock lock;
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
    return (can_->TSR & CAN_TSR_TME) == 0;
}

uavcan::uint64_t CanIface::getErrorCount() const
{
    CriticalSectionLock lock;
    return error_cnt_ + rx_queue_.getOverflowCount();
}

uavcan::uint8_t CanIface::yieldLastHardwareErrorCode()
{
    CriticalSectionLock lock;
    const uavcan::uint8_t val = last_hw_error_code_;
    last_hw_error_code_ = 0;
    return val;
}

/*
 * CanDriver
 */
uavcan::int16_t CanDriver::select(uavcan::CanSelectMasks& inout_masks, uavcan::MonotonicTime blocking_deadline)
{
    (void)inout_masks;
    (void)blocking_deadline;
    return -1;
}

int CanDriver::init(uavcan::uint32_t bitrate)
{
    int res = 0;

    CriticalSectionLock lock;

    /*
     * CAN1
     */
    RCC->APB1ENR  |=  RCC_APB1ENR_CAN1EN;
    RCC->APB1RSTR |=  RCC_APB1RSTR_CAN1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;

    res = if0_.init(bitrate);
    if (res < 0)
    {
        goto fail;
    }
    ifaces[0] = &if0_;

    /*
     * CAN2
     */
#if UAVCAN_STM32_NUM_IFACES > 1
    RCC->APB1ENR  |=  RCC_APB1ENR_CAN2EN;
    RCC->APB1RSTR |=  RCC_APB1RSTR_CAN2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN2RST;

    res = if1_.init(bitrate);
    if (res < 0)
    {
        goto fail;
    }
    ifaces[1] = &if1_;
#endif

    /*
     * IRQ
     */
    nvicEnableVector(CAN1_TX_IRQn,  UAVCAN_STM32_IRQ_PRIORITY_MASK);
    nvicEnableVector(CAN1_RX0_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
    nvicEnableVector(CAN1_RX1_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
    nvicEnableVector(CAN1_SCE_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
#if UAVCAN_STM32_NUM_IFACES > 1
    nvicEnableVector(CAN2_TX_IRQn,  UAVCAN_STM32_IRQ_PRIORITY_MASK);
    nvicEnableVector(CAN2_RX0_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
    nvicEnableVector(CAN2_RX1_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
    nvicEnableVector(CAN2_SCE_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
#endif

    assert(res >= 0);
    return res;

fail:
    assert(res < 0);

    RCC->APB1ENR &= ~RCC_APB1ENR_CAN1EN;

#if UAVCAN_STM32_NUM_IFACES > 1
    RCC->APB1ENR &= ~RCC_APB1ENR_CAN2EN;
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

} // namespace uavcan_stm32

/*
 * Interrupt handlers
 */
#if !(defined(STM32F10X_CL) || defined(STM32F2XX) || defined(STM32F4XX))

// IRQ numbers
#define CAN1_RX0_IRQn USB_LP_CAN1_RX0_IRQn
#define CAN1_TX_IRQn USB_HP_CAN1_TX_IRQn

// IRQ vectors
#if !defined(CAN1_RX0_IRQHandler) || !defined(CAN1_TX_IRQHandler)
# define CAN1_TX_IRQHandler   USB_HP_CAN1_TX_IRQHandler
# define CAN1_RX0_IRQHandler  USB_LP_CAN1_RX0_IRQHandler
#endif

#endif

extern "C"
{

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

UAVCAN_STM32_IRQ_HANDLER(CAN1_SCE_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    uavcan_stm32::handleStatusInterrupt(0);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

#if UAVCAN_STM32_NUM_IFACES > 1

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

UAVCAN_STM32_IRQ_HANDLER(CAN2_SCE_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    uavcan_stm32::handleStatusInterrupt(1);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

#endif

} // extern "C"
