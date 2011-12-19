/************************************************************************************
 * arch/arm/src/stm32/chip/stm32_can.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_CAN_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_CAN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* 3 TX mailboxes */

#define CAN_TXMBOX1 0
#define CAN_TXMBOX2 1
#define CAN_TXMBOX3 2

/* 2 RX mailboxes */

#define CAN_RXMBOX1 0
#define CAN_RXMBOX2 1

/* Number of filters depends on silicon */

#if defined(CONFIG_STM32_CONNECTIVITYLINE) || defined(CONFIG_STM32_STM32F40XX)
#  define CAN_NFILTERS 28
#else
#  define CAN_NFILTERS 14
#endif

/* Register Offsets *****************************************************************/

/* CAN control and status registers */

#define STM32_CAN_MCR_OFFSET      0x0000  /* CAN master control register */
#define STM32_CAN_MSR_OFFSET      0x0004  /* CAN master status register */
#define STM32_CAN_TSR_OFFSET      0x0008  /* CAN transmit status register */
#define STM32_CAN_RF0R_OFFSET     0x000c  /* CAN receive FIFO 0 register */
#define STM32_CAN_RF1R_OFFSET     0x0010  /* CAN receive FIFO 1 register */
#define STM32_CAN_IER_OFFSET      0x0014  /* CAN interrupt enable register */
#define STM32_CAN_ESR_OFFSET      0x0018  /* CAN error status register */
#define STM32_CAN_BTR_OFFSET      0x001c  /* CAN bit timing register */

/* CAN mailbox registers (3 TX and 2 RX) */

#define STM32_CAN_TIR_OFFSET(m)   (0x0180+0x0010*(m))
#define STM32_CAN_TI0R_OFFSET     0x0180  /* TX mailbox identifier register 0 */
#define STM32_CAN_TI1R_OFFSET     0x0190  /* TX mailbox identifier register 1 */
#define STM32_CAN_TI2R_OFFSET     0x01a0  /* TX mailbox identifier register 2 */

#define STM32_CAN_TDTR_OFFSET(m)  (0x0184+0x0010*(m))
#define STM32_CAN_TDT0R_OFFSET    0x0184  /* Mailbox data length control and time stamp register 0 */
#define STM32_CAN_TDT1R_OFFSET    0x0194  /* Mailbox data length control and time stamp register 1 */
#define STM32_CAN_TDT2R_OFFSET    0x01a4  /* Mailbox data length control and time stamp register 2 */

#define STM32_CAN_TDLR_OFFSET(m)  (0x0188+0x0010*(m))
#define STM32_CAN_TDL0R_OFFSET    0x0188  /* Mailbox data low register 0 */
#define STM32_CAN_TDL1R_OFFSET    0x0198  /* Mailbox data low register 1 */
#define STM32_CAN_TDL2R_OFFSET    0x01a8  /* Mailbox data low register 2 */

#define STM32_CAN_TDHR_OFFSET(m)  (0x018c+0x0010*(m))
#define STM32_CAN_TDH0R_OFFSET    0x018c  /* Mailbox data high register 0 */
#define STM32_CAN_TDH1R_OFFSET    0x019c  /* Mailbox data high register 1 */
#define STM32_CAN_TDH2R_OFFSET    0x01ac  /* Mailbox data high register 2 */

#define STM32_CAN_RIR_OFFSET(m)   (0x01b0+0x0010*(m))
#define STM32_CAN_RI0R_OFFSET     0x01b0  /* Rx FIFO mailbox identifier register 0 */
#define STM32_CAN_RI1R_OFFSET     0x01c0  /* Rx FIFO mailbox identifier register 1 */

#define STM32_CAN_RDTR_OFFSET(m)  (0x01b4+0x0010*(m))
#define STM32_CAN_RDT0R_OFFSET    0x01b4  /* Rx FIFO mailbox data length control and time stamp register 0 */
#define STM32_CAN_RDT1R_OFFSET    0x01c4  /* Rx FIFO mailbox data length control and time stamp register 1 */

#define STM32_CAN_RDLR_OFFSET(m)  (0x01b8+0x0010*(m))
#define STM32_CAN_RDL0R_OFFSET    0x01b8  /* Receive FIFO mailbox data low register 0 */
#define STM32_CAN_RDL1R_OFFSET    0x01c8  /* Receive FIFO mailbox data low register 1 */

#define STM32_CAN_RDHR_OFFSET(m)  (0x01bc+0x0010*(m))
#define STM32_CAN_RDH0R_OFFSET    0x01bc  /* Receive FIFO mailbox data high register 0 */
#define STM32_CAN_RDH1R_OFFSET    0x01cc  /* Receive FIFO mailbox data high register 1 */

/* CAN filter registers */

#define STM32_CAN_FMR_OFFSET      0x0200  /* CAN filter master register */
#define STM32_CAN_FM1R_OFFSET     0x0204  /* CAN filter mode register */
#define STM32_CAN_FS1R_OFFSET     0x020c  /* CAN filter scale register */
#define STM32_CAN_FFA1R_OFFSET    0x0214  /* CAN filter FIFO assignment register */
#define STM32_CAN_FA1R_OFFSET     0x021c  /* CAN filter activation register */

/* There are 14 or 28 filter banks (depending) on the device.  Each filter bank is
 * composed of two 32-bit registers, CAN_FiR:
 */

#define STM32_CAN_FIR_OFFSET(b,i) (0x240+0x0010*(b)*0x004*(i))

/* Register Addresses ***************************************************************/

#if STM32_NCAN > 0
#  define STM32_CAN1_MCR          (STM32_CAN1_BASE+STM32_CAN_MCR_OFFSET)
#  define STM32_CAN1_MSR          (STM32_CAN1_BASE+STM32_CAN_MSR_OFFSET)
#  define STM32_CAN1_TSR          (STM32_CAN1_BASE+STM32_CAN_TSR_OFFSET)
#  define STM32_CAN1_RF0R         (STM32_CAN1_BASE+STM32_CAN_RF0R_OFFSET)
#  define STM32_CAN1_RF1R         (STM32_CAN1_BASE+STM32_CAN_RF1R_OFFSET)
#  define STM32_CAN1_IER          (STM32_CAN1_BASE+STM32_CAN_IER_OFFSET)
#  define STM32_CAN1_ESR          (STM32_CAN1_BASE+STM32_CAN_ESR_OFFSET)
#  define STM32_CAN1_BTR          (STM32_CAN1_BASE+STM32_CAN_BTR_OFFSET)

#  define STM32_CAN1_TIR(m)       (STM32_CAN1_BASE+STM32_CAN_TIR_OFFSET(m))
#  define STM32_CAN1_TI0R         (STM32_CAN1_BASE+STM32_CAN_TI0R_OFFSET)
#  define STM32_CAN1_TI1R         (STM32_CAN1_BASE+STM32_CAN_TI1R_OFFSET)
#  define STM32_CAN1_TI2R         (STM32_CAN1_BASE+STM32_CAN_TI2R_OFFSET)

#  define STM32_CAN1_TDTR(m)      (STM32_CAN1_BASE+STM32_CAN_TDTR_OFFSET(m))
#  define STM32_CAN1_TDT0R        (STM32_CAN1_BASE+STM32_CAN_TDT0R_OFFSET)
#  define STM32_CAN1_TDT1R        (STM32_CAN1_BASE+STM32_CAN_TDT1R_OFFSET)
#  define STM32_CAN1_TDT2R        (STM32_CAN1_BASE+STM32_CAN_TDT2R_OFFSET)

#  define STM32_CAN1_TDLR(m)      (STM32_CAN1_BASE+STM32_CAN_TDLR_OFFSET(m))
#  define STM32_CAN1_TDL0R        (STM32_CAN1_BASE+STM32_CAN_TDL0R_OFFSET)
#  define STM32_CAN1_TDL1R        (STM32_CAN1_BASE+STM32_CAN_TDL1R_OFFSET)
#  define STM32_CAN1_TDL2R        (STM32_CAN1_BASE+STM32_CAN_TDL2R_OFFSET)

#  define STM32_CAN1_TDHR(m)      (STM32_CAN1_BASE+STM32_CAN_TDHR_OFFSET(m))
#  define STM32_CAN1_TDH0R        (STM32_CAN1_BASE+STM32_CAN_TDH0R_OFFSET)
#  define STM32_CAN1_TDH1R        (STM32_CAN1_BASE+STM32_CAN_TDH1R_OFFSET)
#  define STM32_CAN1_TDH2R        (STM32_CAN1_BASE+STM32_CAN_TDH2R_OFFSET)

#  define STM32_CAN1_RIR(m)       (STM32_CAN1_BASE+STM32_CAN_RIR_OFFSET(m))
#  define STM32_CAN1_RI0R         (STM32_CAN1_BASE+STM32_CAN_RI0R_OFFSET)
#  define STM32_CAN1_RI1R         (STM32_CAN1_BASE+STM32_CAN_RI1R_OFFSET)

#  define STM32_CAN1_RDTR(m)      (STM32_CAN1_BASE+STM32_CAN_RDTR_OFFSET(m))
#  define STM32_CAN1_RDT0R        (STM32_CAN1_BASE+STM32_CAN_RDT0R_OFFSET)
#  define STM32_CAN1_RDT1R        (STM32_CAN1_BASE+STM32_CAN_RDT1R_OFFSET)

#  define STM32_CAN1_RDLR(m)      (STM32_CAN1_BASE+STM32_CAN_RDLR_OFFSET(m))
#  define STM32_CAN1_RDL0R        (STM32_CAN1_BASE+STM32_CAN_RDL0R_OFFSET)
#  define STM32_CAN1_RDL1R        (STM32_CAN1_BASE+STM32_CAN_RDL1R_OFFSET)

#  define STM32_CAN1_RDHR(m)      (STM32_CAN1_BASE+STM32_CAN_RDHR_OFFSET(m))
#  define STM32_CAN1_RDH0R        (STM32_CAN1_BASE+STM32_CAN_RDH0R_OFFSET)
#  define STM32_CAN1_RDH1R        (STM32_CAN1_BASE+STM32_CAN_RDH1R_OFFSET)

#  define STM32_CAN1_FMR          (STM32_CAN1_BASE+STM32_CAN_FMR_OFFSET)
#  define STM32_CAN1_FM1R         (STM32_CAN1_BASE+STM32_CAN_FM1R_OFFSET)
#  define STM32_CAN1_FS1R         (STM32_CAN1_BASE+STM32_CAN_FS1R_OFFSET)
#  define STM32_CAN1_FFA1R        (STM32_CAN1_BASE+STM32_CAN_FFA1R_OFFSET)
#  define STM32_CAN1_FA1R         (STM32_CAN1_BASE+STM32_CAN_FA1R_OFFSET)
#  define STM32_CAN1_FIR(b,i)     (STM32_CAN1_BASE+STM32_CAN_FIR_OFFSET(b,i))
#endif

#if STM32_NCAN > 1
#  define STM32_CAN2_MCR          (STM32_CAN2_BASE+STM32_CAN_MCR_OFFSET)
#  define STM32_CAN2_MSR          (STM32_CAN2_BASE+STM32_CAN_MSR_OFFSET)
#  define STM32_CAN2_TSR          (STM32_CAN2_BASE+STM32_CAN_TSR_OFFSET)
#  define STM32_CAN2_RF0R         (STM32_CAN2_BASE+STM32_CAN_RF0R_OFFSET)
#  define STM32_CAN2_RF1R         (STM32_CAN2_BASE+STM32_CAN_RF1R_OFFSET)
#  define STM32_CAN2_IER          (STM32_CAN2_BASE+STM32_CAN_IER_OFFSET)
#  define STM32_CAN2_ESR          (STM32_CAN2_BASE+STM32_CAN_ESR_OFFSET)
#  define STM32_CAN2_BTR          (STM32_CAN2_BASE+STM32_CAN_BTR_OFFSET)

#  define STM32_CAN2_TIR(m)       (STM32_CAN2_BASE+STM32_CAN_TIR_OFFSET(m))
#  define STM32_CAN2_TI0R         (STM32_CAN2_BASE+STM32_CAN_TI0R_OFFSET)
#  define STM32_CAN2_TI1R         (STM32_CAN2_BASE+STM32_CAN_TI1R_OFFSET)
#  define STM32_CAN2_TI2R         (STM32_CAN2_BASE+STM32_CAN_TI2R_OFFSET)

#  define STM32_CAN2_TDTR(m)      (STM32_CAN2_BASE+STM32_CAN_TDTR_OFFSET(m))
#  define STM32_CAN2_TDT0R        (STM32_CAN2_BASE+STM32_CAN_TDT0R_OFFSET)
#  define STM32_CAN2_TDT1R        (STM32_CAN2_BASE+STM32_CAN_TDT1R_OFFSET)
#  define STM32_CAN2_TDT2R        (STM32_CAN2_BASE+STM32_CAN_TDT2R_OFFSET)

#  define STM32_CAN2_TDLR(m)      (STM32_CAN2_BASE+STM32_CAN_TDLR_OFFSET(m))
#  define STM32_CAN2_TDL0R        (STM32_CAN2_BASE+STM32_CAN_TDL0R_OFFSET)
#  define STM32_CAN2_TDL1R        (STM32_CAN2_BASE+STM32_CAN_TDL1R_OFFSET)
#  define STM32_CAN2_TDL2R        (STM32_CAN2_BASE+STM32_CAN_TDL2R_OFFSET)

#  define STM32_CAN2_TDHR(m)      (STM32_CAN2_BASE+STM32_CAN_TDHR_OFFSET(m))
#  define STM32_CAN2_TDH0R        (STM32_CAN2_BASE+STM32_CAN_TDH0R_OFFSET)
#  define STM32_CAN2_TDH1R        (STM32_CAN2_BASE+STM32_CAN_TDH1R_OFFSET)
#  define STM32_CAN2_TDH2R        (STM32_CAN2_BASE+STM32_CAN_TDH2R_OFFSET)

#  define STM32_CAN2_RIR(m)       (STM32_CAN2_BASE+STM32_CAN_RIR_OFFSET(m))
#  define STM32_CAN2_RI0R         (STM32_CAN2_BASE+STM32_CAN_RI0R_OFFSET)
#  define STM32_CAN2_RI1R         (STM32_CAN2_BASE+STM32_CAN_RI1R_OFFSET)

#  define STM32_CAN2_RDTR(m)      (STM32_CAN2_BASE+STM32_CAN_RDTR_OFFSET(m))
#  define STM32_CAN2_RDT0R        (STM32_CAN2_BASE+STM32_CAN_RDT0R_OFFSET)
#  define STM32_CAN2_RDT1R        (STM32_CAN2_BASE+STM32_CAN_RDT1R_OFFSET)

#  define STM32_CAN2_RDLR(m)      (STM32_CAN2_BASE+STM32_CAN_RDLR_OFFSET(m))
#  define STM32_CAN2_RDL0R        (STM32_CAN2_BASE+STM32_CAN_RDL0R_OFFSET)
#  define STM32_CAN2_RDL1R        (STM32_CAN2_BASE+STM32_CAN_RDL1R_OFFSET)

#  define STM32_CAN2_RDHR(m)      (STM32_CAN2_BASE+STM32_CAN_RDHR_OFFSET(m))
#  define STM32_CAN2_RDH0R        (STM32_CAN2_BASE+STM32_CAN_RDH0R_OFFSET)
#  define STM32_CAN2_RDH1R        (STM32_CAN2_BASE+STM32_CAN_RDH1R_OFFSET)

#  define STM32_CAN2_FMR          (STM32_CAN2_BASE+STM32_CAN_FMR_OFFSET)
#  define STM32_CAN2_FM1R         (STM32_CAN2_BASE+STM32_CAN_FM1R_OFFSET)
#  define STM32_CAN2_FS1R         (STM32_CAN2_BASE+STM32_CAN_FS1R_OFFSET)
#  define STM32_CAN2_FFA1R        (STM32_CAN2_BASE+STM32_CAN_FFA1R_OFFSET)
#  define STM32_CAN2_FA1R         (STM32_CAN2_BASE+STM32_CAN_FA1R_OFFSET)
#  define STM32_CAN2_FIR(b,i)     (STM32_CAN2_BASE+STM32_CAN_FIR_OFFSET(b,i))
#endif

/* Register Bitfield Definitions ****************************************************/

/* CAN master control register */

#define CAN_MCR_INRQ              (1 << 0)  /* Bit 0: Initialization Request */
#define CAN_MCR_SLEEP             (1 << 1)  /* Bit 1: Sleep Mode Request */
#define CAN_MCR_TXFP              (1 << 2)  /* Bit 2: Transmit FIFO Priority */
#define CAN_MCR_RFLM              (1 << 3)  /* Bit 3: Receive FIFO Locked Mode */
#define CAN_MCR_NART              (1 << 4)  /* Bit 4: No Automatic Retransmission */
#define CAN_MCR_AWUM              (1 << 5)  /* Bit 5: Automatic Wakeup Mode */
#define CAN_MCR_ABOM              (1 << 6)  /* Bit 6: Automatic Bus-Off Management */
#define CAN_MCR_TTCM              (1 << 7)  /* Bit 7: Time Triggered Communication Mode Enable */
#define CAN_MCR_RESET             (1 << 15) /* Bit 15: bxCAN software master reset */
#define CAN_MCR_DBF               (1 << 16) /* Bit 16: Debug freeze */

/* CAN master status register */

#define CAN_MSR_INAK              (1 << 0)  /* Bit 0: Initialization Acknowledge */
#define CAN_MSR_SLAK              (1 << 1)  /* Bit 1: Sleep Acknowledge */
#define CAN_MSR_ERRI              (1 << 2)  /* Bit 2: Error Interrupt */
#define CAN_MSR_WKUI              (1 << 3)  /* Bit 3: Wakeup Interrupt */
#define CAN_MSR_SLAKI             (1 << 4)  /* Bit 4: Sleep acknowledge interrupt */
#define CAN_MSR_TXM               (1 << 8)  /* Bit 8: Transmit Mode */
#define CAN_MSR_RXM               (1 << 9)  /* Bit 9: Receive Mode */
#define CAN_MSR_SAMP              (1 << 20) /* Bit 10: Last Sample Point */
#define CAN_MSR_RX                (1 << 11) /* Bit 11: CAN Rx Signal */

/* CAN transmit status register */

#define CAN_TSR_RQCP0             (1 << 0)  /* Bit 0: Request Completed Mailbox 0 */
#define CAN_TSR_TXOK0             (1 << 1)  /* Bit 1 : Transmission OK of Mailbox 0 */
#define CAN_TSR_ALST0             (1 << 2)  /* Bit 2 : Arbitration Lost for Mailbox 0 */
#define CAN_TSR_TERR0             (1 << 3)  /* Bit 3 : Transmission Error of Mailbox 0 */
#define CAN_TSR_ABRQ0             (1 << 7)  /* Bit 7 : Abort Request for Mailbox 0 */
#define CAN_TSR_RQCP1             (1 << 8)  /* Bit 8 : Request Completed Mailbox 1 */
#define CAN_TSR_TXOK1             (1 << 9)  /* Bit 9 : Transmission OK of Mailbox 1 */
#define CAN_TSR_ALST1             (1 << 10) /* Bit 10 : Arbitration Lost for Mailbox 1 */
#define CAN_TSR_TERR1             (1 << 11) /* Bit 11 : Transmission Error of Mailbox 1 */
#define CAN_TSR_ABRQ1             (1 << 15) /* Bit 15 : Abort Request for Mailbox 1 */
#define CAN_TSR_RQCP2             (1 << 16) /* Bit 16 : Request Completed Mailbox 2 */
#define CAN_TSR_TXOK2             (1 << 17) /* Bit 17 : Transmission OK of Mailbox 2 */
#define CAN_TSR_ALST2             (1 << 18) /* Bit 18: Arbitration Lost for Mailbox 2 */
#define CAN_TSR_TERR2             (1 << 19) /* Bit 19: Transmission Error of Mailbox 2 */
#define CAN_TSR_ABRQ2             (1 << 23) /* Bit 23: Abort Request for Mailbox 2 */
#define CAN_TSR_CODE_SHIFT        (24)      /* Bits 25-24: Mailbox Code */
#define CAN_TSR_CODE_MASK         (3 << CAN_TSR_CODE_SHIFT)
#define CAN_TSR_TME0              (1 << 26) /* Bit 26: Transmit Mailbox 0 Empty */
#define CAN_TSR_TME1              (1 << 27) /* Bit 27: Transmit Mailbox 1 Empty */
#define CAN_TSR_TME2              (1 << 28) /* Bit 28: Transmit Mailbox 2 Empty */
#define CAN_TSR_LOW0              (1 << 29) /* Bit 29: Lowest Priority Flag for Mailbox 0 */
#define CAN_TSR_LOW1              (1 << 30) /* Bit 30: Lowest Priority Flag for Mailbox 1 */
#define CAN_TSR_LOW2              (1 << 31) /* Bit 31: Lowest Priority Flag for Mailbox 2 */

/* CAN receive FIFO 0/1 registers */

#define CAN_RFR_FMP_SHIFT         (0)       /* Bits 1-0: FIFO Message Pending */
#define CAN_RFR_FMP_MASK          (3 << CAN_RFR_FMP_SHIFT)
#define CAN_RFR_FULL              (1 << 3)  /* Bit 3: FIFO 0 Full */
#define CAN_RFR_FOVR              (1 << 4)  /* Bit 4: FIFO 0 Overrun */
#define CAN_RFR_RFOM              (1 << 5)  /* Bit 5: Release FIFO 0 Output Mailbox */

/* CAN interrupt enable register */

#define CAN_IER_TMEIE             (1 << 0)  /* Bit 0: Transmit Mailbox Empty Interrupt Enable */
#define CAN_IER_FMPIE0            (1 << 1)  /* Bit 1: FIFO Message Pending Interrupt Enable */
#define CAN_IER_FFIE0             (1 << 2)  /* Bit 2: FIFO Full Interrupt Enable */
#define CAN_IER_FOVIE0            (1 << 3)  /* Bit 3: FIFO Overrun Interrupt Enable */
#define CAN_IER_FMPIE1            (1 << 4)  /* Bit 4: FIFO Message Pending Interrupt Enable */
#define CAN_IER_FFIE1             (1 << 5)  /* Bit 5: FIFO Full Interrupt Enable */
#define CAN_IER_FOVIE1            (1 << 6)  /* Bit 6: FIFO Overrun Interrupt Enable */
#define CAN_IER_EWGIE             (1 << 8)  /* Bit 8: Error Warning Interrupt Enable */
#define CAN_IER_EPVIE             (1 << 9)  /* Bit 9: Error Passive Interrupt Enable */
#define CAN_IER_BOFIE             (1 << 10) /* Bit 10: Bus-Off Interrupt Enable */
#define CAN_IER_LECIE             (1 << 11) /* Bit 11: Last Error Code Interrupt Enable */
#define CAN_IER_ERRIE             (1 << 15) /* Bit 15: Error Interrupt Enable */
#define CAN_IER_WKUIE             (1 << 16) /* Bit 16: Wakeup Interrupt Enable */
#define CAN_IER_SLKIE             (1 << 17) /* Bit 17: Sleep Interrupt Enable */

/* CAN error status register */

#define CAN_ESR_EWGF              (1 << 0)  /* Bit 0: Error Warning Flag */
#define CAN_ESR_EPVF              (1 << 1)  /* Bit 1: Error Passive Flag */
#define CAN_ESR_BOFF              (1 << 2)  /* Bit 2: Bus-Off Flag */
#define CAN_ESR_LEC_SHIFT         (4)       /* Bits 6-4: Last Error Code */
#define CAN_ESR_LEC_MASK          (7 << CAN_ESR_LEC_SHIFT)
#  define CAN_ESR_NOERROR         (0 << CAN_ESR_LEC_SHIFT) /* 000: No Error */
#  define CAN_ESR_STUFFERROR      (1 << CAN_ESR_LEC_SHIFT) /* 001: Stuff Error */
#  define CAN_ESR_FORMERROR       (2 << CAN_ESR_LEC_SHIFT) /* 010: Form Error */
#  define CAN_ESR_ACKERROR        (3 << CAN_ESR_LEC_SHIFT) /* 011: Acknowledgment Error */
#  define CAN_ESR_BRECERROR       (4 << CAN_ESR_LEC_SHIFT) /* 100: Bit recessive Error */
#  define CAN_ESR_BDOMERROR       (5 << CAN_ESR_LEC_SHIFT) /* 101: Bit dominant Error */
#  define CAN_ESR_CRCERRPR        (6 << CAN_ESR_LEC_SHIFT) /* 110: CRC Error */
#  define CAN_ESR_SWERROR         (7 << CAN_ESR_LEC_SHIFT) /* 111: Set by software */
#define CAN_ESR_TEC_SHIFT         (16)      /* Bits 23-16: LS byte of the 9-bit Transmit Error Counter */
#define CAN_ESR_TEC_MASK          (0xff << CAN_ESR_TEC_SHIF)
#define CAN_ESR_REC_SHIFT         (24)      /* Bits 31-24: Receive Error Counter */
#define CAN_ESR_REC_MASK          (0xff << CAN_ESR_REC_SHIFT)

/* CAN bit timing register */

#define CAN_BTR_BRP_SHIFT         (0)       /* Bits 9-0: Baud Rate Prescaler */
#define CAN_BTR_BRP_MASK          (0x03ff << CAN_BTR_BRP_SHIFT)
#define CAN_BTR_TS1_SHIFT         (16)      /* Bits 19-16: Time Segment 1 */
#define CAN_BTR_TS1_MASK          (0x0f <<  CAN_BTR_TS1_SHIFT)
#define CAN_BTR_TS2_SHIFT         (20)      /* Bits 22-20: Time Segment 2 */
#define CAN_BTR_TS2_MASK          (7 << CAN_BTR_TS2_SHIFT)
#define CAN_BTR_SJW_SHIFT         (24)      /* Bits 25-24: Resynchronization Jump Width */
#define CAN_BTR_SJW_MASK          (3 << CAN_BTR_SJW_SHIFT)
#define CAN_BTR_LBKM              (1 << 30) /* Bit 30: Loop Back Mode (Debug) */
#define CAN_BTR_SILM              (1 << 31) /* Bit 31: Silent Mode (Debug) */

/* TX mailbox identifier register */

#define CAN_TIR_TXRQ              (1 << 0)  /* Bit 0: Transmit Mailbox Request */
#define CAN_TIR_RTR               (1 << 1)  /* Bit 1: Remote Transmission Request */
#define CAN_TIR_IDE               (1 << 2)  /* Bit 2: Identifier Extension */
#define CAN_TIR_EXID_SHIFT        (3)       /* Bit 3-20: Extended Identifier */
#define CAN_TIR_EXID_MASK         (0x0003ffff << CAN_TIR_EXID_SHIFT)
#define CAN_TIR_STID_SHIFT        (21)      /* Bits 21-31: Standard Identifier */
#define CAN_TIR_STID_MASK         (0x07ff << CAN_TIR_STID_SHIFT)

/* Mailbox data length control and time stamp register */

#define CAN_TDTR_DLC_SHIFT        (0)       /* Bits 3:0: Data Length Code */
#define CAN_TDTR_DLC_MASK         (0x0f << CAN_TDTR_DLC_SHIFT)
#define CAN_TDTR_TGT              (1 << 8)  /* Bit 8: Transmit Global Time */
#define CAN_TDTR_TIME_SHIFT       (16)      /* Bits 31:16: Message Time Stamp */
#define CAN_TDTR_TIME_MASK        (0xffff << CAN_TDTR_TIME_SHIFT)

/* Mailbox data low register */

#define CAN_TDLR_DATA0_SHIFT      (0)       /* Bits 7-0: Data Byte 0 */
#define CAN_TDLR_DATA0_MASK       (0xff << CAN_TDLR_DATA0_SHIFT)
#define CAN_TDLR_DATA1_SHIFT      (8)       /* Bits 15-8: Data Byte 1 */
#define CAN_TDLR_DATA1_MASK       (0xff << CAN_TDLR_DATA1_SHIFT)
#define CAN_TDLR_DATA2_SHIFT      (16)      /* Bits 23-16: Data Byte 2 */
#define CAN_TDLR_DATA2_MASK       (0xff << CAN_TDLR_DATA2_SHIFT)
#define CAN_TDLR_DATA3_SHIFT      (24)      /* Bits 31-24: Data Byte 3 */
#define CAN_TDLR_DATA3_MASK       (0xff << CAN_TDLR_DATA3_SHIFT)

/* Mailbox data high register */

#define CAN_TDHR_DATA4_SHIFT      (0)       /* Bits 7-0: Data Byte 4 */
#define CAN_TDHR_DATA4_MASK       (0xff << CAN_TDHR_DATA4_SHIFT)
#define CAN_TDHR_DATA5_SHIFT      (8)       /* Bits 15-8: Data Byte 5 */
#define CAN_TDHR_DATA5_MASK       (0xff << CAN_TDHR_DATA5_SHIFT)
#define CAN_TDHR_DATA6_SHIFT      (16)      /* Bits 23-16: Data Byte 6 */
#define CAN_TDHR_DATA6_MASK       (0xff << CAN_TDHR_DATA6_SHIFT)
#define CAN_TDHR_DATA7_SHIFT      (24)      /* Bits 31-24: Data Byte 7 */
#define CAN_TDHR_DATA7_MASK       (0xff << CAN_TDHR_DATA7_SHIFT)

/* Rx FIFO mailbox identifier register */

#define CAN_RIR_RTR               (1 << 1)  /* Bit 1: Remote Transmission Request */
#define CAN_RIR_IDE               (1 << 2)  /* Bit 2: Identifier Extension */
#define CAN_RIR_EXID_SHIFT        (3)       /* Bit 3-20: Extended Identifier */
#define CAN_RIR_EXID_MASK         (0x0003ffff << CAN_RIR_EXID_SHIFT)
#define CAN_RIR_STID_SHIFT        (21)      /* Bits 21-31: Standard Identifier */
#define CAN_RIR_STID_MASK         (0x07ff << CAN_RIR_STID_SHIFT)

/* Receive FIFO mailbox data length control and time stamp register */

#define CAN_RDTR_DLC_SHIFT        (0)       /* Bits 3:0: Data Length Code */
#define CAN_RDTR_DLC_MASK         (0x0f << CAN_RDTR_DLC_SHIFT)
#define CAN_RDTR_FM_SHIFT         (8)       /* Bits 15-8: Filter Match Index */
#define CAN_RDTR_FM_MASK          (0xff << CAN_RDTR_FM_SHIFT)
#define CAN_RDTR_TIME_SHIFT       (16)      /* Bits 31:16: Message Time Stamp */
#define CAN_RDTR_TIME_MASK        (0xffff << CAN_RDTR_TIME_SHIFT)

/* Receive FIFO mailbox data low register */

#define CAN_RDLR_DATA0_SHIFT      (0)       /* Bits 7-0: Data Byte 0 */
#define CAN_RDLR_DATA0_MASK       (0xff << CAN_RDLR_DATA0_SHIFT)
#define CAN_RDLR_DATA1_SHIFT      (8)       /* Bits 15-8: Data Byte 1 */
#define CAN_RDLR_DATA1_MASK       (0xff << CAN_RDLR_DATA1_SHIFT)
#define CAN_RDLR_DATA2_SHIFT      (16)      /* Bits 23-16: Data Byte 2 */
#define CAN_RDLR_DATA2_MASK       (0xff << CAN_RDLR_DATA2_SHIFT)
#define CAN_RDLR_DATA3_SHIFT      (24)      /* Bits 31-24: Data Byte 3 */
#define CAN_RDLR_DATA3_MASK       (0xff << CAN_RDLR_DATA3_SHIFT)

/* Receive FIFO mailbox data high register */

#define CAN_RDHR_DATA4_SHIFT      (0)       /* Bits 7-0: Data Byte 4 */
#define CAN_RDHR_DATA4_MASK       (0xff << CAN_RDHR_DATA4_SHIFT)
#define CAN_RDHR_DATA5_SHIFT      (8)       /* Bits 15-8: Data Byte 5 */
#define CAN_RDHR_DATA5_MASK       (0xff << CAN_RDHR_DATA5_SHIFT)
#define CAN_RDHR_DATA6_SHIFT      (16)      /* Bits 23-16: Data Byte 6 */
#define CAN_RDHR_DATA6_MASK       (0xff << CAN_RDHR_DATA6_SHIFT)
#define CAN_RDHR_DATA7_SHIFT      (24)      /* Bits 31-24: Data Byte 7 */
#define CAN_RDHR_DATA7_MASK       (0xff << CAN_RDHR_DATA7_SHIFT)

/* CAN filter master register */

#define CAN_FMR_FINIT             (1 << 0)  /* Bit 0:  Filter Init Mode */
#if defined(CONFIG_STM32_CONNECTIVITYLINE) || defined(CONFIG_STM32_STM32F40XX)
#  define CAN_FMR_CAN2SB_SHIFT    (8)       /* Bits 13-8: CAN2 start bank */
#  define CAN_FMR_CAN2SB_MASK     (0x3f << CAN_FMR_CAN2SB_SHIFT)
#endif

/* CAN filter mode register */

#if defined(CONFIG_STM32_CONNECTIVITYLINE) || defined(CONFIG_STM32_STM32F40XX)
#  define CAN_FM1R_FBM_SHIFT      (0)      /* Bits 13:0: Filter Mode */
#  define CAN_FM1R_FBM_MASK       (0x3fff << CAN_FM1R_FBM_SHIFT)
#else
#  define CAN_FM1R_FBM_SHIFT      (0)      /* Bits 27:0: Filter Mode */
#  define CAN_FM1R_FBM_MASK       (0x0fffffff << CAN_FM1R_FBM_SHIFT)
#endif

/* CAN filter scale register */

#if defined(CONFIG_STM32_CONNECTIVITYLINE) || defined(CONFIG_STM32_STM32F40XX)
#  define CAN_FS1R_FSC_SHIFT      (0)      /* Bits 13:0: Filter Scale Configuration */
#  define CAN_FS1R_FSC_MASK       (0x3fff << CAN_FS1R_FSC_SHIFT)
#else
#  define CAN_FS1R_FSC_SHIFT      (0)      /* Bits 27:0: Filter Scale Configuration */
#  define CAN_FS1R_FSC_MASK       (0x0fffffff << CAN_FS1R_FSC_SHIFT)
#endif

/* CAN filter FIFO assignment register */

#if defined(CONFIG_STM32_CONNECTIVITYLINE) || defined(CONFIG_STM32_STM32F40XX)
#  define CAN_FFA1R_FFA_SHIFT     (0)      /* Bits 13:0: Filter FIFO Assignment */
#  define CAN_FFA1R_FFA_MASK      (0x3fff << CAN_FFA1R_FFA_SHIFT)
#else
#  define CAN_FFA1R_FFA_SHIFT     (0)      /* Bits 27:0: Filter FIFO Assignment */
#  define CAN_FFA1R_FFA_MASK      (0x0fffffff << CAN_FFA1R_FFA_SHIFT)
#endif

/* CAN filter activation register */

#if defined(CONFIG_STM32_CONNECTIVITYLINE) || defined(CONFIG_STM32_STM32F40XX)
#  define CAN_FA1R_FACT_SHIFT     (0)      /* Bits 13:0: Filter Active */
#  define CAN_FA1R_FACT_MASK      (0x3fff << CAN_FA1R_FACT_SHIFT)
#else
#  define CAN_FA1R_FACT_SHIFT     (0)      /* Bits 27:0: Filter Active */
#  define CAN_FA1R_FACT_MASK      (0x0fffffff << CAN_FA1R_FACT_SHIFT)
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_CAN_H */
