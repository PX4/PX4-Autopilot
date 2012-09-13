/****************************************************************************************************
 * arch/arm/src/kinetis/kinetis_flexcan.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_FLEXCAN_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_FLEXCAN_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

#define KINETIS_CAN_MCR_OFFSET      0x0000 /* Module Configuration Register */
#define KINETIS_CAN_CTRL1_OFFSET    0x0004 /* Control 1 Register */
#define KINETIS_CAN_TIMER_OFFSET    0x0008 /* Free Running Timer */
#define KINETIS_CAN_RXMGMASK_OFFSET 0x0010 /* Rx Mailboxes Global Mask Register */
#define KINETIS_CAN_RX14MASK_OFFSET 0x0014 /* Rx 14 Mask Register */
#define KINETIS_CAN_RX15MASK_OFFSET 0x0018 /* Rx 15 Mask Register */
#define KINETIS_CAN_ECR_OFFSET      0x001c /* Error Counter */
#define KINETIS_CAN_ESR1_OFFSET     0x0020 /* Error and Status 1 Register */
#define KINETIS_CAN_IMASK2_OFFSET   0x0024 /* Interrupt Masks 2 Register */
#define KINETIS_CAN_IMASK1_OFFSET   0x0028 /* Interrupt Masks 1 Register */
#define KINETIS_CAN_IFLAG2_OFFSET   0x002c /* Interrupt Flags 2 Register */
#define KINETIS_CAN_IFLAG1_OFFSET   0x0030 /* Interrupt Flags 1 Register */
#define KINETIS_CAN_CTRL2_OFFSET    0x0034 /* Control 2 Register */
#define KINETIS_CAN_ESR2_OFFSET     0x0038 /* Error and Status 2 Register */
#define KINETIS_CAN_CRCR_OFFSET     0x0044 /* CRC Register */
#define KINETIS_CAN_RXFGMASK_OFFSET 0x0048 /* Rx FIFO Global Mask Register */
#define KINETIS_CAN_RXFIR_OFFSET    0x004c /* Rx FIFO Information Register */

#define KINETIS_CAN_RXIMR_OFFSET(n) (0x0880+((n)<<2)) /* Rn Individual Mask Registers */
#define KINETIS_CAN_RXIMR0_OFFSET   0x0880 /* R0 Individual Mask Registers */
#define KINETIS_CAN_RXIMR1_OFFSET   0x0884 /* R1 Individual Mask Registers */
#define KINETIS_CAN_RXIMR2_OFFSET   0x0888 /* R2 Individual Mask Registers */
#define KINETIS_CAN_RXIMR3_OFFSET   0x088c /* R3 Individual Mask Registers */
#define KINETIS_CAN_RXIMR4_OFFSET   0x0890 /* R4 Individual Mask Registers */
#define KINETIS_CAN_RXIMR5_OFFSET   0x0894 /* R5 Individual Mask Registers */
#define KINETIS_CAN_RXIMR6_OFFSET   0x0898 /* R6 Individual Mask Registers */
#define KINETIS_CAN_RXIMR7_OFFSET   0x089c /* R7 Individual Mask Registers */
#define KINETIS_CAN_RXIMR8_OFFSET   0x08a0 /* R8 Individual Mask Registers */
#define KINETIS_CAN_RXIMR9_OFFSET   0x08a4 /* R9 Individual Mask Registers */
#define KINETIS_CAN_RXIMR10_OFFSET  0x08a8 /* R10 Individual Mask Registers */
#define KINETIS_CAN_RXIMR11_OFFSET  0x08ac /* R11 Individual Mask Registers */
#define KINETIS_CAN_RXIMR12_OFFSET  0x08b0 /* R12 Individual Mask Registers */
#define KINETIS_CAN_RXIMR13_OFFSET  0x08b4 /* R13 Individual Mask Registers */
#define KINETIS_CAN_RXIMR14_OFFSET  0x08b8 /* R14 Individual Mask Registers */
#define KINETIS_CAN_RXIMR15_OFFSET  0x08bc /* R15 Individual Mask Registers */

/* Register Addresses *******************************************************************************/

#define KINETIS_CAN0_MCR           (KINETIS_CAN0_BASE+KINETIS_CAN_MCR_OFFSET)
#define KINETIS_CAN0_CTRL1         (KINETIS_CAN0_BASE+KINETIS_CAN_CTRL1_OFFSET)
#define KINETIS_CAN0_TIMER         (KINETIS_CAN0_BASE+KINETIS_CAN_TIMER_OFFSET)
#define KINETIS_CAN0_RXMGMASK      (KINETIS_CAN0_BASE+KINETIS_CAN_RXMGMASK_OFFSET)
#define KINETIS_CAN0_RX14MASK      (KINETIS_CAN0_BASE+KINETIS_CAN_RX14MASK_OFFSET)
#define KINETIS_CAN0_RX15MASK      (KINETIS_CAN0_BASE+KINETIS_CAN_RX15MASK_OFFSET)
#define KINETIS_CAN0_ECR           (KINETIS_CAN0_BASE+KINETIS_CAN_ECR_OFFSET)
#define KINETIS_CAN0_ESR1          (KINETIS_CAN0_BASE+KINETIS_CAN_ESR1_OFFSET)
#define KINETIS_CAN0_IMASK2        (KINETIS_CAN0_BASE+KINETIS_CAN_IMASK2_OFFSET)
#define KINETIS_CAN0_IMASK1        (KINETIS_CAN0_BASE+KINETIS_CAN_IMASK1_OFFSET)
#define KINETIS_CAN0_IFLAG2        (KINETIS_CAN0_BASE+KINETIS_CAN_IFLAG2_OFFSET)
#define KINETIS_CAN0_IFLAG1        (KINETIS_CAN0_BASE+KINETIS_CAN_IFLAG1_OFFSET)
#define KINETIS_CAN0_CTRL2         (KINETIS_CAN0_BASE+KINETIS_CAN_CTRL2_OFFSET)
#define KINETIS_CAN0_ESR2          (KINETIS_CAN0_BASE+KINETIS_CAN_ESR2_OFFSET)
#define KINETIS_CAN0_CRCR          (KINETIS_CAN0_BASE+KINETIS_CAN_CRCR_OFFSET)
#define KINETIS_CAN0_RXFGMASK      (KINETIS_CAN0_BASE+KINETIS_CAN_RXFGMASK_OFFSET)
#define KINETIS_CAN0_RXFIR         (KINETIS_CAN0_BASE+KINETIS_CAN_RXFIR_OFFSET)

#define KINETIS_CAN0_RXIMR(n)      (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR_OFFSET(n))
#define KINETIS_CAN0_RXIMR0        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR0_OFFSET)
#define KINETIS_CAN0_RXIMR1        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR1_OFFSET)
#define KINETIS_CAN0_RXIMR2        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR2_OFFSET)
#define KINETIS_CAN0_RXIMR3        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR3_OFFSET)
#define KINETIS_CAN0_RXIMR4        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR4_OFFSET)
#define KINETIS_CAN0_RXIMR5        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR5_OFFSET)
#define KINETIS_CAN0_RXIMR6        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR6_OFFSET)
#define KINETIS_CAN0_RXIMR7        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR7_OFFSET)
#define KINETIS_CAN0_RXIMR8        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR8_OFFSET)
#define KINETIS_CAN0_RXIMR9        (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR9_OFFSET)
#define KINETIS_CAN0_RXIMR10       (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR10_OFFSET)
#define KINETIS_CAN0_RXIMR11       (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR11_OFFSET)
#define KINETIS_CAN0_RXIMR12       (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR12_OFFSET)
#define KINETIS_CAN0_RXIMR13       (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR13_OFFSET)
#define KINETIS_CAN0_RXIMR14       (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR14_OFFSET)
#define KINETIS_CAN0_RXIMR15       (KINETIS_CAN0_BASE+KINETIS_CAN_RXIMR15_OFFSET)

/* Register Bit Definitions *************************************************************************/

/* Module Configuration Register */

#define CAN_MCR_MAXMB_SHIFT        (0)       /* Bits 0-6: Number of the Last Message Buffer */
#define CAN_MCR_MAXMB_MASK         (0x7f << CAN_MCR_MAXMB_SHIFT)
                                             /* Bit 7:  Reserved */
#define CAN_MCR_IDAM_SHIFT         (8)       /* Bits 8-9: ID Acceptance Mode */
#define CAN_MCR_IDAM_MASK          (3 << CAN_MCR_IDAM_SHIFT)
#  define CAN_MCR_IDAM_FMTA        (0 << CAN_MCR_IDAM_SHIFT) /* Format A: One full ID  */
#  define CAN_MCR_IDAM_FMTB        (1 << CAN_MCR_IDAM_SHIFT) /* Format B: Two full (or partial) IDs */
#  define CAN_MCR_IDAM_FMTC        (2 << CAN_MCR_IDAM_SHIFT) /* Format C: Four partial IDs */
#  define CAN_MCR_IDAM_FMTD        (3 << CAN_MCR_IDAM_SHIFT) /* Format D: All frames rejected */
                                             /* Bits 10-11: Reserved */
#define CAN_MCR_AEN                (1 << 12) /* Bit 12: Abort Enable */
#define CAN_MCR_LPRIOEN            (1 << 13) /* Bit 13: Local Priority Enable */
                                             /* Bits 14-15: Reserved */
#define CAN_MCR_IRMQ               (1 << 16) /* Bit 16: Individual Rx Masking and Queue Enable */
#define CAN_MCR_SRXDIS             (1 << 17) /* Bit 17: Self Reception Disable */
#define CAN_MCR_DOZE               (1 << 18) /* Bit 18: Doze Mode Enable */
                                             /* Bit 19: Reserved */
#define CAN_MCR_LPMACK             (1 << 20) /* Bit 20: Low Power Mode Acknowledge */
#define CAN_MCR_WRNEN              (1 << 21) /* Bit 21: Warning Interrupt Enable */
#define CAN_MCR_SLFWAK             (1 << 22) /* Bit 22: Self Wake Up */
#define CAN_MCR_SUPV               (1 << 23) /* Bit 23: Supervisor Mode */
#define CAN_MCR_FRZACK             (1 << 24) /* Bit 24: Freeze Mode Acknowledge */
#define CAN_MCR_SOFTRST            (1 << 25) /* Bit 25: Soft Reset */
#define CAN_MCR_WAKMSK             (1 << 26) /* Bit 26: Wake Up Interrupt Mask */
#define CAN_MCR_NOTRDY             (1 << 27) /* Bit 27: FlexCAN Not Ready */
#define CAN_MCR_HALT               (1 << 28) /* Bit 28: Halt FlexCAN */
#define CAN_MCR_RFEN               (1 << 29) /* Bit 29: Rx FIFO Enable */
#define CAN_MCR_FRZ                (1 << 30) /* Bit 30: Freeze Enable */
#define CAN_MCR_MDIS               (1 << 31) /* Bit 31: Module Disable */

/* Control 1 Register */

#define CAN_CTRL1_ROPSEG_SHIFT     (0)       /* Bits 0-2: Propagation Segment */
#define CAN_CTRL1_ROPSEG_MASK      (7 << CAN_CTRL1_ROPSEG_SHIFT)
#define CAN_CTRL1_LOM              (1 << 3)  /* Bit 3:  Listen-Only Mode */
#define CAN_CTRL1_LBUF             (1 << 4)  /* Bit 4:  Lowest Buffer Transmitted First */
#define CAN_CTRL1_TSYN             (1 << 5)  /* Bit 5:  Timer Sync */
#define CAN_CTRL1_BOFFREC          (1 << 6)  /* Bit 6:  Bus Off Recovery */
#define CAN_CTRL1_SMP              (1 << 7)  /* Bit 7:  CAN Bit Sampling */
                                             /* Bits 8-9: Reserved */
#define CAN_CTRL1_RWRNMSK          (1 << 10) /* Bit 10: Rx Warning Interrupt Mask */
#define CAN_CTRL1_TWRNMSK          (1 << 11) /* Bit 11: Tx Warning Interrupt Mask */
#define CAN_CTRL1_LPB              (1 << 12) /* Bit 12: Loop Back Mode */
#define CAN_CTRL1_CLKSRC           (1 << 13) /* Bit 13: CAN Engine Clock Source */
#define CAN_CTRL1_ERRMSK           (1 << 14) /* Bit 14: Error Mask */
#define CAN_CTRL1_BOFFMSK          (1 << 15) /* Bit 15: Bus Off Mask */
#define CAN_CTRL1_PSEG2_SHIFT      (16)       /* Bits 16-18: Phase Segment 2 */
#define CAN_CTRL1_PSEG2_MASK       (7 << CAN_CTRL1_PSEG2_SHIFT)
#define CAN_CTRL1_PSEG1_SHIFT      (19)       /* Bits 19-21: Phase Segment 1 */
#define CAN_CTRL1_PSEG1_MASK       (7 << CAN_CTRL1_PSEG1_SHIFT)
#define CAN_CTRL1_RJW_SHIFT        (22)       /* Bits 22-23: Resync Jump Width */
#define CAN_CTRL1_RJW_MASK         (3 << CAN_CTRL1_RJW_SHIFT)
#define CAN_CTRL1_PRESDIV_SHIFT    (24)       /* Bits 24-31: Prescaler Division Factor */
#define CAN_CTRL1_PRESDIV_MASK     (0xff << CAN_CTRL1_PRESDIV_SHIFT)

/* Free Running Timer */

#define CAN_TIMER_SHIFT            (0)       /* Bits 0-15: Timer value */
#define CAN_TIMER_MASK             (0xffff << CAN_TIMER_SHIFT)
                                             /* Bits 16-31: Reserved */
/* Rx Mailboxes Global Mask Register (32 Rx Mailboxes Global Mask Bits) */

#define CAN_RXMGMASK(n)            (1 << (n)) /* Bit n: Rx Mailboxe n Global Mask Bit */

/* Rx 14 Mask Register */

#define CAN_RX14MASK(n)            (1 << (n)) /* Bit n: Rx Buffer 14 Mask Bit n */

/* Rx 15 Mask Register */

#define CAN_RX15MASK(n)            (1 << (n)) /* Bit n: Rx Buffer 15 Mask Bit n */

/* Error Counter */

#define CAN_ECR_TXERRCNT_SHIFT     (0)       /* Bits 0-7: Transmit Error Counter */
#define CAN_ECR_TXERRCNT_MASK      (0xff << CAN_ECR_TXERRCNT_SHIFT)
#define CAN_ECR_RXERRCNT_SHIFT     (8)       /* Bits 8-15: Receive Error Counter */
#define CAN_ECR_RXERRCNT_MASK      (0xff << CAN_ECR_RXERRCNT_SHIFT)
                                             /* Bits 16-31: Reserved */
/* Error and Status 1 Register */

#define CAN_ESR1_WAKINT            (1 << 0)  /* Bit 0:  Wake-Up Interrupt */
#define CAN_ESR1_ERRINT            (1 << 1)  /* Bit 1:  Error Interrupt */
#define CAN_ESR1_BOFFINT           (1 << 2)  /* Bit 2:  'Bus Off' Interrupt */
#define CAN_ESR1_RX                (1 << 3)  /* Bit 3:  FlexCAN in Reception */
#define CAN_ESR1_FLTCONF_SHIFT     (4)       /* Bits 4-5: Fault Confinement State */
#define CAN_ESR1_FLTCONF_MASK      (3 << CAN_ESR1_FLTCONF_SHIFT)
#  define CAN_ESR1_FLTCONF_ACTV    (0 << CAN_ESR1_FLTCONF_SHIFT) /* Error Active */
#  define CAN_ESR1_FLTCONF_PASV    (1 << CAN_ESR1_FLTCONF_SHIFT) /* Error Passive */
#  define CAN_ESR1_FLTCONF_OFF     (2 << CAN_ESR1_FLTCONF_SHIFT) /* Bus Off */
#define CAN_ESR1_TX                (1 << 6)  /* Bit 6:  FlexCAN in Transmission */
#define CAN_ESR1_IDLE              (1 << 7)  /* Bit 7:  CAN bus is in IDLE state */
#define CAN_ESR1_RXWRN             (1 << 8)  /* Bit 8:  Rx Error Warning */
#define CAN_ESR1_TXWRN             (1 << 9)  /* Bit 9:  TX Error Warning */
#define CAN_ESR1_STFERR            (1 << 10) /* Bit 10: Stuffing Error */
#define CAN_ESR1_FRMERR            (1 << 11) /* Bit 11: Form Error */
#define CAN_ESR1_CRCERR            (1 << 12) /* Bit 12: Cyclic Redundancy Check Error */
#define CAN_ESR1_ACKERR            (1 << 13) /* Bit 13: Acknowledge Error */
#define CAN_ESR1_BIT0ERR           (1 << 14) /* Bit 14: Bit0 Error */
#define CAN_ESR1_BIT1ERR           (1 << 15) /* Bit 15: Bit1 Error */
#define CAN_ESR1_RWRNINT           (1 << 16) /* Bit 16: Rx Warning Interrupt Flag */
#define CAN_ESR1_TWRNINT           (1 << 17) /* Bit 17: Tx Warning Interrupt Flag */
#define CAN_ESR1_SYNCH             (1 << 18) /* Bit 18: CAN Synchronization Status */
                                             /* Bits 19-31: Reserved */
/* Interrupt Masks 2 Register */

#define CAN_IMASK2(n)              (1 << (n)) /* Bit n: Buffer MBn Mask */

/* Interrupt Masks 1 Register */

#define CAN_IMASK1(n)              (1 << (n)) /* Bit n: Buffer MBn Mask */

/* Interrupt Flags 2 Register */

#define CAN_IFLAG2(n)              (1 << (n)) /* Bit n: Buffer MBn Interrupt */

/* Interrupt Flags 1 Register */

#define CAN_IFLAG1(n)              (1 << (n)) /* Bit n: Buffer MBn Interrupt, n=0..4,8..31 */

/* Control 2 Register */
                                             /* Bits 0-15: Reserved */
#define CAN_CTRL2_EACEN            (1 << 16) /* Bit 16:  Entire Frame Arbitration Field Comparison Enable (Rx) */
#define CAN_CTRL2_RRS              (1 << 17) /* Bit 17:  Remote Request Storing */
#define CAN_CTRL2_MRP              (1 << 18) /* Bit 18:  Mailboxes Reception Priority */
#define CAN_CTRL2_TASD_SHIFT       (19)      /* Bits 19-23: Tx Arbitration Start Delay */
#define CAN_CTRL2_TASD_MASK        (31 << CAN_CTRL2_TASD_SHIFT)
#define CAN_CTRL2_RFFN_SHIFT       (24)      /* Bits 24-27: Number of Rx FIFO Filters */
#define CAN_CTRL2_RFFN_MASK        (15 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_8MB       (0 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_16MB      (1 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_24MB      (2 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_32MB      (3 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_40MB      (4 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_48MB      (5 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_56MB      (6 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_64MB      (7 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_72MB      (8 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_80MB      (9 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_88MB      (10 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_96MB      (11 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_104MB     (12 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_112MB     (13 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_120MB     (14 << CAN_CTRL2_RFFN_SHIFT)
#  define CAN_CTRL2_RFFN_128MB     (15 << CAN_CTRL2_RFFN_SHIFT)
#define CAN_CTRL2_WRMFRZ           (1 << 28) /* Bit 28: Write-Access to Memory in Freeze mode */
                                             /* Bits 29-31: Reserved */
/* Error and Status 2 Register */
                                             /* Bits 0-12: Reserved */
#define CAN_ESR2_IMB               (1 << 13) /* Bit 13: Inactive Mailbox */
#define CAN_ESR2_VPS               (1 << 14) /* Bit 14: Valid Priority Status */
                                             /* Bit 15: Reserved */
#define CAN_ESR2_VPS               (1 << 14) /* Bit 14: Valid Priority Status */
#define CAN_ESR2_LPTM_SHIFT        (16)      /* Bits 16-22: Lowest Priority Tx Mailbox */
#define CAN_ESR2_LPTM_MASK         (0x7f << CAN_ESR2_LPTM_SHIFT)
                                             /* Bits 23-31: Reserved */
/* CRC Register */
                                             /* Bits 23-31: Reserved */
#define CAN_CRCR_MBCRC_SHIFT       (16)      /* Bits 16-22: CRC Mailbox */
#define CAN_CRCR_MBCRC_MASK        (0x7f << CAN_CRCR_MBCRC_SHIFT)
                                             /* Bit 15: Reserved */
#define CAN_CRCR_TXCRC_SHIFT       (0)       /* Bits 0-14: CRC Transmitted */
#define CAN_CRCR_TXCRC_MASK        (0x7fff << CAN_CRCR_TXCRC_SHIFT)

/* Rx FIFO Global Mask Register (32 Rx FIFO Global Mask Bits) */

/* Rx FIFO Information Register */
                                             /* Bits 9-31: Reserved */
#define CAN_RXFIR_IDHIT_SHIFT      (0)       /* Bits 0-8: Identifier Acceptance Filter Hit Indicator */
#define CAN_RXFIR_IDHIT_MASK       (0x1ff << CAN_RXFIR_IDHIT_SHIFT)

/* Rn Individual Mask Registers */

#define CAN_RXIMR(n)               (1 << (n)) /* Bit n: Individual Mask Bits */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_FLEXCAN_H */
