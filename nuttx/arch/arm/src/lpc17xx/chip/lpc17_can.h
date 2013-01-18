/************************************************************************************
 * arch/arm/src/lpc17xx/chip/lpc17_can.h
 *
 *   Copyright (C) 2010-2012, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_CHIP_CAN_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_CHIP_CAN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc17_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/
/* CAN acceptance filter registers */

#define LPC17_CANAF_AFMR_OFFSET     0x0000 /* Acceptance Filter Register */
#define LPC17_CANAF_SFFSA_OFFSET    0x0004 /* Standard Frame Individual Start Address Register */
#define LPC17_CANAF_SFFGRPSA_OFFSET 0x0008 /* Standard Frame Group Start Address Register */
#define LPC17_CANAF_EFFSA_OFFSET    0x000c /* Extended Frame Start Address Register */
#define LPC17_CANAF_EFFGRPSA_OFFSET 0x0010 /* Extended Frame Group Start Address Register */
#define LPC17_CANAF_EOT_OFFSET      0x0014 /* End of AF Tables register */
#define LPC17_CANAF_LUTERRAD_OFFSET 0x0018 /* LUT Error Address register */
#define LPC17_CANAF_LUTERR_OFFSET   0x001c /* LUT Error Register */
#define LPC17_CANAF_FCANIE_OFFSET   0x0020 /* FullCAN interrupt enable register */
#define LPC17_CANAF_FCANIC0_OFFSET  0x0024 /* FullCAN interrupt and capture register 0 */
#define LPC17_CANAF_FCANIC1_OFFSET  0x0028 /* FullCAN interrupt and capture register 1 */

/* Central CAN registers */

#define LPC17_CAN_TXSR_OFFSET       0x0000 /* CAN Central Transmit Status Register */
#define LPC17_CAN_RXSR_OFFSET       0x0004 /* CAN Central Receive Status Register */
#define LPC17_CAN_MSR_OFFSET        0x0008 /* CAN Central Miscellaneous Register */

/* CAN1/2 registers */

#define LPC17_CAN_MOD_OFFSET        0x0000 /* CAN operating mode */
#define LPC17_CAN_CMR_OFFSET        0x0004 /* Command bits */
#define LPC17_CAN_GSR_OFFSET        0x0008 /* Controller Status and Error Counters */
#define LPC17_CAN_ICR_OFFSET        0x000c /* Interrupt and capure register */
#define LPC17_CAN_IER_OFFSET        0x0010 /* Interrupt Enable */
#define LPC17_CAN_BTR_OFFSET        0x0014 /* Bus Timing */
#define LPC17_CAN_EWL_OFFSET        0x0018 /* Error Warning Limit */
#define LPC17_CAN_SR_OFFSET         0x001c /* Status Register */
#define LPC17_CAN_RFS_OFFSET        0x0020 /* Receive frame status */
#define LPC17_CAN_RID_OFFSET        0x0024 /* Received Identifier */
#define LPC17_CAN_RDA_OFFSET        0x0028 /* Received data bytes 1-4 */
#define LPC17_CAN_RDB_OFFSET        0x002c /* Received data bytes 5-8 */
#define LPC17_CAN_TFI1_OFFSET       0x0030 /* Transmit frame info (Tx Buffer 1) */
#define LPC17_CAN_TID1_OFFSET       0x0034 /* Transmit Identifier (Tx Buffer 1) */
#define LPC17_CAN_TDA1_OFFSET       0x0038 /* Transmit data bytes 1-4 (Tx Buffer 1) */
#define LPC17_CAN_TDB1_OFFSET       0x003c /* Transmit data bytes 5-8 (Tx Buffer 1) */
#define LPC17_CAN_TFI2_OFFSET       0x0040 /* Transmit frame info (Tx Buffer 2) */
#define LPC17_CAN_TID2_OFFSET       0x0044 /* Transmit Identifier (Tx Buffer 2) */
#define LPC17_CAN_TDA2_OFFSET       0x0048 /* Transmit data bytes 1-4 (Tx Buffer 2) */
#define LPC17_CAN_TDB2_OFFSET       0x004c /* Transmit data bytes 5-8 (Tx Buffer 2) */
#define LPC17_CAN_TFI3_OFFSET       0x0050 /* Transmit frame info (Tx Buffer 3) */
#define LPC17_CAN_TID3_OFFSET       0x0054 /* Transmit Identifier (Tx Buffer 3) */
#define LPC17_CAN_TDA3_OFFSET       0x0058 /* Transmit data bytes 1-4 (Tx Buffer 3) */
#define LPC17_CAN_TDB3_OFFSET       0x005c /* Transmit data bytes 5-8 (Tx Buffer 3) */

/* Register addresses ***************************************************************/
/* CAN acceptance filter registers */

#define LPC17_CANAF_AFMR            (LPC17_CANAF_BASE+LPC17_CANAF_AFMR_OFFSET)
#define LPC17_CANAF_SFFSA           (LPC17_CANAF_BASE+LPC17_CANAF_SFFSA_OFFSET)
#define LPC17_CANAF_SFFGRPSA        (LPC17_CANAF_BASE+LPC17_CANAF_SFFGRPSA_OFFSET)
#define LPC17_CANAF_EFFSA           (LPC17_CANAF_BASE+LPC17_CANAF_EFFSA_OFFSET)
#define LPC17_CANAF_EFFGRPSA        (LPC17_CANAF_BASE+LPC17_CANAF_EFFGRPSA_OFFSET)
#define LPC17_CANAF_EOT             (LPC17_CANAF_BASE+LPC17_CANAF_EOT_OFFSET)
#define LPC17_CANAF_LUTERRAD        (LPC17_CANAF_BASE+LPC17_CANAF_LUTERRAD_OFFSET)
#define LPC17_CANAF_LUTERR          (LPC17_CANAF_BASE+LPC17_CANAF_LUTERR_OFFSET)
#define LPC17_CANAF_FCANIE          (LPC17_CANAF_BASE+LPC17_CANAF_FCANIE_OFFSET)
#define LPC17_CANAF_FCANIC0         (LPC17_CANAF_BASE+LPC17_CANAF_FCANIC0_OFFSET)
#define LPC17_CANAF_FCANIC1         (LPC17_CANAF_BASE+LPC17_CANAF_FCANIC1_OFFSET)

/* Central CAN registers */

#define LPC17_CAN_TXSR              (LPC17_CAN_BASE+LPC17_CAN_TXSR_OFFSET)
#define LPC17_CAN_RXSR              (LPC17_CAN_BASE+LPC17_CAN_RXSR_OFFSET)
#define LPC17_CAN_MSR               (LPC17_CAN_BASE+LPC17_CAN_MSR_OFFSET)

/* CAN1/2 registers */

#define LPC17_CAN1_MOD              (LPC17_CAN1_BASE+LPC17_CAN_MOD_OFFSET)
#define LPC17_CAN1_CMR              (LPC17_CAN1_BASE+LPC17_CAN_CMR_OFFSET)
#define LPC17_CAN1_GSR              (LPC17_CAN1_BASE+LPC17_CAN_GSR_OFFSET)
#define LPC17_CAN1_ICR              (LPC17_CAN1_BASE+LPC17_CAN_ICR_OFFSET)
#define LPC17_CAN1_IER              (LPC17_CAN1_BASE+LPC17_CAN_IER_OFFSET)
#define LPC17_CAN1_BTR              (LPC17_CAN1_BASE+LPC17_CAN_BTR_OFFSET)
#define LPC17_CAN1_EWL              (LPC17_CAN1_BASE+LPC17_CAN_EWL_OFFSET)
#define LPC17_CAN1_SR               (LPC17_CAN1_BASE+LPC17_CAN_SR_OFFSET)
#define LPC17_CAN1_RFS              (LPC17_CAN1_BASE+LPC17_CAN_RFS_OFFSET)
#define LPC17_CAN1_RID              (LPC17_CAN1_BASE+LPC17_CAN_RID_OFFSET)
#define LPC17_CAN1_RDA              (LPC17_CAN1_BASE+LPC17_CAN_RDA_OFFSET)
#define LPC17_CAN1_RDB              (LPC17_CAN1_BASE+LPC17_CAN_RDB_OFFSET)
#define LPC17_CAN1_TFI1             (LPC17_CAN1_BASE+LPC17_CAN_TFI1_OFFSET)
#define LPC17_CAN1_TID1             (LPC17_CAN1_BASE+LPC17_CAN_TID1_OFFSET)
#define LPC17_CAN1_TDA1             (LPC17_CAN1_BASE+LPC17_CAN_TDA1_OFFSET)
#define LPC17_CAN1_TDB1             (LPC17_CAN1_BASE+LPC17_CAN_TDB1_OFFSET)
#define LPC17_CAN1_TFI2             (LPC17_CAN1_BASE+LPC17_CAN_TFI2_OFFSET)
#define LPC17_CAN1_TID2             (LPC17_CAN1_BASE+LPC17_CAN_TID2_OFFSET)
#define LPC17_CAN1_TDA2             (LPC17_CAN1_BASE+LPC17_CAN_TDA2_OFFSET)
#define LPC17_CAN1_TDB2             (LPC17_CAN1_BASE+LPC17_CAN_TDB2_OFFSET)
#define LPC17_CAN1_TFI3             (LPC17_CAN1_BASE+LPC17_CAN_TFI3_OFFSET)
#define LPC17_CAN1_TID3             (LPC17_CAN1_BASE+LPC17_CAN_TID3_OFFSET)
#define LPC17_CAN1_TDA3             (LPC17_CAN1_BASE+LPC17_CAN_TDA3_OFFSET)
#define LPC17_CAN1_TDB3             (LPC17_CAN1_BASE+LPC17_CAN_TDB3_OFFSET)

#define LPC17_CAN2_MOD              (LPC17_CAN2_BASE+LPC17_CAN_MOD_OFFSET)
#define LPC17_CAN2_CMR              (LPC17_CAN2_BASE+LPC17_CAN_CMR_OFFSET)
#define LPC17_CAN2_GSR              (LPC17_CAN2_BASE+LPC17_CAN_GSR_OFFSET)
#define LPC17_CAN2_ICR              (LPC17_CAN2_BASE+LPC17_CAN_ICR_OFFSET)
#define LPC17_CAN2_IER              (LPC17_CAN2_BASE+LPC17_CAN_IER_OFFSET)
#define LPC17_CAN2_BTR              (LPC17_CAN2_BASE+LPC17_CAN_BTR_OFFSET)
#define LPC17_CAN2_EWL              (LPC17_CAN2_BASE+LPC17_CAN_EWL_OFFSET)
#define LPC17_CAN2_SR               (LPC17_CAN2_BASE+LPC17_CAN_SR_OFFSET)
#define LPC17_CAN2_RFS              (LPC17_CAN2_BASE+LPC17_CAN_RFS_OFFSET)
#define LPC17_CAN2_RID              (LPC17_CAN2_BASE+LPC17_CAN_RID_OFFSET)
#define LPC17_CAN2_RDA              (LPC17_CAN2_BASE+LPC17_CAN_RDA_OFFSET)
#define LPC17_CAN2_RDB              (LPC17_CAN2_BASE+LPC17_CAN_RDB_OFFSET)
#define LPC17_CAN2_TFI1             (LPC17_CAN2_BASE+LPC17_CAN_TFI1_OFFSET)
#define LPC17_CAN2_TID1             (LPC17_CAN2_BASE+LPC17_CAN_TID1_OFFSET)
#define LPC17_CAN2_TDA1             (LPC17_CAN2_BASE+LPC17_CAN_TDA1_OFFSET)
#define LPC17_CAN2_TDB1             (LPC17_CAN2_BASE+LPC17_CAN_TDB1_OFFSET)
#define LPC17_CAN2_TFI2             (LPC17_CAN2_BASE+LPC17_CAN_TFI2_OFFSET)
#define LPC17_CAN2_TID2             (LPC17_CAN2_BASE+LPC17_CAN_TID2_OFFSET)
#define LPC17_CAN2_TDA2             (LPC17_CAN2_BASE+LPC17_CAN_TDA2_OFFSET)
#define LPC17_CAN2_TDB2             (LPC17_CAN2_BASE+LPC17_CAN_TDB2_OFFSET)
#define LPC17_CAN2_TFI3             (LPC17_CAN2_BASE+LPC17_CAN_TFI3_OFFSET)
#define LPC17_CAN2_TID3             (LPC17_CAN2_BASE+LPC17_CAN_TID3_OFFSET)
#define LPC17_CAN2_TDA3             (LPC17_CAN2_BASE+LPC17_CAN_TDA3_OFFSET)
#define LPC17_CAN2_TDB3             (LPC17_CAN2_BASE+LPC17_CAN_TDB3_OFFSET)

/* Register bit definitions *********************************************************/
/* CAN acceptance filter registers */
/* Acceptance Filter Register */

#define CANAF_AFMR_ACCOFF           (1 << 0)  /* Bit 0:  AF non-operational; All RX messages ignored */
#define CANAF_AFMR_ACCBP            (1 << 1)  /* Bit 1:  AF bypass: All RX messages accepted */
#define CANAF_AFMR_EFCAN            (1 << 2)  /* Bit 2:  Enable Full CAN mode */
                                              /* Bits 3-31: Reserved */
/* Standard Frame Individual Start Address Register */
                                              /* Bits 0-1: Reserved */
#define CANAF_SFFSA_SHIFT           (2)       /* Bits 2-10: Address of Standard Identifiers in AF Lookup RAM */
#define CANAF_SFFSA_MASK            (0x01ff << CANAF_SFFSA_SHIFT)
                                              /* Bits 11-31: Reserved */
/* Standard Frame Group Start Address Register */
                                              /* Bits 0-1: Reserved */
#define CANAF_SFFGRPSA_SHIFT        (2)       /* Bits 2-10: Address of grouped Standard Identifiers in AF Lookup RAM */
#define CANAF_SFFGRPSA_MASK         (0x01ff << CANAF_SFFGRPSA_SHIFT)
                                              /* Bits 11-31: Reserved */
/* Extended Frame Start Address Register */
                                              /* Bits 0-1: Reserved */
#define CANAF_EFFSA_SHIFT           (2)       /* Bits 2-10: Address of Extended Identifiers in AF Lookup RAM */
#define CANAF_EFFSA_MASK            (0x01ff << CANAF_EFFSA_SHIFT)
                                              /* Bits 11-31: Reserved */
/* Extended Frame Group Start Address Register */
                                              /* Bits 0-1: Reserved */
#define CANAF_EFFGRPSA_SHIFT        (2)       /* Bits 2-10: Address of grouped Extended Identifiers in AF Lookup RAM */
#define CANAF_EFFGRPSA_MASK         (0x01ff << CANAF_EFFGRPSA_SHIFT)
                                              /* Bits 11-31: Reserved */
/* End of AF Tables register */
                                              /* Bits 0-1: Reserved */
#define CANAF_EOT_SHIFT             (2)       /* Bits 2-10: Last active address in last active AF table */
#define CANAF_EOT_MASK              (0x01ff << CANAF_EOT_SHIFT)
                                              /* Bits 11-31: Reserved */
/* LUT Error Address register */
                                              /* Bits 0-1: Reserved */
#define CANAF_LUTERRAD_SHIFT        (2)       /* Bits 2-10: Address in AF Lookup RAM of error */
#define CANAF_LUTERRAD_MASK         (0x01ff << CANAF_EOT_SHIFT)
                                              /* Bits 11-31: Reserved */
/* LUT Error Register */

#define CANAF_LUTERR_LUTERR         (1 << 0)  /* Bit 0: AF error in AF RAM tables */
                                              /* Bits 1-31: Reserved */
/* FullCAN interrupt enable register */

#define CANAF_FCANIE_FCANIE         (1 << 0)  /* Bit 0: Global FullCAN Interrupt Enable */
                                              /* Bits 1-31: Reserved */

/* FullCAN interrupt and capture register 0 */

#define CANAF_FCANIC0_INTPND(n)     (1 << (n)) /* n=0,1,2,... 31 */

/* FullCAN interrupt and capture register 1 */

#define CANAF_FCANIC1_INTPND(n)     (1 << ((n)-32)) /* n=32,33,...63 */

/* Central CAN registers */
/* CAN Central Transmit Status Register */

#define CAN_TXSR_TS1                (1 << 0)  /* Bit 0:  CAN1 sending */
#define CAN_TXSR_TS2                (1 << 1)  /* Bit 1:  CAN2 sending */
                                              /* Bits 2-7: Reserved */
#define CAN_TXSR_TBS1               (1 << 8)  /* Bit 8:  All 3 CAN1 TX buffers available */
#define CAN_TXSR_TBS2               (1 << 9)  /* Bit 9:  All 3 CAN2 TX buffers available */
                                              /* Bits 10-15: Reserved */
#define CAN_TXSR_TCS1               (1 << 16) /* Bit 16:  All CAN1 xmissions completed */
#define CAN_TXSR_TCS2               (1 << 17) /* Bit 17:  All CAN2 xmissions completed */
                                              /* Bits 18-31: Reserved */
/* CAN Central Receive Status Register */

#define CAN_RXSR_RS1                (1 << 0)  /* Bit 0:  CAN1 receiving */
#define CAN_RXSR_RS2                (1 << 1)  /* Bit 1:  CAN2 receiving */
                                              /* Bits 2-7: Reserved */
#define CAN_RXSR_RB1                (1 << 8)  /* Bit 8:  CAN1 received message available */
#define CAN_RXSR_RB2                (1 << 9)  /* Bit 9:  CAN2 received message available */
                                              /* Bits 10-15: Reserved */
#define CAN_RXSR_DOS1               (1 << 16) /* Bit 16:  All CAN1 message lost */
#define CAN_RXSR_DOS2               (1 << 17) /* Bit 17:  All CAN2 message lost */
                                              /* Bits 18-31: Reserved */
/* CAN Central Miscellaneous Register */

#define CAN_MSR_E1                  (1 << 0)  /* Bit 0:  CAN1 error counters at limit */
#define CAN_MSR_E2                  (1 << 1)  /* Bit 1:  CAN2 error counters at limit */
                                              /* Bits 2-7: Reserved */
#define CAN_MSR_BS1                 (1 << 8)  /* Bit 8:  CAN1 busy */
#define CAN_MSR_BS2                 (1 << 9)  /* Bit 7:  CAN2 busy */
                                              /* Bits 10-31: Reserved */
/* CAN1/2 registers */
/* CAN operating mode */

#define CAN_MOD_RM                  (1 << 0)  /* Bit 0:  Reset Mode */
#define CAN_MOD_LOM                 (1 << 1)  /* Bit 1:  Listen Only Mode */
#define CAN_MOD_STM                 (1 << 2)  /* Bit 2:  Self Test Mode */
#define CAN_MOD_TPM                 (1 << 3)  /* Bit 3:  Transmit Priority Mode */
#define CAN_MOD_SM                  (1 << 4)  /* Bit 4:  Sleep Mode */
#define CAN_MOD_RPM                 (1 << 5)  /* Bit 5:  Receive Polarity Mode */
                                              /* Bit 6:  Reserved */
#define CAN_MOD_TM                  (1 << 7)  /* Bit 7:  Test Mode */
                                              /* Bits 8-31: Reserved */
/* Command bits */

#define CAN_CMR_TR                  (1 << 0)  /* Bit 0:  Transmission Request */
#define CAN_CMR_AT                  (1 << 1)  /* Bit 1:  Abort Transmission */
#define CAN_CMR_RRB                 (1 << 2)  /* Bit 2:  Release Receive Buffer */
#define CAN_CMR_CDO                 (1 << 3)  /* Bit 3:  Clear Data Overrun */
#define CAN_CMR_SRR                 (1 << 4)  /* Bit 4:  Self Reception Request */
#define CAN_CMR_STB1                (1 << 5)  /* Bit 5:  Select Tx Buffer 1 */
#define CAN_CMR_STB2                (1 << 6)  /* Bit 6:  Select Tx Buffer 2 */
#define CAN_CMR_STB3                (1 << 7)  /* Bit 7:  Select Tx Buffer 3 */
                                              /* Bits 8-31: Reserved */
/* Controller Status and Error Counters */

#define CAN_GSR_RBS                 (1 << 0)  /* Bit 0:  Receive Buffer Status */
#define CAN_GSR_DOS                 (1 << 1)  /* Bit 1:  Data Overrun Status */
#define CAN_GSR_TBS                 (1 << 2)  /* Bit 2:  Transmit Buffer Status */
#define CAN_GSR_TCS                 (1 << 3)  /* Bit 3:  Transmit Complete Status */
#define CAN_GSR_RS                  (1 << 4)  /* Bit 4:  Receive Status */
#define CAN_GSR_TS                  (1 << 5)  /* Bit 5:  Transmit Status */
#define CAN_GSR_ES                  (1 << 6)  /* Bit 6:  Error Status */
#define CAN_GSR_BS                  (1 << 7)  /* Bit 7:  Bus Status */
                                              /* Bits 8-15: Reserved */
#define CAN_GSR_RXERR_SHIFT         (16)      /* Bits 16-23: Rx Error Counter */
#define CAN_GSR_RXERR_MASK          (0xff << CAN_GSR_RXERR_SHIFT)
#define CAN_GSR_TXERR_SHIFT         (24)       /* Bits 24-31: Tx Error Counter */
#define CAN_GSR_TXERR_MASK          (0xff << CAN_GSR_TXERR_SHIFT)

/* Interrupt and capture register */

#define CAN_ICR_RI                  (1 << 0)  /* Bit 0:  Receive Interrupt */
#define CAN_ICR_TI1                 (1 << 1)  /* Bit 1:  Transmit Interrupt 1 */
#define CAN_ICR_EI                  (1 << 2)  /* Bit 2:  Error Warning Interrupt */
#define CAN_ICR_DOI                 (1 << 3)  /* Bit 3:  Data Overrun Interrupt */
#define CAN_ICR_WUI                 (1 << 4)  /* Bit 4:  Wake-Up Interrupt */
#define CAN_ICR_EPI                 (1 << 5)  /* Bit 5:  Error Passive Interrupt */
#define CAN_ICR_ALI                 (1 << 6)  /* Bit 6:  Arbitration Lost Interrupt */
#define CAN_ICR_BEI                 (1 << 7)  /* Bit 7:  Bus Error Interrupt */
#define CAN_ICR_IDI                 (1 << 8)  /* Bit 8:  ID Ready Interrupt */
#define CAN_ICR_TI2                 (1 << 9)  /* Bit 9:  Transmit Interrupt 2 */
#define CAN_ICR_TI3                 (1 << 10) /* Bit 10: Transmit Interrupt 3 */
                                              /* Bits 11-15: Reserved */
#define CAN_ICR_ERRBIT_SHIFT        (16)      /* Bits 16-20: Error Code Capture */
#define CAN_ICR_ERRBIT_MASK         (0x1f << CAN_ICR_ERRBIT_SHIFT)
# define CAN_ICR_ERRBIT_SOF         (3  << CAN_ICR_ERRBIT_SHIFT) /* Start of Frame */
# define CAN_ICR_ERRBIT_ID28        (2  << CAN_ICR_ERRBIT_SHIFT) /* ID28 ... ID21 */
# define CAN_ICR_ERRBIT_SRTR        (4  << CAN_ICR_ERRBIT_SHIFT) /* SRTR Bit */
# define CAN_ICR_ERRBIT_IDE         (5  << CAN_ICR_ERRBIT_SHIFT) /* DE bit */
# define CAN_ICR_ERRBIT_ID20        (6  << CAN_ICR_ERRBIT_SHIFT) /* ID20 ... ID18 */
# define CAN_ICR_ERRBIT_ID17        (7  << CAN_ICR_ERRBIT_SHIFT) /* ID17 ... 13 */
# define CAN_ICR_ERRBIT_CRC         (8  << CAN_ICR_ERRBIT_SHIFT) /* CRC Sequence */
# define CAN_ICR_ERRBIT_DATA        (10 << CAN_ICR_ERRBIT_SHIFT) /* Data Field */
# define CAN_ICR_ERRBIT_LEN         (11 << CAN_ICR_ERRBIT_SHIFT) /* Data Length Code */
# define CAN_ICR_ERRBIT_ RTR        (12 << CAN_ICR_ERRBIT_SHIFT) /* RTR Bit */
# define CAN_ICR_ERRBIT_ID4         (14 << CAN_ICR_ERRBIT_SHIFT) /* ID4 ... ID0 */
# define CAN_ICR_ERRBIT_ID12        (15 << CAN_ICR_ERRBIT_SHIFT) /* ID12 ... ID5 */
# define CAN_ICR_ERRBIT_AERR        (17 << CAN_ICR_ERRBIT_SHIFT) /* Active Error Flag */
# define CAN_ICR_ERRBIT_INTERMSN    (18 << CAN_ICR_ERRBIT_SHIFT) /* Intermission */
# define CAN_ICR_ERRBIT_DOM         (19 << CAN_ICR_ERRBIT_SHIFT) /* Tolerate Dominant Bits */
# define CAN_ICR_ERRBIT_PERR        (22 << CAN_ICR_ERRBIT_SHIFT) /* Passive Error Flag */
# define CAN_ICR_ERRBIT_ERRDLM      (23 << CAN_ICR_ERRBIT_SHIFT) /* Error Delimiter */
# define CAN_ICR_ERRBIT_CRCDLM      (24 << CAN_ICR_ERRBIT_SHIFT) /* CRC Delimiter */
# define CAN_ICR_ERRBIT_ACKSLT      (25 << CAN_ICR_ERRBIT_SHIFT) /* Acknowledge Slot */
# define CAN_ICR_ERRBIT_EOF         (26 << CAN_ICR_ERRBIT_SHIFT) /* End of Frame */
# define CAN_ICR_ERRBIT_ACKDLM      (27 << CAN_ICR_ERRBIT_SHIFT) /* Acknowledge Delimiter */
# define CAN_ICR_ERRBIT_OVLD        (28 << CAN_ICR_ERRBIT_SHIFT) /* Overload flag */
#define CAN_ICR_ERRDIR              (1 << 21) /* Bit 21: Direction bit at time of error */
#define CAN_ICR_ERRC_SHIFT          (22)      /* Bits 22-23: Type of error */
#define CAN_ICR_ERRC_MASK           (3 << CAN_ICR_ERRC_SHIFT)
#  define CAN_ICR_ERRC_BIT          (0 << CAN_ICR_ERRC_SHIFT)
#  define CAN_ICR_ERRC_FORM         (1 << CAN_ICR_ERRC_SHIFT)
#  define CAN_ICR_ERRC_STUFF        (2 << CAN_ICR_ERRC_SHIFT)
#  define CAN_ICR_ERRC_OTHER        (3 << CAN_ICR_ERRC_SHIFT)
#define CAN_ICR_ALCBIT_SHIFT        (24)      /* Bits 24-31: Bit number within frame */
#define CAN_ICR_ALCBIT_MASK         (0xff << CAN_ICR_ALCBIT_SHIFT)

/* Interrupt Enable */

#define CAN_IER_RIE                 (1 << 0)  /* Bit 0:  Receiver Interrupt Enable */
#define CAN_IER_TIE1                (1 << 1)  /* Bit 1:  Transmit Interrupt Enable for Buffer1 */
#define CAN_IER_EIE                 (1 << 2)  /* Bit 2:  Error Warning Interrupt Enable */
#define CAN_IER_DOIE                (1 << 3)  /* Bit 3:  Data Overrun Interrupt Enable */
#define CAN_IER_WUIE                (1 << 4)  /* Bit 4:  Wake-Up Interrupt Enable */
#define CAN_IER_EPIE                (1 << 5)  /* Bit 5:  Error Passive Interrupt Enable */
#define CAN_IER_ALIE                (1 << 6)  /* Bit 6:  Arbitration Lost Interrupt Enable */
#define CAN_IER_BEIE                (1 << 7)  /* Bit 7:  Bus Error Interrupt */
#define CAN_IER_IDIE                (1 << 8)  /* Bit 8:  ID Ready Interrupt Enable */
#define CAN_IER_TIE2                (1 << 9)  /* Bit 9:  Transmit Interrupt Enable for Buffer2 */
#define CAN_IER_TIE3                (1 << 10) /* Bit 10: Transmit Interrupt Enable for Buffer3 */
                                              /* Bits 11-31: Reserved */
/* Bus Timing */

#define CAN_BTR_BRP_SHIFT           (0)       /* Bits 0-9: Baud Rate Prescaler */
#define CAN_BTR_BRP_MASK            (0x3ff << CAN_BTR_BRP_SHIFT)
                                              /* Bits 10-13: Reserved */
#define CAN_BTR_SJW_SHIFT           (14)      /* Bits 14-15: Synchronization Jump Width */
#define CAN_BTR_SJW_MASK            (3 << CAN_BTR_SJW_SHIFT)
#define CAN_BTR_TSEG1_SHIFT         (16)      /* Bits 16-19: Sync to sample delay */
#define CAN_BTR_TSEG1_MASK          (15 << CAN_BTR_TSEG1_SHIFT)
#define CAN_BTR_TSEG2_SHIFT         (20)      /* Bits 20-22: smaple to next delay */
#define CAN_BTR_TSEG2_MASK          (7 << CAN_BTR_TSEG2_SHIFT)
#define CAN_BTR_SAM                 (1 << 23) /* Bit 23: Sampling */
                                              /* Bits 24-31: Reserved */

#define CAN_BTR_BRP_MAX             (1024)    /* Maximum BTR value (without decrement) */
#define CAN_BTR_TSEG1_MAX           (16)      /* Maximum TSEG1 value (without decrement) */
#define CAN_BTR_TSEG2_MAX           (8)       /* Maximum TSEG2 value (without decrement) */

/* Error Warning Limit */

#define CAN_EWL_SHIFT               (0)       /* Bits 0-7: Error warning limit */ 
#define CAN_EWL_MASK                (0xff << CAN_EWL_SHIFT)
                                              /* Bits 8-31: Reserved */
/* Status Register */

#define CAN_SR_RBS1                 (1 << 0)  /* Bit 0:  Receive Buffer Status */
#define CAN_SR_DOS1                 (1 << 1)  /* Bit 1:  Data Overrun Status */
#define CAN_SR_TBS1                 (1 << 2)  /* Bit 2:  Transmit Buffer Status 1 */
#define CAN_SR_TCS1                 (1 << 3)  /* Bit 3:  Transmission Complete Status */
#define CAN_SR_RS1                  (1 << 4)  /* Bit 4:  Receive Status */
#define CAN_SR_TS1                  (1 << 5)  /* Bit 5:  Transmit Status 1 */
#define CAN_SR_ES1                  (1 << 6)  /* Bit 6:  Error Status */
#define CAN_SR_BS1                  (1 << 7)  /* Bit 7:  Bus Status */
#define CAN_SR_RBS2                 (1 << 8)  /* Bit 8:  Receive Buffer Status */
#define CAN_SR_DOS2                 (1 << 9)  /* Bit 9:  Data Overrun Status */
#define CAN_SR_TBS2                 (1 << 10) /* Bit 10: Transmit Buffer Status 2 */
#define CAN_SR_TCS2                 (1 << 11) /* Bit 11: Transmission Complete Status */
#define CAN_SR_RS2                  (1 << 12) /* Bit 12: Receive Status */
#define CAN_SR_TS2                  (1 << 13) /* Bit 13: Transmit Status 2 */
#define CAN_SR_ES2                  (1 << 14) /* Bit 14: Error Status */
#define CAN_SR_BS2                  (1 << 15) /* Bit 15: Bus Status  */
#define CAN_SR_RBS3                 (1 << 16) /* Bit 16: Receive Buffer Status */
#define CAN_SR_DOS3                 (1 << 17) /* Bit 17: Data Overrun Status */
#define CAN_SR_TBS3                 (1 << 18) /* Bit 18: Transmit Buffer Status 3 */
#define CAN_SR_TCS3                 (1 << 19) /* Bit 19: Transmission Complete Status */
#define CAN_SR_RS3                  (1 << 20) /* Bit 20: Receive Status */
#define CAN_SR_TS3                  (1 << 21) /* Bit 21: Transmit Status 3 */
#define CAN_SR_ES3                  (1 << 22) /* Bit 22: Error Status */
#define CAN_SR_BS3                  (1 << 23) /* Bit 23: Bus Status */
                                              /* Bits 24-31: Reserved */
/* Receive frame status */

#define CAN_RFS_ID_SHIFT            (0)       /* Bits 0-9: ID Index */
#define CAN_RFS_ID_MASK             (0x03ff << CAN_RFS_ID_SHIFT)
#define CAN_RFS_BP                  (1 << 10) /* Bit 10: Received in AF Bypass mode */
                                              /* Bits 11-15: Reserved */
#define CAN_RFS_DLC_SHIFT           (16)      /* Bits 16-19: Message Data Length Code (DLC) */
#define CAN_RFS_DLC_MASK            (15 << CAN_RFS_DLC_SHIFT)
                                              /* Bits 20-29: Reserved */
#define CAN_RFS_RTR                 (1 << 30) /* Bit 30: Message Remote Transmission Request */
#define CAN_RFS_FF                  (1 << 31) /* Bit 31: Message 29-bit vs 11-bit ID */

/* Received Identifier */

#define CAN_RID_ID11_MASK           (0x7ff)   /* Bits 0-10: 11-bit Identifier (FF=0) */
                                              /* Bits 11-31: Reserved */
#define CAN_RID_ID29_MASK           (0x1fffffff) /* Bits 0-28: 29-bit Identifiter (FF=1) */
                                              /* Bits 29-31: Reserved */
/* Received data bytes 1-4 */

#define CAN_RDA_DATA1_SHIFT         (0)       /* Bits 0-7: If CANRFS >= 1 */
#define CAN_RDA_DATA1_MASK          (0x0ff << CAN_RDA_DATA1_SHIFT)
#define CAN_RDA_DATA2_SHIFT         (8)       /* Bits 8-15: If CANRFS >= 2 */
#define CAN_RDA_DATA2_MASK          (0x0ff << CAN_RDA_DATA2_SHIFT)
#define CAN_RDA_DATA3_SHIFT         (16)      /* Bits 16-23: If CANRFS >= 3 */
#define CAN_RDA_DATA3_MASK          (0x0ff << CAN_RDA_DATA3_SHIFT)
#define CAN_RDA_DATA4_SHIFT         (24)      /* Bits 24-31: If CANRFS >= 4 */
#define CAN_RDA_DATA4_MASK          (0x0ff << CAN_RDA_DATA4_SHIFT)

/* Received data bytes 5-8 */

#define CAN_RDB_DATA5_SHIFT         (0)       /* Bits 0-7: If CANRFS >= 5 */
#define CAN_RDB_DATA5_MASK          (0x0ff << CAN_RDB_DATA5_SHIFT)
#define CAN_RDB_DATA6_SHIFT         (8)       /* Bits 8-15: If CANRFS >= 6 */
#define CAN_RDB_DATA6_MASK          (0x0ff << CAN_RDB_DATA6_SHIFT)
#define CAN_RDB_DATA7_SHIFT         (16)      /* Bits 16-23: If CANRFS >= 7 */
#define CAN_RDB_DATA7_MASK          (0x0ff << CAN_RDB_DATA7_SHIFT)
#define CAN_RDB_DATA8_SHIFT         (24)      /* Bits 24-31: If CANRFS >= 8 */
#define CAN_RDB_DATA8_MASK          (0x0ff << CAN_RDB_DATA8_SHIFT)

/* Transmit frame info (Tx Buffer 1),  Transmit frame info (Tx Buffer 2), and
 * Transmit frame info (Tx Buffer 3) common bit field definitions
 */

#define CAN_TFI_PRIO_SHIFT          (0)       /* Bits 0-7: TX buffer priority */
#define CAN_TFI_PRIO_MASK           (0xff << CAN_TFI_PRIO_SHIFT)
                                              /* Bits 8-15: Reserved */
#define CAN_TFI_DLC_SHIFT           (16)      /* Bits 16-19: TX Data Length Code */
#define CAN_TFI_DLC_MASK            (15 << CAN_TFI_DLC_SHIFT)
                                              /* Bits 20-29: Reserved */
#define CAN_TFI_RTR                 (1 << 30) /* Bit 30: TX RTR bit */
#define CAN_TFI_FF                  (1 << 31) /* Bit 31: Message 29-bit vs 11-bit ID */

/* Transmit Identifier (Tx Buffer 1), Transmit Identifier (Tx Buffer 2), and
 * Transmit Identifier (Tx Buffer 3) common bit field definitions.
 */

#define CAN_TID_ID11_MASK           (0x7ff)   /* Bits 0-10: 11-bit Identifier (FF=0) */
                                              /* Bits 11-31: Reserved */
#define CAN_TID_ID29_MASK           (0x1fffffff) /* Bits 0-28: 29-bit Identifiter (FF=1) */
                                              /* Bits 29-31: Reserved */

/* Transmit data bytes 1-4 (Tx Buffer 1), Transmit data bytes 1-4 (Tx Buffer 2), and
 * Transmit data bytes 1-4 (Tx Buffer 3) common bit field definitions.
 */

#define CAN_TDA_DATA1_SHIFT         (0)       /* Bits 0-7: RTR=0 && DLC >= 1 */
#define CAN_TDA_DATA1_MASK          (0x0ff << CAN_TDA_DATA1_SHIFT)
#define CAN_TDA_DATA2_SHIFT         (8)       /* Bits 8-15: RTR=0 && DLC >= 2 */
#define CAN_TDA_DATA2_MASK          (0x0ff << CAN_TDA_DATA2_SHIFT)
#define CAN_TDA_DATA3_SHIFT         (16)      /* Bits 16-23: RTR=0 && DLC >= 3 */
#define CAN_TDA_DATA3_MASK          (0x0ff << CAN_TDA_DATA3_SHIFT)
#define CAN_TDA_DATA4_SHIFT         (24)      /* Bits 24-31: RTR=0 && DLC >= 4 */
#define CAN_TDA_DATA4_MASK          (0x0ff << CAN_TDA_DATA4_SHIFT)

/* Transmit data bytes 5-8 (Tx Buffer 1), Transmit data bytes 5-8 (Tx Buffer 2), and 
 * Transmit data bytes 5-8 (Tx Buffer 3) common bit field definitions.
 */

#define CAN_RDB_DATA5_SHIFT         (0)       /* Bits 0-7: RTR=0 && DLC >= 5 */
#define CAN_RDB_DATA5_MASK          (0x0ff << CAN_RDB_DATA5_SHIFT)
#define CAN_RDB_DATA6_SHIFT         (8)       /* Bits 8-15: RTR=0 && DLC >= 6 */
#define CAN_RDB_DATA6_MASK          (0x0ff << CAN_RDB_DATA6_SHIFT)
#define CAN_RDB_DATA7_SHIFT         (16)      /* Bits 16-23: RTR=0 && DLC >= 7 */
#define CAN_RDB_DATA7_MASK          (0x0ff << CAN_RDB_DATA7_SHIFT)
#define CAN_RDB_DATA8_SHIFT         (24)      /* Bits 24-31: RTR=0 && DLC >= 8 */
#define CAN_RDB_DATA8_MASK          (0x0ff << CAN_RDB_DATA8_SHIFT)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_CHIP_CAN_H */
