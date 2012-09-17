/************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_can.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_CAN_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_CAN_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define LPC43_CAN_CNTL_OFFSET        0x0000 /* CAN control register */
#define LPC43_CAN_STAT_OFFSET        0x0004 /* Status register */
#define LPC43_CAN_EC_OFFSET          0x0008 /* Error counter register */
#define LPC43_CAN_BT_OFFSET          0x000c /* Bit timing register */
#define LPC43_CAN_INT_OFFSET         0x0010 /* Interrupt register */
#define LPC43_CAN_TEST_OFFSET        0x0014 /* Test register */
#define LPC43_CAN_BRPE_OFFSET        0x0018 /* Baud rate prescaler extension register */

#define LPC43_CAN_IF1_CMDREQ_OFFSET  0x0020 /* Message interface 1 command request */
#define LPC43_CAN_IF1_CMDMSKW_OFFSET 0x0024 /* Message interface 1 command mask (write) */
#define LPC43_CAN_IF1_CMDMSKR_OFFSET 0x0024 /* Message interface 1 command mask (read) */
#define LPC43_CAN_IF1_MSK1_OFFSET    0x0028 /* Message interface 1 mask 1 */
#define LPC43_CAN_IF1_MSK2_OFFSET    0x002c /* Message interface 1 mask 2 */
#define LPC43_CAN_IF1_ARB1_OFFSET    0x0030 /* Message interface 1 arbitration */
#define LPC43_CAN_IF1_ARB2_OFFSET    0x0034 /* Message interface 1 arbitration */
#define LPC43_CAN_IF1_MCTRL_OFFSET   0x0038 /* Message interface 1 message control */
#define LPC43_CAN_IF1_DA1_OFFSET     0x003c /* Message interface 1 data A1 */
#define LPC43_CAN_IF1_DA2_OFFSET     0x0040 /* Message interface 1 data A2 */
#define LPC43_CAN_IF1_DB1_OFFSET     0x0044 /* Message interface 1 data B1 */
#define LPC43_CAN_IF1_DB2_OFFSET     0x0048 /* Message interface 1 data B2 */

#define LPC43_CAN_IF2_CMDREQ_OFFSET  0x0080 /* Message interface 2 command request */
#define LPC43_CAN_IF2_CMDMSKW_OFFSET 0x0084 /* Message interface 2 command mask (write) */
#define LPC43_CAN_IF2_CMDMSKR_OFFSET 0x0084 /* Message interface 2 command mask (read) */
#define LPC43_CAN_IF2_MSK1_OFFSET    0x0088 /* Message interface 2 mask 1 */
#define LPC43_CAN_IF2_MSK2_OFFSET    0x008c /* Message interface 2 mask 2 */
#define LPC43_CAN_IF2_ARB1_OFFSET    0x0090 /* Message interface 2 arbitration 1 */
#define LPC43_CAN_IF2_ARB2_OFFSET    0x0094 /* Message interface 2 arbitration 2 */
#define LPC43_CAN_IF2_MCTRL_OFFSET   0x0098 /* Message interface 2 message control */
#define LPC43_CAN_IF2_DA1_OFFSET     0x009c /* Message interface 2 data A1 */
#define LPC43_CAN_IF2_DA2_OFFSET     0x00a0 /* Message interface 2 data A2 */
#define LPC43_CAN_IF2_DB1_OFFSET     0x00a4 /* Message interface 2 data B1 */
#define LPC43_CAN_IF2_DB2_OFFSET     0x00a8 /* Message interface 2 data B2 */

#define LPC43_CAN_TXREQ1_OFFSET      0x0100 /* Transmission request 1 */
#define LPC43_CAN_TXREQ2_OFFSET      0x0104 /* Transmission request 2 */
#define LPC43_CAN_ND1_OFFSET         0x0120 /* New data 1 */
#define LPC43_CAN_ND2_OFFSET         0x0124 /* New data 2 */
#define LPC43_CAN_IR1_OFFSET         0x0140 /* Interrupt pending 1 */
#define LPC43_CAN_IR2_OFFSET         0x0144 /* Interrupt pending 2 */
#define LPC43_CAN_MSGV1_OFFSET       0x0160 /* Message valid 1 */
#define LPC43_CAN_MSGV2_OFFSET       0x0164 /* Message valid 2 */
#define LPC43_CAN_CLKDIV_OFFSET      0x0180 /* CAN clock divider register */

/* Register Addresses ***************************************************************/

#define LPC43_CAN1_CNTL              (LPC43_CAN1_BASE+LPC43_CAN_CNTL_OFFSET)
#define LPC43_CAN1_STAT              (LPC43_CAN1_BASE+LPC43_CAN_STAT_OFFSET)
#define LPC43_CAN1_EC                (LPC43_CAN1_BASE+LPC43_CAN_EC_OFFSET)
#define LPC43_CAN1_BT                (LPC43_CAN1_BASE+LPC43_CAN_BT_OFFSET)
#define LPC43_CAN1_INT               (LPC43_CAN1_BASE+LPC43_CAN_INT_OFFSET)
#define LPC43_CAN1_TEST              (LPC43_CAN1_BASE+LPC43_CAN_TEST_OFFSET)
#define LPC43_CAN1_BRPE              (LPC43_CAN1_BASE+LPC43_CAN_BRPE_OFFSET)

#define LPC43_CAN1_IF1_CMDREQ        (LPC43_CAN1_BASE+LPC43_CAN_IF1_CMDREQ_OFFSET)
#define LPC43_CAN1_IF1_CMDMSKW       (LPC43_CAN1_BASE+LPC43_CAN_IF1_CMDMSKW_OFFSET)
#define LPC43_CAN1_IF1_CMDMSKR       (LPC43_CAN1_BASE+LPC43_CAN_IF1_CMDMSKR_OFFSET)
#define LPC43_CAN1_IF1_MSK1          (LPC43_CAN1_BASE+LPC43_CAN_IF1_MSK1_OFFSET)
#define LPC43_CAN1_IF1_MSK2          (LPC43_CAN1_BASE+LPC43_CAN_IF1_MSK2_OFFSET)
#define LPC43_CAN1_IF1_ARB1          (LPC43_CAN1_BASE+LPC43_CAN_IF1_ARB1_OFFSET)
#define LPC43_CAN1_IF1_ARB2          (LPC43_CAN1_BASE+LPC43_CAN_IF1_ARB2_OFFSET)
#define LPC43_CAN1_IF1_MCTRL         (LPC43_CAN1_BASE+LPC43_CAN_IF1_MCTRL_OFFSET)
#define LPC43_CAN1_IF1_DA1           (LPC43_CAN1_BASE+LPC43_CAN_IF1_DA1_OFFSET)
#define LPC43_CAN1_IF1_DA2           (LPC43_CAN1_BASE+LPC43_CAN_IF1_DA2_OFFSET)
#define LPC43_CAN1_IF1_DB1           (LPC43_CAN1_BASE+LPC43_CAN_IF1_DB1_OFFSET)
#define LPC43_CAN1_IF1_DB2           (LPC43_CAN1_BASE+LPC43_CAN_IF1_DB2_OFFSET)

#define LPC43_CAN1_IF2_CMDREQ        (LPC43_CAN1_BASE+LPC43_CAN_IF2_CMDREQ_OFFSET)
#define LPC43_CAN1_IF2_CMDMSKW       (LPC43_CAN1_BASE+LPC43_CAN_IF2_CMDMSKW_OFFSET)
#define LPC43_CAN1_IF2_CMDMSKR       (LPC43_CAN1_BASE+LPC43_CAN_IF2_CMDMSKR_OFFSET)
#define LPC43_CAN1_IF2_MSK1          (LPC43_CAN1_BASE+LPC43_CAN_IF2_MSK1_OFFSET)
#define LPC43_CAN1_IF2_MSK2          (LPC43_CAN1_BASE+LPC43_CAN_IF2_MSK2_OFFSET)
#define LPC43_CAN1_IF2_ARB1          (LPC43_CAN1_BASE+LPC43_CAN_IF2_ARB1_OFFSET)
#define LPC43_CAN1_IF2_ARB2          (LPC43_CAN1_BASE+LPC43_CAN_IF2_ARB2_OFFSET)
#define LPC43_CAN1_IF2_MCTRL         (LPC43_CAN1_BASE+LPC43_CAN_IF2_MCTRL_OFFSET)
#define LPC43_CAN1_IF2_DA1           (LPC43_CAN1_BASE+LPC43_CAN_IF2_DA1_OFFSET)
#define LPC43_CAN1_IF2_DA2           (LPC43_CAN1_BASE+LPC43_CAN_IF2_DA2_OFFSET)
#define LPC43_CAN1_IF2_DB1           (LPC43_CAN1_BASE+LPC43_CAN_IF2_DB1_OFFSET)
#define LPC43_CAN1_IF2_DB2           (LPC43_CAN1_BASE+LPC43_CAN_IF2_DB2_OFFSET)

#define LPC43_CAN1_TXREQ1            (LPC43_CAN1_BASE+LPC43_CAN_TXREQ1_OFFSET)
#define LPC43_CAN1_TXREQ2            (LPC43_CAN1_BASE+LPC43_CAN_TXREQ2_OFFSET)
#define LPC43_CAN1_ND1               (LPC43_CAN1_BASE+LPC43_CAN_ND1_OFFSET)
#define LPC43_CAN1_ND2               (LPC43_CAN1_BASE+LPC43_CAN_ND2_OFFSET)
#define LPC43_CAN1_IR1               (LPC43_CAN1_BASE+LPC43_CAN_IR1_OFFSET)
#define LPC43_CAN1_IR2               (LPC43_CAN1_BASE+LPC43_CAN_IR2_OFFSET)
#define LPC43_CAN1_MSGV1             (LPC43_CAN1_BASE+LPC43_CAN_MSGV1_OFFSET)
#define LPC43_CAN1_MSGV2             (LPC43_CAN1_BASE+LPC43_CAN_MSGV2_OFFSET)
#define LPC43_CAN1_CLKDIV            (LPC43_CAN1_BASE+LPC43_CAN_CLKDIV_OFFSET)

#define LPC43_CAN2_CNTL              (LPC43_CAN2_BASE+LPC43_CAN_CNTL_OFFSET)
#define LPC43_CAN2_STAT              (LPC43_CAN2_BASE+LPC43_CAN_STAT_OFFSET)
#define LPC43_CAN2_EC                (LPC43_CAN2_BASE+LPC43_CAN_EC_OFFSET)
#define LPC43_CAN2_BT                (LPC43_CAN2_BASE+LPC43_CAN_BT_OFFSET)
#define LPC43_CAN2_INT               (LPC43_CAN2_BASE+LPC43_CAN_INT_OFFSET)
#define LPC43_CAN2_TEST              (LPC43_CAN2_BASE+LPC43_CAN_TEST_OFFSET)
#define LPC43_CAN2_BRPE              (LPC43_CAN2_BASE+LPC43_CAN_BRPE_OFFSET)

#define LPC43_CAN2_IF1_CMDREQ        (LPC43_CAN2_BASE+LPC43_CAN_IF1_CMDREQ_OFFSET)
#define LPC43_CAN2_IF1_CMDMSKW       (LPC43_CAN2_BASE+LPC43_CAN_IF1_CMDMSKW_OFFSET)
#define LPC43_CAN2_IF1_CMDMSKR       (LPC43_CAN2_BASE+LPC43_CAN_IF1_CMDMSKR_OFFSET)
#define LPC43_CAN2_IF1_MSK1          (LPC43_CAN2_BASE+LPC43_CAN_IF1_MSK1_OFFSET)
#define LPC43_CAN2_IF1_MSK2          (LPC43_CAN2_BASE+LPC43_CAN_IF1_MSK2_OFFSET)
#define LPC43_CAN2_IF1_ARB1          (LPC43_CAN2_BASE+LPC43_CAN_IF1_ARB1_OFFSET)
#define LPC43_CAN2_IF1_ARB2          (LPC43_CAN2_BASE+LPC43_CAN_IF1_ARB2_OFFSET)
#define LPC43_CAN2_IF1_MCTRL         (LPC43_CAN2_BASE+LPC43_CAN_IF1_MCTRL_OFFSET)
#define LPC43_CAN2_IF1_DA1           (LPC43_CAN2_BASE+LPC43_CAN_IF1_DA1_OFFSET)
#define LPC43_CAN2_IF1_DA2           (LPC43_CAN2_BASE+LPC43_CAN_IF1_DA2_OFFSET)
#define LPC43_CAN2_IF1_DB1           (LPC43_CAN2_BASE+LPC43_CAN_IF1_DB1_OFFSET)
#define LPC43_CAN2_IF1_DB2           (LPC43_CAN2_BASE+LPC43_CAN_IF1_DB2_OFFSET)

#define LPC43_CAN2_IF2_CMDREQ        (LPC43_CAN2_BASE+LPC43_CAN_IF2_CMDREQ_OFFSET)
#define LPC43_CAN2_IF2_CMDMSKW       (LPC43_CAN2_BASE+LPC43_CAN_IF2_CMDMSKW_OFFSET)
#define LPC43_CAN2_IF2_CMDMSKR       (LPC43_CAN2_BASE+LPC43_CAN_IF2_CMDMSKR_OFFSET)
#define LPC43_CAN2_IF2_MSK1          (LPC43_CAN2_BASE+LPC43_CAN_IF2_MSK1_OFFSET)
#define LPC43_CAN2_IF2_MSK2          (LPC43_CAN2_BASE+LPC43_CAN_IF2_MSK2_OFFSET)
#define LPC43_CAN2_IF2_ARB1          (LPC43_CAN2_BASE+LPC43_CAN_IF2_ARB1_OFFSET)
#define LPC43_CAN2_IF2_ARB2          (LPC43_CAN2_BASE+LPC43_CAN_IF2_ARB2_OFFSET)
#define LPC43_CAN2_IF2_MCTRL         (LPC43_CAN2_BASE+LPC43_CAN_IF2_MCTRL_OFFSET)
#define LPC43_CAN2_IF2_DA1           (LPC43_CAN2_BASE+LPC43_CAN_IF2_DA1_OFFSET)
#define LPC43_CAN2_IF2_DA2           (LPC43_CAN2_BASE+LPC43_CAN_IF2_DA2_OFFSET)
#define LPC43_CAN2_IF2_DB1           (LPC43_CAN2_BASE+LPC43_CAN_IF2_DB1_OFFSET)
#define LPC43_CAN2_IF2_DB2           (LPC43_CAN2_BASE+LPC43_CAN_IF2_DB2_OFFSET)

#define LPC43_CAN2_TXREQ1            (LPC43_CAN2_BASE+LPC43_CAN_TXREQ1_OFFSET)
#define LPC43_CAN2_TXREQ2            (LPC43_CAN2_BASE+LPC43_CAN_TXREQ2_OFFSET)
#define LPC43_CAN2_ND1               (LPC43_CAN2_BASE+LPC43_CAN_ND1_OFFSET)
#define LPC43_CAN2_ND2               (LPC43_CAN2_BASE+LPC43_CAN_ND2_OFFSET)
#define LPC43_CAN2_IR1               (LPC43_CAN2_BASE+LPC43_CAN_IR1_OFFSET)
#define LPC43_CAN2_IR2               (LPC43_CAN2_BASE+LPC43_CAN_IR2_OFFSET)
#define LPC43_CAN2_MSGV1             (LPC43_CAN2_BASE+LPC43_CAN_MSGV1_OFFSET)
#define LPC43_CAN2_MSGV2             (LPC43_CAN2_BASE+LPC43_CAN_MSGV2_OFFSET)
#define LPC43_CAN2_CLKDIV            (LPC43_CAN2_BASE+LPC43_CAN_CLKDIV_OFFSET)

/* Register Bit Definitions *********************************************************/

/* CAN control register */

#define CAN_CNTL_INIT                (1 << 0)  /* Bit 0:  Initialization */
#define CAN_CNTL_IE                  (1 << 1)  /* Bit 1:  Module interrupt enable */
#define CAN_CNTL_SIE                 (1 << 2)  /* Bit 2:  Status change interrupt enable */
#define CAN_CNTL_EIE                 (1 << 3)  /* Bit 3:  Error interrupt enable */
                                               /* Bit 4:  Reserved */
#define CAN_CNTL_DAR                 (1 << 5)  /* Bit 5:  Disable automatic retransmission */
#define CAN_CNTL_CCE                 (1 << 6)  /* Bit 6:  Configuration change enable */
#define CAN_CNTL_TEST                (1 << 7)  /* Bit 7:  Test mode enable */
                                               /* Bits 8-31: Reserved */
/* Status register */

#define CAN_STAT_LEC_SHIFT           (0)       /* Bits 0-2: Last error code */
#define CAN_STAT_LEC_MASK            (7 << CAN_STAT_LEC_SHIFT)
#define CAN_STAT_LEC_NOE             (0 << CAN_STAT_LEC_SHIFT) /* No error */
#define CAN_STAT_LEC_STUFFE          (1 << CAN_STAT_LEC_SHIFT) /* Stuff error */
#define CAN_STAT_LEC_FORME           (2 << CAN_STAT_LEC_SHIFT) /* Form error */
#define CAN_STAT_LEC_ACKE            (3 << CAN_STAT_LEC_SHIFT) /* AckError */
#define CAN_STAT_LEC_BI1E            (4 << CAN_STAT_LEC_SHIFT) /* Bit1Error */
#define CAN_STAT_LEC_BIT0E           (5 << CAN_STAT_LEC_SHIFT) /* Bit0Error */
#define CAN_STAT_LEC_CRCE            (6 << CAN_STAT_LEC_SHIFT) /* CRCError */
#define CAN_STAT_TXOK                (1 << 3)  /* Bit 3:  Transmitted a message successfully */
#define CAN_STAT_RXOK                (1 << 4)  /* Bit 4:  Received a message successfully */
#define CAN_STAT_EPASS               (1 << 5)  /* Bit 5:  Error passive */
#define CAN_STAT_EWARN               (1 << 6)  /* Bit 6:  Warning status */
#define CAN_STAT_BOFF                (1 << 7)  /* Bit 7:  Busoff status */
                                               /* Bits 8-31: Reserved */
/* Error counter register */

#define CAN_EC_TEC_SHIFT             (0)       /* Bits 0-7: Transmit error counter */
#define CAN_EC_TEC_MASK              (0xff << CAN_EC_TEC_SHIFT)
#define CAN_EC_REC_SHIFT             (8)       /* Bits 8-14: Receive error counter */
#define CAN_EC_REC_MASK              (0x7f << CAN_EC_REC_SHIFT)
#define CAN_EC_RP                    (1 << 15)  /* Bit 15:  Receive error passive */
                                                /* Bits 16-31: Reserved */
/* Bit timing register */

#define CAN_BT_BRP_SHIFT             (0)        /* Bits 0-5: Baud rate prescaler */
#define CAN_BT_BRP_MASK              (0x3f << CAN_BT_BRP_SHIFT)
#define CAN_BT_SJW_SHIFT             (6)        /* Bits 6-7: (Re)synchronization jump width */
#define CAN_BT_SJW_MASK              (3 << CAN_BT_SJW_SHIFT)
#define CAN_BT_TSEG1_SHIFT           (8)        /* Bits 8-11: Time segment after the sample point */
#define CAN_BT_TSEG1_MASK            (15 << CAN_BT_TSEG1_SHIFT)
#define CAN_BT_TSEG2_SHIFT           (12)       /* Bits 12-14: Time segment before the sample point */
#define CAN_BT_TSEG2_MASK            (7 << CAN_BT_TSEG2_SHIFT)
                                                /* Bits 15-31: Reserved */
/* Interrupt register */

#define CAN_INT_SHIFT                (0)       /* Bits 0-15: Interrupt ID */
#define CAN_INT_MASK                 (0xffff << CAN_INT_SHIFT)
#  define CAN_INT_NONE               (0 << CAN_INT_SHIFT)  /* No interrupt pending */
#  define CAN_INT_MSG1               (1 << CAN_INT_SHIFT)  /* Message 1 */
#  define CAN_INT_MSG2               (2 << CAN_INT_SHIFT)  /* Message 2 */
#  define CAN_INT_MSG3               (3 << CAN_INT_SHIFT)  /* Message 3 */
#  define CAN_INT_MSG4               (4 << CAN_INT_SHIFT)  /* Message 4 */
#  define CAN_INT_MSG5               (5 << CAN_INT_SHIFT)  /* Message 5 */
#  define CAN_INT_MSG6               (6 << CAN_INT_SHIFT)  /* Message 6 */
#  define CAN_INT_MSG7               (7 << CAN_INT_SHIFT)  /* Message 7 */
#  define CAN_INT_MSG8               (8 << CAN_INT_SHIFT)  /* Message 8 */
#  define CAN_INT_MSG9               (9 << CAN_INT_SHIFT)  /* Message 9 */
#  define CAN_INT_MSG10              (10 << CAN_INT_SHIFT) /* Message 10 */
#  define CAN_INT_MSG11              (11 << CAN_INT_SHIFT) /* Message 11 */
#  define CAN_INT_MSG12              (12 << CAN_INT_SHIFT) /* Message 12 */
#  define CAN_INT_MSG13              (13 << CAN_INT_SHIFT) /* Message 13 */
#  define CAN_INT_MSG14              (14 << CAN_INT_SHIFT) /* Message 14 */
#  define CAN_INT_MSG15              (15 << CAN_INT_SHIFT) /* Message 15 */
#  define CAN_INT_MSG16              (16 << CAN_INT_SHIFT) /* Message 16 */
#  define CAN_INT_MSG17              (17 << CAN_INT_SHIFT) /* Message 17 */
#  define CAN_INT_MSG18              (18 << CAN_INT_SHIFT) /* Message 18 */
#  define CAN_INT_MSG19              (19 << CAN_INT_SHIFT) /* Message 19 */
#  define CAN_INT_MSG20              (20 << CAN_INT_SHIFT) /* Message 20 */
#  define CAN_INT_MSG21              (21 << CAN_INT_SHIFT) /* Message 21 */
#  define CAN_INT_MSG22              (22 << CAN_INT_SHIFT) /* Message 22 */
#  define CAN_INT_MSG23              (23 << CAN_INT_SHIFT) /* Message 23 */
#  define CAN_INT_MSG24              (24 << CAN_INT_SHIFT) /* Message 24 */
#  define CAN_INT_MSG25              (25 << CAN_INT_SHIFT) /* Message 25 */
#  define CAN_INT_MSG26              (26 << CAN_INT_SHIFT) /* Message 26 */
#  define CAN_INT_MSG27              (27 << CAN_INT_SHIFT) /* Message 27 */
#  define CAN_INT_MSG28              (28 << CAN_INT_SHIFT) /* Message 28 */
#  define CAN_INT_MSG29              (29 << CAN_INT_SHIFT) /* Message 29 */
#  define CAN_INT_MSG30              (30 << CAN_INT_SHIFT) /* Message 30 */
#  define CAN_INT_MSG31              (31 << CAN_INT_SHIFT) /* Message 31 */
#  define CAN_INT_MSG32              (32 << CAN_INT_SHIFT) /* Message 32 */
#  define CAN_INT_MSG32              (0x8000 << CAN_INT_SHIFT) /* Status interrupt */
                                               /* Bits 16-31: Reserved */
/* Test register */
                                               /* Bits 0-1: Reserved */
#define CAN_TEST_BASIC               (1 << 2)  /* Bit 2:  Basic mode */
#define CAN_TEST_SILENT              (1 << 3)  /* Bit 3:  Silent mode */
#define CAN_TEST_LBACK               (1 << 4)  /* Bit 4:  Loop back mode */
#define CAN_TEST_TX_SHIFT            (5)       /* Bits 5-6: Control of TD pins */
#define CAN_TEST_TX_MASK             (3 << CAN_TEST_TX_SHIFT)
#  define CAN_TEST_TX_CAN            (0 << CAN_TEST_TX_SHIFT) /* Level controlled CAN controller */
#  define CAN_TEST_TX_MONITOR        (1 << CAN_TEST_TX_SHIFT) /* Sample point monitored TD pin */
#  define CAN_TEST_TX_DOMINANT       (2 << CAN_TEST_TX_SHIFT) /* TD pin LOW/dominant */
#  define CAN_TEST_TX_RECESSIVE      (3 << CAN_TEST_TX_SHIFT) /* TD pin HIGH/recessive */
#define CAN_TEST_RX                  (1 << 7)  /* Bit 7:  Monitors actual value of RD Pin */
                                               /* Bits 8-31: Reserved */
/* Baud rate prescaler extension register */

#define CAN_BRPE_SHIFT               (0)       /* Bits 0-3: Baud rate prescaler extension */
#define CAN_BRPE_MASK                (15 << CAN_BRPE_SHIFT)
                                               /* Bits 4-31: Reserved */
/* Message interface 1/2 command request */

#define CAN_CMDREQ_MSGNO_SHIFT       (0)       /* Bits 0-5: Message number */
#define CAN_CMDREQ_MSGNO_MASK        (0x3f << CAN_CMDREQ_MSGNO_SHIFT)
                                               /* Bits 6-14: Reserved */
#define CAN_CMDREQ_BUSY              (1 << 15) /* Bit 15: BUSY flag */
                                               /* Bits 16-31: Reserved */
/* Message interface 1/2 command mask (write) */

#define CAN_CMDMSKW_DATAB            (1 << 0)  /* Bit 0:  Access data bytes 4-7 */
#define CAN_CMDMSKW_DATAA            (1 << 1)  /* Bit 1:  Access data bytes 0-3 */
#define CAN_CMDMSKW_TXRQST           (1 << 2)  /* Bit 2:  Access transmission request bit */
#define CAN_CMDMSKW_CLRINTPND        (1 << 3)  /* Bit 3:  Ignored in the write direction */
#define CAN_CMDMSKW_CTRL             (1 << 4)  /* Bit 4:  Access control bits */
#define CAN_CMDMSKW_ARB              (1 << 5)  /* Bit 5:  Access arbitration bits */
#define CAN_CMDMSKW_MASK             (1 << 6)  /* Bit 6:  Access mask bits */
#define CAN_CMDMSKW_WRRD             (1 << 7)  /* Bit 7:  Write transfer (1) */
                                               /* Bits 8-31: Reserved */
/* Message interface 1/2 command mask (read) */

#define CAN_CMDMSKR_DATAB            (1 << 0)  /* Bit 0:  Access data bytes 4-7 */
#define CAN_CMDMSKR_DATAA            (1 << 1)  /* Bit 1:  Access data bytes 0-3 */
#define CAN_CMDMSKR_NEWDAT           (1 << 2)  /* Bit 2:  Access new data bit */
#define CAN_CMDMSKR_CLRINTPND        (1 << 3)  /* Bit 3:  Clear interrupt pending bit */
#define CAN_CMDMSKR_CTRL             (1 << 4)  /* Bit 4:  Access control bits */
#define CAN_CMDMSKR_ARB              (1 << 5)  /* Bit 5:  Access arbitration bits */
#define CAN_CMDMSKR_MASK             (1 << 6)  /* Bit 6:  Access mask bits */
#define CAN_CMDMSKR_WRRD             (1 << 7)  /* Bit 7:  Read transfer (0) */
                                               /* Bits 8-31: Reserved */
/* Message interface 1/2 mask 1 */

#define CAN_MSK1                     0xffff    /* Bits 0-15: Identifier mask 0-15 */
                                               /* Bits 16-31: Reserved */
/* Message interface 1/2 mask 2 */

#define CAN_MSK2                     0x1fff    /* Bits 0-12: Identifier mask 16-28 */
                                               /* Bit 13: Reserved */
#define CAN_MSK2_MDIR                (1 << 14) /* Bit 14: Mask message direction */
#define CAN_MSK2_MXTD                (1 << 15) /* Bit 15: Mask extend identifier */
                                               /* Bits 16-31: Reserved */
/* Message interface 1/2 arbitration */

#define CAN_ARB1                     0xffff    /* Bits 0-15: Identifier mask 0-15 */
                                               /* Bits 16-31: Reserved */
/* Message interface 1/2 arbitration */

#define CAN_MSK2                     0x1fff    /* Bits 0-12: Identifier mask 16-28 */
#define CAN_MSK2_DIR                 (1 << 13) /* Bit 13: Message direction */
#define CAN_MSK2_XTD                 (1 << 14) /* Bit 14: Extend identifier */
#define CAN_MSK2_MSGVAL              (1 << 15) /* Bit 15: Message valid */
                                               /* Bits 16-31: Reserved */
/* Message interface 1 message control */

#define CAN_MCTRL_DLC_SHIFT          (0)       /* Bits 0-3: Data length code */
#define CAN_MCTRL_DLC_MASK           (15 << CAN_MCTRL_DLC_SHIFT)
                                               /* Bits 4-6: Reserved */
#define CAN_MCTRL_EOB                (1 << 7)  /* Bit 7:  End of buffer */
#define CAN_MCTRL_TXRQST             (1 << 8)  /* Bit 8:  Transmit request */
#define CAN_MCTRL_RMTEN              (1 << 9)  /* Bit 9:  Remote enable */
#define CAN_MCTRL_RXIE               (1 << 10) /* Bit 10: Receive interrupt enable */
#define CAN_MCTRL_TXIE               (1 << 11) /* Bit 11: Transmit interrupt enable */
#define CAN_MCTRL_UMASK              (1 << 12) /* Bit 12: Use acceptance mask */
#define CAN_MCTRL_INTPND             (1 << 13) /* Bit 13: Interrupt pending */
#define CAN_MCTRL_MSGLST             (1 << 14) /* Bit 14: Message lost */
#define CAN_MCTRL_NEWDAT             (1 << 15) /* Bit 15: New data */
                                               /* Bits 16-31: Reserved */
/* Message interface 1/2 data A1 */

#define CAN_DA1_DATA0_SHIFT          (0)       /* Bits 0-7: Data byte 0 */
#define CAN_DA1_DATA0_MASK           (0xff << CAN_DA1_DATA0_SHIFT)
#define CAN_DA1_DATA1_SHIFT          (8)       /* Bits 8-15: Data byte 1 */
#define CAN_DA1_DATA1_MASK           (0xff << CAN_DA1_DATA1_SHIFT)
                                               /* Bits 16-31: Reserved */
/* Message interface 1/2 data A2 */

#define CAN_DA2_DATA2_SHIFT          (0)       /* Bits 0-7: Data byte 2 */
#define CAN_DA2_DATA2_MASK           (0xff << CAN_DA2_DATA2_SHIFT)
#define CAN_DA2_DATA3_SHIFT          (8)       /* Bits 8-15: Data byte 3 */
#define CAN_DA2_DATA3_MASK           (0xff << CAN_DA2_DATA3_SHIFT)
                                               /* Bits 16-31: Reserved */
/* Message interface 1/2 data B1 */

#define CAN_DB1_DATA4_SHIFT          (0)       /* Bits 0-7: Data byte 4 */
#define CAN_DB1_DATA4_MASK           (0xff << CAN_DB1_DATA4_SHIFT)
#define CAN_DB1_DATA5_SHIFT          (8)       /* Bits 8-15: Data byte 5 */
#define CAN_DB1_DATA5_MASK           (0xff << CAN_DB1_DATA5_SHIFT)
                                               /* Bits 16-31: Reserved */
/* Message interface 1/2 data B2 */

#define CAN_DB2_DATA6_SHIFT          (0)       /* Bits 0-7: Data byte 6 */
#define CAN_DB2_DATA6_MASK           (0xff << CAN_DB2_DATA6_SHIFT)
#define CAN_DB2_DATA7_SHIFT          (8)       /* Bits 8-15: Data byte 7 */
#define CAN_DB2_DATA7_MASK           (0xff << CAN_DB2_DATA6_SHIFT)
                                               /* Bits 16-31: Reserved */
/* Transmission request 1 */

#define CAN_TXREQ1_MASK              0xffff    /* Bits 0-15: TX request bit msg 1-16 */
#define CAN_TXREQ1(n)                (1 << ((n)-1)
                                               /* Bits 16-31: Reserved */
/* Transmission request 2 */

#define CAN_TXREQ2_MASK              0xffff    /* Bits 0-15: TX request bit msg 17-32 */
#define CAN_TXREQ2(n)                (1 << ((n)-17)
                                               /* Bits 16-31: Reserved */
/* New data 1 */

#define CAN_ND1_MASK                 0xffff    /* Bits 0-15: New data bits msg 1-16 */
#define CAN_ND1(n)                   (1 << ((n)-1)
                                               /* Bits 16-31: Reserved */
/* New data 2 */

#define CAN_ND2_MASK                 0xffff    /* Bits 0-15: New data bits msg 17-32 */
#define CAN_ND2(n)                   (1 << ((n)-17)
                                               /* Bits 16-31: Reserved */
/* Interrupt pending 1 */

#define CAN_IR1_MASK                 0xffff    /* Bits 0-15: Interrup pending msg 1-16 */
#define CAN_IR1(n)                   (1 << ((n)-1)
                                               /* Bits 16-31: Reserved */
/* Interrupt pending 2 */

#define CAN_IR2_MASK                 0xffff    /* Bits 0-15: Interrup pending msg 17-32 */
#define CAN_IR2(n)                   (1 << ((n)-17)
                                               /* Bits 16-31: Reserved */
/* Message valid 1 */

#define CAN_MSGV1_MASK               0xffff    /* Bits 0-15: Interrup pending msg 1-16 */
#define CAN_MSGV1(n)                 (1 << ((n)-1)
                                               /* Bits 16-31: Reserved */
/* Message valid 2 */

#define CAN_MSGV2_MASK               0xffff    /* Bits 0-15: Interrup pending msg 17-32 */
#define CAN_MSGV2(n)                 (1 << ((n)-17)
                                               /* Bits 16-31: Reserved */
/* CAN clock divider register */

#define CAN_CLKDIV_SHIFT            (0)       /* Bits 0-3: Clock divider value */
#define CAN_CLKDIV_MASK             (15 << CAN_CLKDIV_SHIFT)
#  define CAN_CLKDIV_DIV1           (0 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 1 */
#  define CAN_CLKDIV_DIV2           (1 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 2 */
#  define CAN_CLKDIV_DIV3           (2 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 3 */
#  define CAN_CLKDIV_DIV5           (3 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 5 */
#  define CAN_CLKDIV_DIV9           (4 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 9 */
#  define CAN_CLKDIV_DIV 17         (5 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 17 */
#  define CAN_CLKDIV_DIV33          (6 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 33 */
#  define CAN_CLKDIV_DIV65          (7 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 65 */
#  define CAN_CLKDIV_DIV129         (8 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 129 */
#  define CAN_CLKDIV_DIV257         (9 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 257 */
#  define CAN_CLKDIV_DIV513         (10 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 513 */
#  define CAN_CLKDIV_DIV1025        (11 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 1025 */
#  define CAN_CLKDIV_DIV2049        (12 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 2049 */
#  define CAN_CLKDIV_DIV4097        (13 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 4097 */
#  define CAN_CLKDIV_DIV8093        (14 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 8093 */
#  define CAN_CLKDIV_DIV16385       (15 << CAN_CLKDIV_SHIFT) /* CAN_CLK = PCLK / 16385 */
                                               /* Bits 4-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_CAN_H */
