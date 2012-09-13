/********************************************************************************************
 * arch/arm/src/kinetis/kinetis_enet.h
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_ENET_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_ENET_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(KINETIS_NENET) &&  KINETIS_NENET > 0

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KINETIS_ENET_EIR_OFFSET      0x0004 /* Interrupt Event Register */
#define KINETIS_ENET_EIMR_OFFSET     0x0008 /* Interrupt Mask Register */
#define KINETIS_ENET_RDAR_OFFSET     0x0010 /* Receive Descriptor Active Register */
#define KINETIS_ENET_TDAR_OFFSET     0x0014 /* Transmit Descriptor Active Register */
#define KINETIS_ENET_ECR_OFFSET      0x0024 /* Ethernet Control Register */
#define KINETIS_ENET_MMFR_OFFSET     0x0040 /* MII Management Frame Register */
#define KINETIS_ENET_MSCR_OFFSET     0x0044 /* MII Speed Control Register */
#define KINETIS_ENET_MIBC_OFFSET     0x0064 /* MIB Control Register */
#define KINETIS_ENET_RCR_OFFSET      0x0084 /* Receive Control Register */
#define KINETIS_ENET_TCR_OFFSET      0x00c4 /* Transmit Control Register */
#define KINETIS_ENET_PALR_OFFSET     0x00e4 /* Physical Address Lower Register */
#define KINETIS_ENET_PAUR_OFFSET     0x00e8 /* Physical Address Upper Register */
#define KINETIS_ENET_OPD_OFFSET      0x00ec /* Opcode/Pause Duration Register */
#define KINETIS_ENET_IAUR_OFFSET     0x0118 /* Descriptor Individual Upper Address Register */
#define KINETIS_ENET_IALR_OFFSET     0x011c /* Descriptor Individual Lower Address Register */
#define KINETIS_ENET_GAUR_OFFSET     0x0120 /* Descriptor Group Upper Address Register */
#define KINETIS_ENET_GALR_OFFSET     0x0124 /* Descriptor Group Lower Address Register */
#define KINETIS_ENET_TFWR_OFFSET     0x0144 /* Transmit FIFO Watermark Register */
#define KINETIS_ENET_RDSR_OFFSET     0x0180 /* Receive Descriptor Ring Start Register */
#define KINETIS_ENET_TDSR_OFFSET     0x0184 /* Transmit Buffer Descriptor Ring Start Register */
#define KINETIS_ENET_MRBR_OFFSET     0x0188 /* Maximum Receive Buffer Size Register */
#define KINETIS_ENET_RSFL_OFFSET     0x0190 /* Receive FIFO Section Full Threshold */
#define KINETIS_ENET_RSEM_OFFSET     0x0194 /* Receive FIFO Section Empty Threshold */
#define KINETIS_ENET_RAEM_OFFSET     0x0198 /* Receive FIFO Almost Empty Threshold */
#define KINETIS_ENET_RAFL_OFFSET     0x019c /* Receive FIFO Almost Full Threshold */
#define KINETIS_ENET_TSEM_OFFSET     0x01a0 /* Transmit FIFO Section Empty Threshold */
#define KINETIS_ENET_TAEM_OFFSET     0x01a4 /* Transmit FIFO Almost Empty Threshold */
#define KINETIS_ENET_TAFL_OFFSET     0x01a8 /* Transmit FIFO Almost Full Threshold */
#define KINETIS_ENET_TIPG_OFFSET     0x01ac /* Transmit Inter-Packet Gap */
#define KINETIS_ENET_FTRL_OFFSET     0x01b0 /* Frame Truncation Length */
#define KINETIS_ENET_TACC_OFFSET     0x01c0 /* Transmit Accelerator Function Configuration */
#define KINETIS_ENET_RACC_OFFSET     0x01c4 /* Receive Accelerator Function Configuration */

#define KINETIS_ENET_ATCR_OFFSET     0x0400 /* Timer Control Register */
#define KINETIS_ENET_ATVR_OFFSET     0x0404 /* Timer Value Register */
#define KINETIS_ENET_ATOFF_OFFSET    0x0408 /* Timer Offset Register */
#define KINETIS_ENET_ATPER_OFFSET    0x040c /* Timer Period Register */
#define KINETIS_ENET_ATCOR_OFFSET    0x0410 /* Timer Correction Register */
#define KINETIS_ENET_ATINC_OFFSET    0x0414 /* Time-Stamping Clock Period Register */
#define KINETIS_ENET_ATSTMP_OFFSET   0x0418 /* Timestamp of Last Transmitted Frame */

#define KINETIS_ENET_TGSR_OFFSET     0x0604 /* Timer Global Status Register */
#define KINETIS_ENET_TCSR0_OFFSET    0x0608 /* Timer Control Status Register */
#define KINETIS_ENET_TCCR0_OFFSET    0x060c /* Timer Compare Capture Register */
#define KINETIS_ENET_TCSR1_OFFSET    0x0610 /* Timer Control Status Register */
#define KINETIS_ENET_TCCR1_OFFSET    0x0614 /* Timer Compare Capture Register */
#define KINETIS_ENET_TCSR2_OFFSET    0x0618 /* Timer Control Status Register */
#define KINETIS_ENET_TCCR2_OFFSET    0x061c /* Timer Compare Capture Register */
#define KINETIS_ENET_TCSR3_OFFSET    0x0620 /* Timer Control Status Register */
#define KINETIS_ENET_TCCR3_OFFSET    0x0624 /* Timer Compare Capture Register */

/* Register Addresses ***********************************************************************/

#define KINETIS_ENET_EIR             (KINETIS_EMAC_BASE+KINETIS_ENET_EIR_OFFSET)
#define KINETIS_ENET_EIMR            (KINETIS_EMAC_BASE+KINETIS_ENET_EIMR_OFFSET)
#define KINETIS_ENET_RDAR            (KINETIS_EMAC_BASE+KINETIS_ENET_RDAR_OFFSET)
#define KINETIS_ENET_TDAR            (KINETIS_EMAC_BASE+KINETIS_ENET_TDAR_OFFSET)
#define KINETIS_ENET_ECR             (KINETIS_EMAC_BASE+KINETIS_ENET_ECR_OFFSET)
#define KINETIS_ENET_MMFR            (KINETIS_EMAC_BASE+KINETIS_ENET_MMFR_OFFSET)
#define KINETIS_ENET_MSCR            (KINETIS_EMAC_BASE+KINETIS_ENET_MSCR_OFFSET)
#define KINETIS_ENET_MIBC            (KINETIS_EMAC_BASE+KINETIS_ENET_MIBC_OFFSET)
#define KINETIS_ENET_RCR             (KINETIS_EMAC_BASE+KINETIS_ENET_RCR_OFFSET)
#define KINETIS_ENET_TCR             (KINETIS_EMAC_BASE+KINETIS_ENET_TCR_OFFSET)
#define KINETIS_ENET_PALR            (KINETIS_EMAC_BASE+KINETIS_ENET_PALR_OFFSET)
#define KINETIS_ENET_PAUR            (KINETIS_EMAC_BASE+KINETIS_ENET_PAUR_OFFSET)
#define KINETIS_ENET_OPD             (KINETIS_EMAC_BASE+KINETIS_ENET_OPD_OFFSET)
#define KINETIS_ENET_IAUR            (KINETIS_EMAC_BASE+KINETIS_ENET_IAUR_OFFSET)
#define KINETIS_ENET_IALR            (KINETIS_EMAC_BASE+KINETIS_ENET_IALR_OFFSET)
#define KINETIS_ENET_GAUR            (KINETIS_EMAC_BASE+KINETIS_ENET_GAUR_OFFSET)
#define KINETIS_ENET_GALR            (KINETIS_EMAC_BASE+KINETIS_ENET_GALR_OFFSET)
#define KINETIS_ENET_TFWR            (KINETIS_EMAC_BASE+KINETIS_ENET_TFWR_OFFSET)
#define KINETIS_ENET_RDSR            (KINETIS_EMAC_BASE+KINETIS_ENET_RDSR_OFFSET)
#define KINETIS_ENET_TDSR            (KINETIS_EMAC_BASE+KINETIS_ENET_TDSR_OFFSET)
#define KINETIS_ENET_MRBR            (KINETIS_EMAC_BASE+KINETIS_ENET_MRBR_OFFSET)
#define KINETIS_ENET_RSFL            (KINETIS_EMAC_BASE+KINETIS_ENET_RSFL_OFFSET)
#define KINETIS_ENET_RSEM            (KINETIS_EMAC_BASE+KINETIS_ENET_RSEM_OFFSET)
#define KINETIS_ENET_RAEM            (KINETIS_EMAC_BASE+KINETIS_ENET_RAEM_OFFSET)
#define KINETIS_ENET_RAFL            (KINETIS_EMAC_BASE+KINETIS_ENET_RAFL_OFFSET)
#define KINETIS_ENET_TSEM            (KINETIS_EMAC_BASE+KINETIS_ENET_TSEM_OFFSET)
#define KINETIS_ENET_TAEM            (KINETIS_EMAC_BASE+KINETIS_ENET_TAEM_OFFSET)
#define KINETIS_ENET_TAFL            (KINETIS_EMAC_BASE+KINETIS_ENET_TAFL_OFFSET)
#define KINETIS_ENET_TIPG            (KINETIS_EMAC_BASE+KINETIS_ENET_TIPG_OFFSET)
#define KINETIS_ENET_FTRL            (KINETIS_EMAC_BASE+KINETIS_ENET_FTRL_OFFSET)
#define KINETIS_ENET_TACC            (KINETIS_EMAC_BASE+KINETIS_ENET_TACC_OFFSET)
#define KINETIS_ENET_RACC            (KINETIS_EMAC_BASE+KINETIS_ENET_RACC_OFFSET)

#define KINETIS_ENET_ATCR            (KINETIS_EMAC_BASE+KINETIS_ENET_ATCR_OFFSET)
#define KINETIS_ENET_ATVR            (KINETIS_EMAC_BASE+KINETIS_ENET_ATVR_OFFSET)
#define KINETIS_ENET_ATOFF           (KINETIS_EMAC_BASE+KINETIS_ENET_ATOFF_OFFSET)
#define KINETIS_ENET_ATPER           (KINETIS_EMAC_BASE+KINETIS_ENET_ATPER_OFFSET)
#define KINETIS_ENET_ATCOR           (KINETIS_EMAC_BASE+KINETIS_ENET_ATCOR_OFFSET)
#define KINETIS_ENET_ATINC           (KINETIS_EMAC_BASE+KINETIS_ENET_ATINC_OFFSET)
#define KINETIS_ENET_ATSTMP          (KINETIS_EMAC_BASE+KINETIS_ENET_ATSTMP_OFFSET)

#define KINETIS_ENET_TGSR            (KINETIS_EMAC_BASE+KINETIS_ENET_TGSR_OFFSET)
#define KINETIS_ENET_TCSR0           (KINETIS_EMAC_BASE+KINETIS_ENET_TCSR0_OFFSET)
#define KINETIS_ENET_TCCR0           (KINETIS_EMAC_BASE+KINETIS_ENET_TCCR0_OFFSET)
#define KINETIS_ENET_TCSR1           (KINETIS_EMAC_BASE+KINETIS_ENET_TCSR1_OFFSET)
#define KINETIS_ENET_TCCR1           (KINETIS_EMAC_BASE+KINETIS_ENET_TCCR1_OFFSET)
#define KINETIS_ENET_TCSR2           (KINETIS_EMAC_BASE+KINETIS_ENET_TCSR2_OFFSET)
#define KINETIS_ENET_TCCR2           (KINETIS_EMAC_BASE+KINETIS_ENET_TCCR2_OFFSET)
#define KINETIS_ENET_TCSR3           (KINETIS_EMAC_BASE+KINETIS_ENET_TCSR3_OFFSET)
#define KINETIS_ENET_TCCR3           (KINETIS_EMAC_BASE+KINETIS_ENET_TCCR3_OFFSET)

/* Register Bit Definitions *****************************************************************/

/* Interrupt Event Register, Interrupt Mask Register */
                                               /* Bits 0-14: Reserved */
#define ENET_INT_TS_TIMER            (1 << 15) /* Bit 15: Timestamp timer */
#define ENET_INT_TS_AVAIL            (1 << 16) /* Bit 16: Transmit timestamp available */
#define ENET_INT_WAKEUP              (1 << 17) /* Bit 17: Node wake-up request indication */
#define ENET_INT_PLR                 (1 << 18) /* Bit 18: Payload receive error */
#define ENET_INT_UN                  (1 << 19) /* Bit 19: Transmit FIFO underrun */
#define ENET_INT_RL                  (1 << 20) /* Bit 20: Collision Retry Limit */
#define ENET_INT_LC                  (1 << 21) /* Bit 21: Late Collision */
#define ENET_INT_EBERR               (1 << 22) /* Bit 22: Ethernet Bus Error */
#define ENET_INT_MII                 (1 << 23) /* Bit 23: MII Interrupt */
#define ENET_INT_RXB                 (1 << 24) /* Bit 24: Receive Buffer Interrupt */
#define ENET_INT_RXF                 (1 << 25) /* Bit 25: Receive Frame Interrupt */
#define ENET_INT_TXB                 (1 << 26) /* Bit 26: Transmit Buffer Interrupt */
#define ENET_INT_TXF                 (1 << 27) /* Bit 27: Transmit Frame Interrupt */
#define ENET_INT_GRA                 (1 << 28) /* Bit 28: Graceful Stop Complete */
#define ENET_INT_BABT                (1 << 29) /* Bit 29: Babbling Transmit Error */
#define ENET_INT_BABR                (1 << 30) /* Bit 30: Babbling Receive Error */
                                               /* Bit 31: Reserved */
/* Receive Descriptor Active Register */
                                               /* Bits 0-23: Reserved */
#define ENET_RDAR                    (1 << 24) /* Bit 24: Receive descriptor active */
                                               /* Bits 25-31: Reserved */
/* Transmit Descriptor Active Register */
                                               /* Bits 0-23: Reserved */
#define ENET_TDAR                    (1 << 24) /* Bit 24: Transmit descriptor active */
                                               /* Bits 25-31: Reserved */
/* Ethernet Control Register */

#define ENET_ECR_RESET               (1 << 0)  /* Bit 0:  Ethernet MAC reset */
#define ENET_ECR_ETHEREN             (1 << 1)  /* Bit 1:  Ethernet enable */
#define ENET_ECR_MAGICEN             (1 << 2)  /* Bit 2:  Magic packet detection enable */
#define ENET_ECR_SLEEP               (1 << 3)  /* Bit 3:  Sleep mode enable */
#define ENET_ECR_EN1588              (1 << 4)  /* Bit 4:  EN1588 enable */
                                               /* Bit 5: Reserved */
#define ENET_ECR_DBGEN               (1 << 6)  /* Bit 6:  Debug enable */
#define ENET_ECR_STOPEN              (1 << 7)  /* Bit 7:  STOPEN Signal Control */
                                               /* Bits 8-31: Reserved */
/* MII Management Frame Register */

#define ENET_MMFR_DATA_SHIFT         (0)       /* Bits 0-15: Management frame data */
#define ENET_MMFR_DATA_MASK          (0xffff << ENET_MMFR_DATA_SHIFT)
#define ENET_MMFR_TA_SHIFT           (16)      /* Bits 16-17: Turn around */
#define ENET_MMFR_TA_MASK            (3 << ENET_MMFR_TA_SHIFT)
#define ENET_MMFR_RA_SHIFT           (18)      /* Bits 18-22: Register address */
#define ENET_MMFR_RA_MASK            (31 << ENET_MMFR_RA_SHIFT)
#define ENET_MMFR_PA_SHIFT           (23)      /* Bits 23-27: PHY address */
#define ENET_MMFR_PA_MASK            (31 << ENET_MMFR_PA_SHIFT)
#define ENET_MMFR_OP_SHIFT           (28)      /* Bits 28-29: Operation code */
#define ENET_MMFR_OP_MASK            (3 << ENET_MMFR_OP_SHIFT)
#  define ENET_MMFR_OP_WRNOTMII      (0 << ENET_MMFR_OP_SHIFT) /* Write frame, not MII compliant */
#  define ENET_MMFR_OP_WRMII         (1 << ENET_MMFR_OP_SHIFT) /* Write frame, MII management frame */
#  define ENET_MMFR_OP_RDMII         (2 << ENET_MMFR_OP_SHIFT) /* Read frame, MII management frame */
#  define ENET_MMFR_OP_RdNOTMII      (3 << ENET_MMFR_OP_SHIFT) /* Read frame, not MII compliant */
#define ENET_MMFR_ST_SHIFT           (30)      /* Bits 30-31: Start of frame delimiter */
#define ENET_MMFR_ST_MASK            (3 << ENET_MMFR_ST_SHIFT)

/* MII Speed Control Register */
                                               /* Bit 0: Reserved */
#define ENET_MSCR_MII_SPEED_SHIFT    (1)       /* Bits 1-6: MII speed */
#define ENET_MSCR_MII_SPEED_MASK     (63 << ENET_MSCR_MII_SPEED_SHIFT)
#define ENET_MSCR_DIS_PRE            (1 << 7)  /* Bit 7:  Disable preamble */
#define ENET_MSCR_HOLDTIME_SHIFT     (8)       /* Bits 8-10: Holdtime on MDIO output */
#define ENET_MSCR_HOLDTIME_MASK      (7 << ENET_MSCR_HOLDTIME_SHIFT)
#  define ENET_MSCR_HOLDTIME_1CYCLE  (0 << ENET_MSCR_HOLDTIME_SHIFT) /* 1 internal module clock cycle */
#  define ENET_MSCR_HOLDTIME_2CYCLES (1 << ENET_MSCR_HOLDTIME_SHIFT) /* 2 internal module clock cycles */
#  define ENET_MSCR_HOLDTIME_3CYCLES (2 << ENET_MSCR_HOLDTIME_SHIFT) /* 3 internal module clock cycles */
#  define ENET_MSCR_HOLDTIME_8CYCLES (7 << ENET_MSCR_HOLDTIME_SHIFT) /* 8 internal module clock cycles */
                                               /* Bits 11-31: Reserved */
/* MIB Control Register */
                                               /* Bits 0-28: Reserved */
#define ENET_MIBC_MIB_CLEAR          (1 << 29) /* Bit 29: MIB clear */
#define ENET_MIBC_MIB_IDLE           (1 << 30) /* Bit 30: MIB idle */
#define ENET_MIBC_MIB_DIS            (1 << 31) /* Bit 31: Disable MIB logic */

/* Receive Control Register */

#define ENET_RCR_LOOP                (1 << 0)  /* Bit 0:  Internal loopback */
#define ENET_RCR_DRT                 (1 << 1)  /* Bit 1:  Disable receive on transmit */
#define ENET_RCR_MII_MODE            (1 << 2)  /* Bit 2:  Media independent interface mode */
#define ENET_RCR_PROM                (1 << 3)  /* Bit 3:  Promiscuous mode */
#define ENET_RCR_BC_REJ              (1 << 4)  /* Bit 4:  Broadcast frame reject */
#define ENET_RCR_FCE                 (1 << 5)  /* Bit 5:  Flow control enable */
                                               /* Bits 6-7: Reserved */
#define ENET_RCR_RMII_MODE           (1 << 8)  /* Bit 8: RMII mode enable */
#define ENET_RCR_RMII_10T            (1 << 9)  /* Bit 9: Enables 10-Mbps mode of the RMII */
                                               /* Bits 10-11: Reserved */
#define ENET_RCR_PADEN               (1 << 12) /* Bit 12: Enable frame padding remove on receive */
#define ENET_RCR_PAUFWD              (1 << 13) /* Bit 13: Terminate/forward pause frames */
#define ENET_RCR_CRCFWD              (1 << 14) /* Bit 14: Terminate/forward received CRC */
#define ENET_RCR_CFEN                (1 << 15) /* Bit 15: MAC control frame enable */
#define ENET_RCR_MAX_FL_SHIFT        (16)      /* Bits 16-29: Maximum frame length */
#define ENET_RCR_MAX_FL_MASK         (0x3fff << ENET_RCR_MAX_FL_SHIFT)
#define ENET_RCR_NLC                 (1 << 30) /* Bit 30: Payload length check disable */
#define ENET_RCR_GRS                 (1 << 31) /* Bit 31: Graceful receive stopped */

/* Transmit Control Register */

#define ENET_TCR_GTS                 (1 << 0)  /* Bit 0:  Graceful transmit stop */
                                               /* Bit 1: Reserved */
#define ENET_TCR_ADDINS              (1 << 8)  /* Bit 8:  Set MAC address on transmit */
#define ENET_TCR_FDEN                (1 << 2)  /* Bit 2:  Full duplex enable */
#define ENET_TCR_TFC_PAUSE           (1 << 3)  /* Bit 3:  Transmit frame control pause */
#define ENET_TCR_RFC_PAUSE           (1 << 4)  /* Bit 4:  Receive frame control pause */
#define ENET_TCR_ADDSEL_SHIFT        (5)       /* Bits 5-7: Source MAC address select on transmit */
#define ENET_TCR_ADDSEL_MASK         (7 << ENET_TCR_ADDSEL_SHIFT)
#  define ENET_TCR_ADDSEL_PADDR12    (0 << ENET_TCR_ADDSEL_SHIFT) /* Node MAC address programmed on PADDR1/2 registers */
#define ENET_TCR_CRCFWD              (1 << 9)  /* Bit 9:  Forward frame from application with CRC */
                                               /* Bits 10-31: Reserved */
/* Physical Address Lower/Upper Register (32-bits of 48-address) */
/* Physical Address Upper Register */

#define ENET_PAUR_TYPE_SHIFT         (0)       /* Bits 0-15: Type field in PAUSE frame */
#define ENET_PAUR_TYPE_MASK          (0xffff << ENET_PAUR_TYPE_MASK)
#define ENET_PAUR_PADDR2_SHIFT       (16)      /* Bits 16-31: Bytes 4 and 5 of the 6-byte address */
#define ENET_PAUR_PADDR2_MASK        (0xffff << ENET_PAUR_PADDR2_SHIFT)

/* Opcode/Pause Duration Register */

#define ENET_OPD_PAUSE_DUR_SHIFT     (0)       /* Bits 0-15: Pause duration */
#define ENET_OPD_PAUSE_DUR_MASK      (0xffff << ENET_OPD_PAUSE_DUR_SHIFT)
#define ENET_OPD_OPCODE_SHIFT        (16)      /* Bits 16-31: Opcode field in PAUSE frames */
#define ENET_OPD_OPCODE_MASK         (0xffff << ENET_OPD_OPCODE_SHIFT)

/* Descriptor Individual Uupper/Lower Address Register (64-bit address in two 32-bit registers) */
/* Descriptor Group Upper/Lower Address Register (64-bit address in two 32-bit registers) */

/* Transmit FIFO Watermark Register */

#define ENET_TFWR_TFWR_SHIFT         (0)       /* Bits 0-5: Transmit FIFO write */
                                               /* Bits 6-7: Reserved */
#define ENET_TFWR_TFWR_MASK          (63 << ENET_TFWR_TFWR_SHIFT)
#define ENET_TFWR_STRFWD             (1 << 8)  /* Bit 8: Store and forward enable */
                                               /* Bits 9-31: Reserved */
/* Receive Descriptor Ring Start Register */
                                               /* Bits 0-2: Reserved */
#define ENET_RDSR_SHIFT              (3)       /* Bits 3-31: Start of the receive buffer descriptor queue */
#define ENET_RDSR_MASK               (0xfffffff8) 

/* Transmit Buffer Descriptor Ring Start Register */
                                               /* Bits 0-2: Reserved */
#define ENET_TDSR_SHIFT              (3)       /* Bits 3-31: Start of the transmit buffer descriptor queue */
#define ENET_TDSR_MASK               (0xfffffff8)

/* Maximum Receive Buffer Size Register */
                                               /* Bits 14-31: Reserved */
#define ENET_MRBR_SHIFT              (4)       /* Bits 4-13: Receive buffer size in bytes */
#define ENET_MRBR_MASK               (0x3ff << ENET_MRBR_SHIFT)
                                               /* Bits 0-3: Reserved */
/* Receive FIFO Section Full Threshold */
                                               /* Bits 8-31: Reserved */
#define ENET_RSFL_SHIFT              (0)       /* Bits 0-7: Value of receive FIFO section full threshold */
#define ENET_RSFL_MASK               (0xff << ENET_RSFL_SHIFT)

/* Receive FIFO Section Empty Threshold */

#define ENET_RSEM_SHIFT              (0)       /* Bits 0-7: Value of the receive FIFO section empty threshold */
#define ENET_RSEM_MASK               (0xff << ENET_RSEM_SHIFT)
                                               /* Bits 8-31: Reserved */
/* Receive FIFO Almost Empty Threshold */

#define ENET_RAEM_SHIFT              (0)       /* Bits 0-7: Value of the receive FIFO almost empty threshold */
#define ENET_RAEM_MASK               (0xff << ENET_RAEM_SHIFT)
                                               /* Bits 8-31: Reserved */
/* Receive FIFO Almost Full Threshold */

#define ENET_RAFL_SHIFT              (0)       /* Bits 0-7: Value of the receive FIFO almost full threshold */
#define ENET_RAFL_MASK               (0xff << ENET_RAFL_SHIFT)
                                               /* Bits 8-31: Reserved */
/* Transmit FIFO Section Empty Threshold */

#define ENET_TSEM_SHIFT              (0)       /* Bits 0-7: Value of the transmit FIFO section empty threshold */
#define ENET_TSEM_MASK               (0xff << ENET_TSEM_SHIFT)
                                               /* Bits 8-31: Reserved */
/* Transmit FIFO Almost Empty Threshold */

#define ENET_TAEM_SHIFT              (0)       /* Bits 0-7: Value of the transmit FIFO section empty threshold */
#define ENET_TAEM_MASK               (0xff << ENET_TAEM_SHIFT)
                                               /* Bits 8-31: Reserved */
/* Transmit FIFO Almost Full Threshold */

#define ENET_TAFL_SHIFT              (0)       /* Bits 0-7: Value of the transmit FIFO section empty threshold */
#define ENET_TAFL_MASK               (0xff << ENET_TAFL_SHIFT)
                                               /* Bits 8-31: Reserved */
/* Transmit Inter-Packet Gap */

#define ENET_TIPG_SHIFT              (0)       /* Bits 0-4: Value of the transmit FIFO section empty threshold */
#define ENET_TIPG_MASK               (31 << ENET_TIPG_SHIFT)
                                               /* Bits 5-31: Reserved */
/* Frame Truncation Length */

#define ENET_FTRL_SHIFT              (0)       /* Bits 0-13: Value of the transmit FIFO section empty threshold */
#define ENET_FTRL_MASK               (0x3fff << ENET_FTRL_SHIFT)
                                               /* Bits 14-31: Reserved */
/* Transmit Accelerator Function Configuration */

#define ENET_TACC_SHIFT16            (1 << 0)  /* Bit 0:  TX FIFO shift-16 */
                                               /* Bits 1-2: Reserved */
#define ENET_TACC_IPCHK              (1 << 3)  /* Bit 3:  Enables insertion of IP header checksum */
#define ENET_TACC_PROCHK             (1 << 4)  /* Bit 4:  Enables insertion of protocol checksum */
                                               /* Bits 5-31: Reserved */
/* Receive Accelerator Function Configuration */

#define ENET_RACC_PADREM             (1 << 0)  /* Bit 0: Enable padding removal for short IP frames */
#define ENET_RACC_IPDIS              (1 << 1)  /* Bit 1: Enable discard of frames with wrong IPv4 header checksum */
#define ENET_RACC_PRODIS             (1 << 2)  /* Bit 2: Enable discard of frames with wrong protocol checksum */
                                               /* Bits 3-5: Reserved */
#define ENET_RACC_LINEDIS            (1 << 6)  /* Bit 6: Enable discard of frames with MAC layer errors */
#define ENET_RACC_SHIFT16            (1 << 7)  /* Bit 7: RX FIFO shift-16 */
                                               /* Bits 8-31: Reserved */
/* Timer Control Register */

#define ENET_ATCR_EN                 (1 << 0)  /* Bit 0:  Enable timer */
                                               /* Bit 1:  Reserved */
#define ENET_ATCR_OFFEN              (1 << 2)  /* Bit 2:  Enable one-shot offset event */
#define ENET_ATCR_OFFRST             (1 << 3)  /* Bit 3:  Reset timer on offset event */
#define ENET_ATCR_PEREN              (1 << 4)  /* Bit 4:  Enable periodical event */
                                               /* Bits 5-6: Reserved */
#define ENET_ATCR_PINPER             (1 << 7)  /* Bit 7:  Enables event signal output assertion on period event */
                                               /* Bit 8:  Reserved */
#define ENET_ATCR_RESTART            (1 << 9)  /* Bit 9:  Reset timer */
                                               /* Bit 10: Reserved */
#define ENET_ATCR_CAPTURE            (1 << 11) /* Bit 11: Capture timer value */
                                               /* Bit 12: Reserved */
#define ENET_ATCR_SLAVE              (1 << 13) /* Bit 13: Enable timer slave mode */
                                               /* Bits 14-31: Reserved */
/* Timer Value Register (32-bit timer value) */
/* Timer Offset Register (32-bit offset value) */
/* Timer Period Register (32-bit timer period) */

/* Timer Correction Register */

#define ENET_ATCOR_MASK              (0x7fffffff) /* Bits 0-3: Correction counter wrap-around value */
                                               /* Bit 31: Reserved */
/* Time-Stamping Clock Period Register */

#define ENET_ATINC_INC_SHIFT         (0)       /* Bits 0-6: Clock period of the timestamping clock (ts_clk) in nanoseconds */
#define ENET_ATINC_INC_MASK          (0x7f << ENET_ATINC_INC_SHIFT)
                                               /* Bit 7: Reserved */
#define ENET_ATINC_INC_CORR_SHIFT    (8)       /* Bits 8-14: Correction increment value */
#define ENET_ATINC_INC_CORR_MASK     (0x7f << ENET_ATINC_INC_CORR_SHIFT)
                                               /* Bits 15-31: Reserved */
/* Timestamp of Last Transmitted Frame (32-bit timestamp) */

/* Timer Global Status Register */

#define ENET_TGSR_TF0                (1 << 0)  /* Bit 0:  Copy of Timer Flag for channel 0 */
#define ENET_TGSR_TF1                (1 << 1)  /* Bit 1:  Copy of Timer Flag for channel 1 */
#define ENET_TGSR_TF2                (1 << 2)  /* Bit 2:  Copy of Timer Flag for channel 2 */
#define ENET_TGSR_TF3                (1 << 3)  /* Bit 3:  Copy of Timer Flag for channel 3 */
                                               /* Bits 14-31: Reserved */
/* Timer Control Status Register n */

#define ENET_TCSR_TDRE               (1 << 0)  /* Bit 0:  Timer DMA Request Enable */
                                               /* Bit 1: Reserved */
#define ENET_TCSR_TMODE_SHIFT        (2)       /* Bits 2-5: Timer Mode */
#define ENET_TCSR_TMODE_MASK         (15 << ENET_TCSR_TMODE_SHIFT)
#  define ENET_TCSR_TMODE_DISABLED   (0 << ENET_TCSR_TMODE_SHIFT)  /* Disabled */
#  define ENET_TCSR_TMODE_ICRISING   (1 << ENET_TCSR_TMODE_SHIFT)  /* Input Capture on rising edge */
#  define ENET_TCSR_TMODE_ICFALLLING (2 << ENET_TCSR_TMODE_SHIFT)  /* Input Capture on falling edge */
#  define ENET_TCSR_TMODE_ICBOTH     (3 << ENET_TCSR_TMODE_SHIFT)  /* Input Capture on both edges */
#  define ENET_TCSR_TMODE_OCSW       (4 << ENET_TCSR_TMODE_SHIFT)  /* Output Compare, S/W only */
#  define ENET_TCSR_TMODE_OCTOGGLE   (5 << ENET_TCSR_TMODE_SHIFT)  /* Output Compare, toggle on compare */
#  define ENET_TCSR_TMODE_OCCLR      (6 << ENET_TCSR_TMODE_SHIFT)  /* Output Compare, clear on compare */
#  define ENET_TCSR_TMODE_OCSET      (7 << ENET_TCSR_TMODE_SHIFT)  /*  Output Compare, set on compare */
#  define ENET_TCSR_TMODE_OCSETCLR   (9 << ENET_TCSR_TMODE_SHIFT)  /* Output Compare, set on compare, clear on overflow */
#  define ENET_TCSR_TMODE_OCCLRSET   (10 << ENET_TCSR_TMODE_SHIFT) /*  Output Compare, clear on compare, set on overflow */
#  define ENET_TCSR_TMODE_PCPULSEL   (14 << ENET_TCSR_TMODE_SHIFT) /* Output Compare, pulse low on compare */
#  define ENET_TCSR_TMODE_PCPULSEH   (15 << ENET_TCSR_TMODE_SHIFT) /* Output Compare, pulse high on compare */
#define ENET_TCSR_TIE                (1 << 6)  /* Bit 6:  Timer interrupt enable */
#define ENET_TCSR_TF                 (1 << 7)  /* Bit 7:  Timer Flag */
                                               /* Bits 8-31: Reserved */
/* Timer Compare Capture Register (32-bit compare value) */

/* Buffer Descriptors ***********************************************************************/
/* Endian-independent descriptor offsets */

#define DESC_STATUS1_OFFSET         (0)
#define DESC_LENGTH_OFFSET          (2)
#define DESC_DATAPTR_OFFSET         (4)
#define DESC_LEGACY_LEN             (8)

#define DESC_STATUS2_OFFSET         (8)
#define DESC_LENPROTO_OFFSET        (12)
#define DESC_CHECKSUM_OFFSET        (14)
#define DESC_BDU_OFFSET             (16)
#define DESC_TIMESTAMP_OFFSET       (20)
#define DESC_ENHANCED_LEN           (32)

/* Legacy/Common TX Buffer Descriptor Bit Definitions.
 *
 *   The descriptors are represented by structures  Unfortunately, when the
 *   structures are overlayed on the data, the bytes are reversed because
 *   the underlying hardware writes the data in big-endian byte order.
 */

#ifdef CONFIG_ENDIAN_BIG
#  define TXDESC_ABC                 (1 << 9)  /* Legacy */
#  define TXDESC_TC                  (1 << 10) /* Common */
#  define TXDESC_L                   (1 << 11) /* Common */
#  define TXDESC_TO2                 (1 << 12) /* Common */
#  define TXDESC_W                   (1 << 13) /* Common */
#  define TXDESC_TO1                 (1 << 14) /* Common */
#  define TXDESC_R                   (1 << 15) /* Common */
#else
#  define TXDESC_ABC                 (1 << 1)  /* Legacy */
#  define TXDESC_TC                  (1 << 2)  /* Common */
#  define TXDESC_L                   (1 << 3)  /* Common */
#  define TXDESC_TO2                 (1 << 4)  /* Common */
#  define TXDESC_W                   (1 << 5)  /* Common */
#  define TXDESC_TO1                 (1 << 6)  /* Common */
#  define TXDESC_R                   (1 << 7)  /* Common */
#endif

/* Enhanced (only) TX Buffer Descriptor Bit Definitions */

#ifdef CONFIG_ENDIAN_BIG
#  define TXDESC_TSE                 (1 << 8)
#  define TXDESC_OE                  (1 << 9) 
#  define TXDESC_LCE                 (1 << 10) 
#  define TXDESC_FE                  (1 << 11) 
#  define TXDESC_EE                  (1 << 12)
#  define TXDESC_UE                  (1 << 13)
#  define TXDESC_TXE                 (1 << 15)

#  define TXDESC_IINS                (1 << 27)
#  define TXDESC_PINS                (1 << 28)
#  define TXDESC_TS                  (1 << 29)
#  define TXDESC_INT                 (1 << 30)

#  define TXDESC_BDU                 (1 << 31)
#else
#  define TXDESC_IINS                (1 << 3)
#  define TXDESC_PINS                (1 << 4)
#  define TXDESC_TS                  (1 << 5)
#  define TXDESC_INT                 (1 << 6)

#  define TXDESC_TSE                 (1 << 16) 
#  define TXDESC_OE                  (1 << 17) 
#  define TXDESC_LCE                 (1 << 18) 
#  define TXDESC_FE                  (1 << 19) 
#  define TXDESC_EE                  (1 << 20)
#  define TXDESC_UE                  (1 << 21)
#  define TXDESC_TXE                 (1 << 23)

#  define TXDESC_BDU                 (1 << 7)    
#endif

/* Legacy (and Common) RX Buffer Descriptor Bit Definitions */

#ifdef CONFIG_ENDIAN_BIG
#  define RXDESC_TR                  (1 << 0)
#  define RXDESC_OV                  (1 << 1)
#  define RXDESC_CR                  (1 << 2)
#  define RXDESC_NO                  (1 << 4)
#  define RXDESC_LG                  (1 << 5)
#  define RXDESC_MC                  (1 << 6)
#  define RXDESC_BC                  (1 << 7)
#  define RXDESC_M                   (1 << 8)
#  define RXDESC_L                   (1 << 11)
#  define RXDESC_R02                 (1 << 12)
#  define RXDESC_W                   (1 << 13)
#  define RXDESC_R01                 (1 << 14)
#  define RXDESC_E                   (1 << 15)
#else
#  define RXDESC_M                   (1 << 0)
#  define RXDESC_L                   (1 << 3)
#  define RXDESC_R02                 (1 << 4)
#  define RXDESC_W                   (1 << 5)
#  define RXDESC_R01                 (1 << 6)
#  define RXDESC_E                   (1 << 7)
#  define RXDESC_TR                  (1 << 8)
#  define RXDESC_OV                  (1 << 9)
#  define RXDESC_CR                  (1 << 10)
#  define RXDESC_NO                  (1 << 12)
#  define RXDESC_LG                  (1 << 13)
#  define RXDESC_MC                  (1 << 14)
#  define RXDESC_BC                  (1 << 15)
#endif

/* Enhanced (only) TX Buffer Descriptor Bit Definitions */

#ifdef CONFIG_ENDIAN_BIG
#  define RXDESC_FRAG                (1 << 0)
#  define RXDESC_IPV6                (1 << 1)
#  define RXDESC_VLAN                (1 << 2)
#  define RXDESC_PCR                 (1 << 4)
#  define RXDESC_ICE                 (1 << 5)
#  define RXDESC_INT                 (1 << 23)
#  define RXDESC_UC                  (1 << 24)
#  define RXDESC_CE                  (1 << 25)
#  define RXDESC_PE                  (1 << 26)
#  define RXDESC_ME                  (1 << 31)

#  define RXDESC_BDU                 (1 << 31)    
#else
#  define RXDESC_UC                  (1 << 0)
#  define RXDESC_CE                  (1 << 1)
#  define RXDESC_PE                  (1 << 2)
#  define RXDESC_ME                  (1 << 7)
#  define RXDESC_INT                 (1 << 15)
#  define RXDESC_FRAG                (1 << 24)
#  define RXDESC_IPV6                (1 << 25)
#  define RXDESC_VLAN                (1 << 26)
#  define RXDESC_PCR                 (1 << 28)
#  define RXDESC_ICE                 (1 << 29)

#  define RXDESC_BDU                 (1 << 7)
#endif

/********************************************************************************************
 * Public Types
 ********************************************************************************************/
/* Buffer Descriptors ***********************************************************************/
/* Legacy Buffer Descriptor */

#ifdef CONFIG_ENET_ENHANCEDBD
struct enet_desc_s
{
  uint16_t status1;     /* Control and status */
  uint16_t length;      /* Data length */
  uint8_t  *data;       /* Buffer address */
  uint32_t status2;     /* Extended status */
  uint16_t lenproto;    /* Header length + Protocol type */
  uint16_t checksum;    /* Payload checksum */
  uint32_t bdu;         /* BDU */
  uint32_t timestamp;   /* Time stamp */
  uint32_t reserved1;   /* unused */
  uint32_t reserved2;   /* unused */
}
#else
struct enet_desc_s
{
  uint16_t status1;     /* Control and status */
  uint16_t length;      /* Data length */
  uint8_t  *data;       /* Buffer address */
};
#endif

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* KINETIS_NENET &&  KINETIS_NENET > 0 */
#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_ENET_H */
