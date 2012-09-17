/********************************************************************************************
 * arch/arm/src/kinetis/kinetis_dspi.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_DSPI_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_DSPI_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KINETICS_SPI_MCR_OFFSET    0x0000 /* DSPI Module Configuration Register */
#define KINETICS_SPI_TCR_OFFSET    0x0008 /* DSPI Transfer Count Register */
#define KINETICS_SPI_CTAR0_OFFSET  0x000c /* DSPI Clock and Transfer Attributes Register */
#define KINETICS_SPI_CTAR1_OFFSET  0x0010 /* DSPI Clock and Transfer Attributes Register */
#define KINETICS_SPI_SR_OFFSET     0x002c /* DSPI Status Register */
#define KINETICS_SPI_RSER_OFFSET   0x0030 /* DSPI DMA/Interrupt Request Select and Enable Register */
#define KINETICS_SPI_PUSHR_OFFSET  0x0034 /* DSPI PUSH TX FIFO Register */
#define KINETICS_SPI_POPR_OFFSET   0x0038 /* DSPI POP RX FIFO Register */
#define KINETICS_SPI_TXFR0_OFFSET  0x003c /* DSPI Transmit FIFO Registers */
#define KINETICS_SPI_TXFR1_OFFSET  0x0040 /* DSPI Transmit FIFO Registers */
#define KINETICS_SPI_TXFR2_OFFSET  0x0044 /* DSPI Transmit FIFO Registers */
#define KINETICS_SPI_TXFR3_OFFSET  0x0048 /* DSPI Transmit FIFO Registers */
#define KINETICS_SPI_RXFR0_OFFSET  0x007c /* DSPI Receive FIFO Registers */
#define KINETICS_SPI_RXFR1_OFFSET  0x0080 /* DSPI Receive FIFO Registers */
#define KINETICS_SPI_RXFR2_OFFSET  0x0084 /* DSPI Receive FIFO Registers */
#define KINETICS_SPI_RXFR3_OFFSET  0x0088 /* DSPI Receive FIFO Registers */

/* Register Addresses ***********************************************************************/

#define KINETICS_SPI0_MCR          (KINETIS_SPI0_BASE+KINETICS_SPI_MCR_OFFSET)
#define KINETICS_SPI0_TCR          (KINETIS_SPI0_BASE+KINETICS_SPI_TCR_OFFSET)
#define KINETICS_SPI0_CTAR0        (KINETIS_SPI0_BASE+KINETICS_SPI_CTAR0_OFFSET)
#define KINETICS_SPI0_CTAR1        (KINETIS_SPI0_BASE+KINETICS_SPI_CTAR1_OFFSET)
#define KINETICS_SPI0_SR           (KINETIS_SPI0_BASE+KINETICS_SPI_SR_OFFSET)
#define KINETICS_SPI0_RSER         (KINETIS_SPI0_BASE+KINETICS_SPI_RSER_OFFSET)
#define KINETICS_SPI0_PUSHR        (KINETIS_SPI0_BASE+KINETICS_SPI_PUSHR_OFFSET)
#define KINETICS_SPI0_POPR         (KINETIS_SPI0_BASE+KINETICS_SPI_POPR_OFFSET)
#define KINETICS_SPI0_TXFR0        (KINETIS_SPI0_BASE+KINETICS_SPI_TXFR0_OFFSET)
#define KINETICS_SPI0_TXFR1        (KINETIS_SPI0_BASE+KINETICS_SPI_TXFR1_OFFSET)
#define KINETICS_SPI0_TXFR2        (KINETIS_SPI0_BASE+KINETICS_SPI_TXFR2_OFFSET)
#define KINETICS_SPI0_TXFR3        (KINETIS_SPI0_BASE+KINETICS_SPI_TXFR3_OFFSET)
#define KINETICS_SPI0_RXFR0        (KINETIS_SPI0_BASE+KINETICS_SPI_RXFR0_OFFSET)
#define KINETICS_SPI0_RXFR1        (KINETIS_SPI0_BASE+KINETICS_SPI_RXFR1_OFFSET)
#define KINETICS_SPI0_RXFR2        (KINETIS_SPI0_BASE+KINETICS_SPI_RXFR2_OFFSET)
#define KINETICS_SPI0_RXFR3        (KINETIS_SPI0_BASE+KINETICS_SPI_RXFR3_OFFSET)

#define KINETICS_SPI1_MCR          (KINETIS_SPI1_BASE+KINETICS_SPI_MCR_OFFSET)
#define KINETICS_SPI1_TCR          (KINETIS_SPI1_BASE+KINETICS_SPI_TCR_OFFSET)
#define KINETICS_SPI1_CTAR0        (KINETIS_SPI1_BASE+KINETICS_SPI_CTAR0_OFFSET)
#define KINETICS_SPI1_CTAR1        (KINETIS_SPI1_BASE+KINETICS_SPI_CTAR1_OFFSET)
#define KINETICS_SPI1_SR           (KINETIS_SPI1_BASE+KINETICS_SPI_SR_OFFSET)
#define KINETICS_SPI1_RSER         (KINETIS_SPI1_BASE+KINETICS_SPI_RSER_OFFSET)
#define KINETICS_SPI1_PUSHR        (KINETIS_SPI1_BASE+KINETICS_SPI_PUSHR_OFFSET)
#define KINETICS_SPI1_POPR         (KINETIS_SPI1_BASE+KINETICS_SPI_POPR_OFFSET)
#define KINETICS_SPI1_TXFR0        (KINETIS_SPI1_BASE+KINETICS_SPI_TXFR0_OFFSET)
#define KINETICS_SPI1_TXFR1        (KINETIS_SPI1_BASE+KINETICS_SPI_TXFR1_OFFSET)
#define KINETICS_SPI1_TXFR2        (KINETIS_SPI1_BASE+KINETICS_SPI_TXFR2_OFFSET)
#define KINETICS_SPI1_TXFR3        (KINETIS_SPI1_BASE+KINETICS_SPI_TXFR3_OFFSET)
#define KINETICS_SPI1_RXFR0        (KINETIS_SPI1_BASE+KINETICS_SPI_RXFR0_OFFSET)
#define KINETICS_SPI1_RXFR1        (KINETIS_SPI1_BASE+KINETICS_SPI_RXFR1_OFFSET)
#define KINETICS_SPI1_RXFR2        (KINETIS_SPI1_BASE+KINETICS_SPI_RXFR2_OFFSET)
#define KINETICS_SPI1_RXFR3        (KINETIS_SPI1_BASE+KINETICS_SPI_RXFR3_OFFSET)

#define KINETICS_SPI2_MCR          (KINETIS_SPI2_BASE+KINETICS_SPI_MCR_OFFSET)
#define KINETICS_SPI2_TCR          (KINETIS_SPI2_BASE+KINETICS_SPI_TCR_OFFSET)
#define KINETICS_SPI2_CTAR0        (KINETIS_SPI2_BASE+KINETICS_SPI_CTAR0_OFFSET)
#define KINETICS_SPI2_CTAR1        (KINETIS_SPI2_BASE+KINETICS_SPI_CTAR1_OFFSET)
#define KINETICS_SPI2_SR           (KINETIS_SPI2_BASE+KINETICS_SPI_SR_OFFSET)
#define KINETICS_SPI2_RSER         (KINETIS_SPI2_BASE+KINETICS_SPI_RSER_OFFSET)
#define KINETICS_SPI2_PUSHR        (KINETIS_SPI2_BASE+KINETICS_SPI_PUSHR_OFFSET)
#define KINETICS_SPI2_POPR         (KINETIS_SPI2_BASE+KINETICS_SPI_POPR_OFFSET)
#define KINETICS_SPI2_TXFR0        (KINETIS_SPI2_BASE+KINETICS_SPI_TXFR0_OFFSET)
#define KINETICS_SPI2_TXFR1        (KINETIS_SPI2_BASE+KINETICS_SPI_TXFR1_OFFSET)
#define KINETICS_SPI2_TXFR2        (KINETIS_SPI2_BASE+KINETICS_SPI_TXFR2_OFFSET)
#define KINETICS_SPI2_TXFR3        (KINETIS_SPI2_BASE+KINETICS_SPI_TXFR3_OFFSET)
#define KINETICS_SPI2_RXFR0        (KINETIS_SPI2_BASE+KINETICS_SPI_RXFR0_OFFSET)
#define KINETICS_SPI2_RXFR1        (KINETIS_SPI2_BASE+KINETICS_SPI_RXFR1_OFFSET)
#define KINETICS_SPI2_RXFR2        (KINETIS_SPI2_BASE+KINETICS_SPI_RXFR2_OFFSET)
#define KINETICS_SPI2_RXFR3        (KINETIS_SPI2_BASE+KINETICS_SPI_RXFR3_OFFSET)

/* Register Bit Definitions *****************************************************************/

/* DSPI Module Configuration Register */

#define SPI_MCR_HALT               (1 << 0)  /* Bit 0:  Halt */
                                             /* Bits 1-7: Reserved */
#define SPI_MCR_SMPL_PT_SHIFT      (8)       /* Bits 8-9: Sample Point */
#define SPI_MCR_SMPL_PT_MASK       (3 << SPI_MCR_SMPL_PT_SHIFT)
#  define SPI_MCR_SMPL_PT_0CLKS    (0 << SPI_MCR_SMPL_PT_SHIFT) /* 0 clocks between edge and sample */
#  define SPI_MCR_SMPL_PT_1CLKS    (1 << SPI_MCR_SMPL_PT_SHIFT) /* 1 clock between edge and sample */
#  define SPI_MCR_SMPL_PT_2CLKS    (2 << SPI_MCR_SMPL_PT_SHIFT) /* 2 clocks between edge and sample */
#define SPI_MCR_CLR_RXF            (1 << 10) /* Bit 10: Clear RX FIFO */
#define SPI_MCR_CLR_TXF            (1 << 11) /* Bit 11: Clear TX FIFO */
#define SPI_MCR_DIS_RXF            (1 << 12) /* Bit 12: Disable Receive FIFO */
#define SPI_MCR_DIS_TXF            (1 << 13) /* Bit 13: Disable Transmit FIFO */
#define SPI_MCR_MDIS               (1 << 14) /* Bit 14: Module Disable */
#define SPI_MCR_DOZE               (1 << 15) /* Bit 15: Doze Enable */
#define SPI_MCR_PCSIS_SHIFT        (16)      /* Bits 16-21: Peripheral Chip Select x Inactive State */
#define SPI_MCR_PCSIS_MASK         (0x3f << SPI_MCR_PCSIS_SHIFT)
#  define SPI_MCR_PCSIS_CS(n)      ((1 << (n)) << SPI_MCR_PCSIS_SHIFT)
                                             /* Bits 22–23: Reserved */
#define SPI_MCR_ROOE               (1 << 24) /* Bit 24: Receive FIFO Overflow Overwrite Enable */
#define SPI_MCR_PCSSE              (1 << 25) /* Bit 25: Peripheral Chip Select Strobe Enable */
#define SPI_MCR_MTFE               (1 << 26) /* Bit 26: Modified Timing Format Enable */
#define SPI_MCR_FRZ                (1 << 27) /* Bit 27: Freeze */
#define SPI_MCR_DCONF_SHIFT        (28)      /* Bits 28-29: DSPI Configuration */
#define SPI_MCR_DCONF_MASK         (3 << SPI_MCR_DCONF_SHIFT)
#  define SPI_MCR_DCONF_SPI        (0 << SPI_MCR_DCONF_SHIFT)
#define SPI_MCR_CONT_SCKE          (1 << 30) /* Bit 30: Continuous SCK Enable */
#define SPI_MCR_MSTR               (1 << 31) /* Bit 31: Master/Slave Mode Select */

/* DSPI Transfer Count Register */
                                             /* Bits 0-15: Reserved */
#define SPI_TCR_SPI_TCNT_SHIFT     (16)      /* Bits 16-31: SPI Transfer Counter */
#define SPI_TCR_SPI_TCNT_MASK      (0xffff << SPI_TCR_SPI_TCNT_SHIFT)

/* DSPI Clock and Transfer Attributes Register (Common Bits) */

#define SPI_CTAR_CPHA              (1 << 25) /* Bit 25: Clock Phase */
#define SPI_CTAR_CPOL              (1 << 26) /* Bit 26: Clock Polarity */

/* DSPI Clock and Transfer Attributes Register (Master Mode) */

#define SPI_CTARM_BR_SHIFT         (0)       /* Bits 0-3: Baud Rate Scaler */
#define SPI_CTARM_BR_MASK          (15 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_2           (0 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_4           (1 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_6           (2 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_8           (3 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_16          (4 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_32          (5 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_64          (6 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_128         (7 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_256         (8 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_512         (9 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_1024        (10 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_2048        (11 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_4096        (12 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_8192        (13 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_16384       (14 << SPI_CTARM_BR_SHIFT)
#  define SPI_CTARM_BR_32768       (15 << SPI_CTARM_BR_SHIFT)
#define SPI_CTARM_DT_SHIFT         (4)       /* Bits 4-7: Delay After Transfer Scaler */
#define SPI_CTARM_DT_MASK          (15 << SPI_CTARM_DT_SHIFT)
#define SPI_CTARM_ASC_SHIFT        (8)       /* Bits 8-11: After SCK Delay Scaler */
#define SPI_CTARM_ASC_MASK         (15 << SPI_CTARM_ASC_SHIFT)
#define SPI_CTARM_CSSCK_SHIFT      (12)      /* Bits 12-15: PCS to SCK Delay Scaler */
#define SPI_CTARM_CSSCK_MASK       (15 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_2        (0 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_4        (1 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_8        (2 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_16       (3 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_32       (4 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_64       (5 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_128      (6 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_256      (7 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_512      (8 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_1024     (9 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_2048     (10 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_4096     (11 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_8192     (12 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_16384    (13 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_32768    (14 << SPI_CTARM_CSSCK_SHIFT)
#  define SPI_CTARM_CSSCK_65536    (15 << SPI_CTARM_CSSCK_SHIFT)
#define SPI_CTARM_PBR_SHIFT        (16)      /* Bits 16-17: Baud Rate Prescaler */
#define SPI_CTARM_PBR_MASK         (3 << SPI_CTARM_PBR_SHIFT)
#  define SPI_CTARM_PBR_2          (0 << SPI_CTARM_PBR_SHIFT)
#  define SPI_CTARM_PBR_3          (1 << SPI_CTARM_PBR_SHIFT)
#  define SPI_CTARM_PBR_5          (2 << SPI_CTARM_PBR_SHIFT)
#  define SPI_CTARM_PBR_7          (3 << SPI_CTARM_PBR_SHIFT)
#define SPI_CTARM_PDT_SHIFT        (18)      /* Bits 18-19: Delay after Transfer Prescaler */
#define SPI_CTARM_PDT_MASK         (3 << SPI_CTARM_PDT_SHIFT)
#  define SPI_CTARM_PDT_1          (0 << SPI_CTARM_PDT_SHIFT)
#  define SPI_CTARM_PDT_3          (1 << SPI_CTARM_PDT_SHIFT)
#  define SPI_CTARM_PDT_5          (2 << SPI_CTARM_PDT_SHIFT)
#  define SPI_CTARM_PDT_7          (3 << SPI_CTARM_PDT_SHIFT)
#define SPI_CTARM_PASC_SHIFT       (20)      /* Bits 20-21: After SCK Delay Prescaler */
#define SPI_CTARM_PASC_MASK        (3 << SPI_CTARM_PASC_SHIFT)
#  define SPI_CTARM_PASC_1         (0 << SPI_CTARM_PASC_SHIFT)
#  define SPI_CTARM_PASC_3         (1 << SPI_CTARM_PASC_SHIFT)
#  define SPI_CTARM_PASC_5         (2 << SPI_CTARM_PASC_SHIFT)
#  define SPI_CTARM_PASC_7         (3 << SPI_CTARM_PASC_SHIFT)
#define SPI_CTARM_PCSSCK_SHIFT     (22)      /* Bits 22-23: PCS to SCK Delay Prescaler */
#define SPI_CTARM_PCSSCK_MASK      (3 << SPI_CTARM_PCSSCK_SHIFT)
#  define SPI_CTARM_PCSSCK_1       (0 << SPI_CTARM_PCSSCK_SHIFT)
#  define SPI_CTARM_PCSSCK_3       (1 << SPI_CTARM_PCSSCK_SHIFT)
#  define SPI_CTARM_PCSSCK_5       (2 << SPI_CTARM_PCSSCK_SHIFT)
#  define SPI_CTARM_PCSSCK_7       (3 << SPI_CTARM_PCSSCK_SHIFT)
#define SPI_CTARM_LSBFE            (1 << 24) /* Bit 24: LBS First */
                                             /* Bits 25-26:  See common bits above */
#define SPI_CTARM_FMSZ_SHIFT       (27)      /* Bits 27-30: Frame Size */
#define SPI_CTARM_FMSZ_MASK        (15 << SPI_CTARM_FMSZ_SHIFT)
#define SPI_CTARM_DBR              (1 << 31) /* Bit 31:  Double Baud Rate */

/* DSPI Clock and Transfer Attributes Register (Slave Mode) */
                                             /* Bits 0-24: Reserved */
                                             /* Bits 25-26:  See common bits above */
#define SPI_CTARS_FMSZ_SHIFT       (27)      /* Bits 27-31: Frame Size */
#define SPI_CTARS_FMSZ_MASK        (31 << SPI_CTARS_FMSZ_SHIFT)

/* DSPI Status Register */

#define SPI_SR_POPNXTPTR_SHIFT     (0)       /* Bits 0-3: Pop Next Pointer */
#define SPI_SR_POPNXTPTR_MASK      (15 << SPI_SR_POPNXTPTR_SHIFT)
#define SPI_SR_RXCTR_SHIFT         (4)       /* Bits 4-7: RX FIFO Counter */
#define SPI_SR_RXCTR_MASK          (15 << SPI_SR_RXCTR_SHIFT)
#define SPI_SR_TXNXTPTR_SHIFT      (8)       /* Bits 8-11: Transmit Next Pointer */
#define SPI_SR_TXNXTPTR_MASK       (15 << SPI_SR_TXNXTPTR_SHIFT)
#define SPI_SR_TXCTR_SHIFT         (12)      /* Bits 12-15: TX FIFO Counter */
#define SPI_SR_TXCTR_MASK          (15 << SPI_SR_TXCTR_SHIFT)
                                             /* Bit 16: Reserved */
#define SPI_SR_RFDF                (1 << 17) /* Bit 17: Receive FIFO Drain Flag */
                                             /* Bit 18: Reserved */
#define SPI_SR_RFOF                (1 << 19) /* Bit 19: Receive FIFO Overflow Flag */
                                             /* Bit 20-24: Reserved */
#define SPI_SR_TFFF                (1 << 25) /* Bit 25: Transmit FIFO Fill Flag */
                                             /* Bit 26: Reserved */
#define SPI_SR_TFUF                (1 << 27) /* Bit 27: Transmit FIFO Underflow Flag */
#define SPI_SR_EOQF                (1 << 28) /* Bit 28: End of Queue Flag */
                                             /* Bit 29: Reserved */
#define SPI_SR_TXRXS               (1 << 30) /* Bit 30: TX and RX Status */
#define SPI_SR_TCF                 (1 << 31) /* Bit 31: Transfer Complete Flag */

/* DSPI DMA/Interrupt Request Select and Enable Register */
                                             /* Bits 0-15: Reserved */
#define SPI_RSER_RFDF_DIRS         (1 << 16) /* Bit 16: Receive FIFO Drain DMA or Interrupt Request Select */
#define SPI_RSER_RFDF_RE           (1 << 17) /* Bit 17: Receive FIFO Drain Request Enable */
                                             /* Bit 18: Reserved */
#define SPI_RSER_RFOF_RE           (1 << 19) /* Bit 19: Receive FIFO Overflow Request Enable */
                                             /* Bit 20-23: Reserved */
#define SPI_RSER_TFFF_DIRS         (1 << 24) /* Bit 24: Transmit FIFO Fill DMA or Interrupt Request Select */
#define SPI_RSER_TFFF_RE           (1 << 25) /* Bit 25: Transmit FIFO Fill Request Enable */
                                             /* Bit 26: Reserved */
#define SPI_RSER_TFUF_RE           (1 << 27) /* Bit 27: Transmit FIFO Underflow Request Enable */
#define SPI_RSER_EOQF_RE           (1 << 28) /* Bit 28: DSPI Finished Request Enable */
                                             /* Bits 29-30: Reserved */
#define SPI_RSER_TCF_RE            (1 << 31) /* Bit 31: Transmission Complete Request Enable */

/* DSPI PUSH TX FIFO Register (Master Mode)*/

#define SPI_PUSHR_TXDATA_SHIFT     (0)       /* Bits 0-15: Transmit Data */
#define SPI_PUSHR_TXDATA_MASK      (0xffff << SPI_PUSHR_TXDATA_SHIFT)
#define SPI_PUSHR_PCS_SHIFT        (16)      /* Bits 16-21: Select PCS signals to assert */
#define SPI_PUSHR_PCS_MASK         (0x3f << SPI_PUSHR_PCS_SHIFT)
#  define SPI_PUSHR_PCS(n)         ((1 << (n)) << SPI_PUSHR_PCS_SHIFT)
                                             /* Bit 22-25: Reserved */
#define SPI_PUSHR_CTCNT            (1 << 26) /* Bit 26: Clear Transfer Counter */
#define SPI_PUSHR_EOQ              (1 << 27) /* Bit 27: End Of Queue */
#define SPI_PUSHR_CTAS_SHIFT       (28)      /* Bits 28-30: Clock and Transfer Attributes Select */
#define SPI_PUSHR_CTAS_MASK        (7 << SPI_PUSHR_CTAS_SHIFT)
#  define SPI_PUSHR_CTAS_CTAR0     (0 << SPI_PUSHR_CTAS_SHIFT)
#  define SPI_PUSHR_CTAS_CTAR1     (1 << SPI_PUSHR_CTAS_SHIFT)
#define SPI_PUSHR_CONT             (1 << 31) /* Bit 31: Continuous Peripheral Chip Select Enable */

/* DSPI PUSH TX FIFO Register (Slave Mode, 32-bits of RXDATA)*/

/* DSPI POP RX FIFO Register (32-bits of RXDATA) */

/* DSPI Transmit FIFO Registers */

#define SPI_TXFR_TXDATA_SHIFT      (0)       /* Bits 0-15: Transmit Data */
#define SPI_TXFR_TXDATA_MASK       (0xffff << SPI_TXFR_TXDATA_SHIFT)
#define SPI_TXFR_TXCDATA_SHIFT     (16)      /* Bits 16-31: Transmit Command or Transmit Data */
#define SPI_TXFR_TXCDATA_MASK      (0xffff << SPI_TXFR_TXCDATA_SHIFT)

/* DSPI Receive FIFO Registers (32-bits of RXDATA) */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_DSPI_H */
