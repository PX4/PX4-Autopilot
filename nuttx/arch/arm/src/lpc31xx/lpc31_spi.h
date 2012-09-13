/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_spi.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_SPI_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_SPI_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* SPI register base address offset into the APB2 domain ****************************************/

#define LPC31_SPI_VBASE                (LPC31_APB2_VSECTION+LPC31_APB2_SPI_OFFSET)
#define LPC31_SPI_PBASE                (LPC31_APB2_PSECTION+LPC31_APB2_SPI_OFFSET)

/* SPI register offsets (with respect to the SPI base) ******************************************/
/* SPI configuration registers */

#define LPC31_SPI_CONFIG_OFFSET        0x000 /* Configuration register */
#define LPC31_SPI_SLVENABLE_OFFSET     0x004 /* Slave enable register */
#define LPC31_SPI_TXFIFO_OFFSET        0x008 /* Transmit FIFO flush register */
#define LPC31_SPI_FIFODATA_OFFSET      0x00C /* FIFO data register */
#define LPC31_SPI_NHPPOP_OFFSET        0x010 /* NHP pop register */
#define LPC31_SPI_NHPMODE_OFFSET       0x014 /* NHP mode selection register */
#define LPC31_SPI_DMA_OFFSET           0x018 /* DMA settings register */
#define LPC31_SPI_STATUS_OFFSET        0x01c /* Status register */
#define LPC31_SPI_HWINFO_OFFSET        0x020 /* Hardware information register */

/* SPI slave registers */

#define LPC31_SPI_SLV0_1_OFFSET        0x024 /* Slave settings register 1 (for slave 0) */
#define LPC31_SPI_SLV0_2_OFFSET        0x028 /* Slave settings register 2 (for slave 0) */
#define LPC31_SPI_SLV1_1_OFFSET        0x02c /* Slave settings register 1 (for slave 1) */
#define LPC31_SPI_SLV1_2_OFFSET        0x030 /* Slave settings register 2 (for slave 1) */
#define LPC31_SPI_SLV2_1_OFFSET        0x034 /* Slave settings register 1 (for slave 2) */
#define LPC31_SPI_SLV2_2_OFFSET        0x038 /* Slave settings register 2 (for slave 2) */
                                               /* 0x03c-0xfd0: Reserved */
/* SPI interrupt registers */

#define LPC31_SPI_INTTHR_OFFSET        0xfd4 /* Tx/Rx threshold interrupt levels */
#define LPC31_SPI_INTCLRENABLE_OFFSET  0xfd8 /* INT_ENABLE bits clear register */
#define LPC31_SPI_INTSETENABLE_OFFSET  0xfdc /* INT_ENABLE bits set register */
#define LPC31_SPI_INTSTATUS_OFFSET     0xfe0 /* Interrupt status register */
#define LPC31_SPI_INTENABLE_OFFSET     0xfe4 /* Interrupt enable register */
#define LPC31_SPI_INTCLRSTATUS_OFFSET  0xfe8 /* INT_STATUS bits clear register */
#define LPC31_SPI_INTSETSTATUS_OFFSET  0xfec /* INT_STATUS bits set register */
                                               /* 0xff0-0xff8: Reserved */

/* SPI register (virtual) addresses *************************************************************/

/* SPI configuration registers */

#define LPC31_SPI_CONFIG               (LPC31_SPI_VBASE+LPC31_SPI_CONFIG_OFFSET)
#define LPC31_SPI_SLVENABLE            (LPC31_SPI_VBASE+LPC31_SPI_SLVENABLE_OFFSET)
#define LPC31_SPI_TXFIFO               (LPC31_SPI_VBASE+LPC31_SPI_TXFIFO_OFFSET)
#define LPC31_SPI_FIFODATA             (LPC31_SPI_VBASE+LPC31_SPI_FIFODATA_OFFSET)
#define LPC31_SPI_NHPPOP               (LPC31_SPI_VBASE+LPC31_SPI_NHPPOP_OFFSET)
#define LPC31_SPI_NHPMODE              (LPC31_SPI_VBASE+LPC31_SPI_NHPMODE_OFFSET)
#define LPC31_SPI_DMA                  (LPC31_SPI_VBASE+LPC31_SPI_DMA_OFFSET)
#define LPC31_SPI_STATUS               (LPC31_SPI_VBASE+LPC31_SPI_STATUS_OFFSET)
#define LPC31_SPI_HWINFO               (LPC31_SPI_VBASE+LPC31_SPI_HWINFO_OFFSET)

/* SPI slave registers */

#define LPC31_SPI_SLV0_1               (LPC31_SPI_VBASE+LPC31_SPI_SLV0_1_OFFSET)
#define LPC31_SPI_SLV0_2               (LPC31_SPI_VBASE+LPC31_SPI_SLV0_2_OFFSET)
#define LPC31_SPI_SLV1_1               (LPC31_SPI_VBASE+LPC31_SPI_SLV1_1_OFFSET)
#define LPC31_SPI_SLV1_2               (LPC31_SPI_VBASE+LPC31_SPI_SLV1_2_OFFSET)
#define LPC31_SPI_SLV2_1               (LPC31_SPI_VBASE+LPC31_SPI_SLV2_1_OFFSET)
#define LPC31_SPI_SLV2_2               (LPC31_SPI_VBASE+LPC31_SPI_SLV2_2_OFFSET)

/* SPI interrupt registers */

#define LPC31_SPI_INTTHR               (LPC31_SPI_VBASE+LPC31_SPI_INTTHR_OFFSET)
#define LPC31_SPI_INTCLRENABLE         (LPC31_SPI_VBASE+LPC31_SPI_INTCLRENABLE_OFFSET)
#define LPC31_SPI_INTSETENABLE         (LPC31_SPI_VBASE+LPC31_SPI_INTSETENABLE_OFFSET)
#define LPC31_SPI_INTSTATUS            (LPC31_SPI_VBASE+LPC31_SPI_INTSTATUS_OFFSET)
#define LPC31_SPI_INTENABLE            (LPC31_SPI_VBASE+LPC31_SPI_INTENABLE_OFFSET)
#define LPC31_SPI_INTCLRSTATUS         (LPC31_SPI_VBASE+LPC31_SPI_INTCLRSTATUS_OFFSET)
#define LPC31_SPI_INTSETSTATUS         (LPC31_SPI_VBASE+LPC31_SPI_INTSETSTATUS_OFFSET)

/* SPI register bit definitions *****************************************************************/
/* SPI Configuration register CONFIG, address 0x15002000 */

#define SPI_CONFIG_INTERSLVDELAY_SHIFT   (16)      /* Bits 16-31: Delay between xfrs to different slaves  */
#define SPI_CONFIG_NTERSLVDELAY_MASK     (0xffff << SPI_CONFIG_INTERSLVDELAY_SHIFT)
#define SPI_CONFIG_UPDENABLE             (1 << 7)  /* Bit 7:  7 Update enable bit */
#define SPI_CONFIG_SOFTRST               (1 << 6)  /* Bit 6:  6 Software reset bit */
#define SPI_CONFIG_SLVDISABLE            (1 << 4)  /* Bit 4:  4 Slave output disable (slave mode) */
#define SPI_CONFIG_XMITMODE              (1 << 3)  /* Bit 3:  3 Transmit mode */
#define SPI_CONFIG_LOOPBACK              (1 << 2)  /* Bit 2:  2 Loopback mode bit */
#define SPI_CONFIG_MS                    (1 << 1)  /* Bit 1:  1 Master/slave mode bit */
#define SPI_CONFIG_SPIENABLE             (1 << 0)  /* Bit 0:  0 SPI enable bit */

/* Slave Enable register SLVENABLE, address 0x15002004 */

#define SPI_SLVENABLE3_SHIFT             (4)       /* Bits 4-5: Slave 3 enable bits */
#define SPI_SLVENABLE3_MASK              (3 << SPI_SLVENABLE3_SHIFT)
#  define SPI_SLVENABLE3_DISABLED        (0 << SPI_SLVENABLE3_SHIFT) /* Disabled */
#  define SPI_SLVENABLE3_ENABLED         (1 << SPI_SLVENABLE3_SHIFT) /* Enabled */
#  define SPI_SLVENABLE3_SUSPENDED       (3 << SPI_SLVENABLE3_SHIFT) /* Suspended */
#define SPI_SLVENABLE2_SHIFT             (2)       /* Bits 2-3: Slave 2 enable bits */
#define SPI_SLVENABLE2_MASK              (3 << SPI_SLVENABLE2_SHIFT)
#  define SPI_SLVENABLE2_DISABLED        (0 << SPI_SLVENABLE2_SHIFT) /* Disabled */
#  define SPI_SLVENABLE2_ENABLED         (1 << SPI_SLVENABLE2_SHIFT) /* Enabled */
#  define SPI_SLVENABLE2_SUSPENDED       (3 << SPI_SLVENABLE2_SHIFT) /* Suspended */
#define SPI_SLVENABLE1_SHIFT             (0)       /* Bits 0-1: Slave 1 enable bits */
#define SPI_SLVENABLE1_MASK              (3 << SPI_SLVENABLE1_SHIFT)
#  define SPI_SLVENABLE1_DISABLED        (0 << SPI_SLVENABLE1_SHIFT) /* Disabled */
#  define SPI_SLVENABLE1_ENABLED         (1 << SPI_SLVENABLE1_SHIFT) /* Enabled */
#  define SPI_SLVENABLE1_SUSPENDED       (3 << SPI_SLVENABLE1_SHIFT) /* Suspended */

/* Transmit FIFO flush register TXFIFO, address 0x15002008 */

#define SPI_TXFIFO_FLUSH                 (1 << 0)  /* Bit 0:  Transmit FIFO flush bit */

/* FIFO data register FIFODATA, address 0x1500200c */

#define SPI_FIFODATA_SHIFT               (0)       /* Bits 0-15: FIFO data */
#define SPI_FIFODATA_MASK                (0xffff << SPI_FIFODATA_SHIFT)

/* NHP POP register NHPPOP, address 0x15002010 */

#define SPI_NHPPOP                       (1 << 0)  /* Bit 0:  NHP pop bit */

/* NHP mode register NHPMODE, address 0x15002014 */

#define SPI_NHPMODE                      (1 << 0)  /* Bit 0:  NHP mode bit */

/* DMA setting register DMA, address 0x15002018 */

#define SPI_DMA_TXENABLE                 (1 << 1)  /* Bit 1:  Tx DMA enable bit */
#define SPI_DMA_RXEMABLE                 (1 << 0)  /* Bit 0:  Rx DMA enable bit */

/* Status register STATUS, address 0x1500201c */

#define SPI_STATUS_SMSBUSY               (1 << 5)  /* Bit 5:  Sequential multi-slave mode busy */
#define SPI_STATUS_BUSY                  (1 << 4)  /* Bit 4:  SPI busy flag */
#define SPI_STATUS_RXFIFOFULL            (1 << 3)  /* Bit 3:  Receive FIFO full bit */
#define SPI_STATUS_RXFIFOEMPTY           (1 << 2)  /* Bit 2:  Receive FIFO empty bit */
#define SPI_STATUS_TXFIFOFULL            (1 << 1)  /* Bit 1:  Transmit FIFO full bit */
#define SPI_STATUS_TXFIFOEMPTY           (1 << 0)  /* Bit 0:  Transmit FIFO empty bit */

/* Hardware information register HWINFO, address 0x15002020 */

#define SPI_HWINFO_FIFOIMPLT             (1 << 30) /* Bit 30:  FIFO memory implementation */
#define SPI_HWINFO_NSLAVES_SHIFT         (26)      /* Bits 26-29: Maximum number of slaves (minus 1) */
#define SPI_HWINFO_NSLAVES_MASK          (0x0f << SPI_HWINFO_NSLAVES_SHIFT)
#define SPI_HWINFO_TXFIFOWIDTH_SHIFT     (21)      /* Bits 21-25: Width transmit FIFO (minus 1) */
#define SPI_HWINFO_TXFIFOWIDTH_MASK      (0x1f << SPI_HWINFO_TXFIFOWIDTH_SHIFT)
#define SPI_HWINFO_RXFIFOWIDTH_SHIFT     (16)      /* Bits 16-20: Width receive FIFO (minus 1) */
#define SPI_HWINFO_RXFIFOWIDTH_MASK      (0x1f << SPI_HWINFO_RXFIFOWIDTH_SHIFT)
#define SPI_HWINFO_TXFIFODEPTH_SHIFT     (8)       /* Bits 8-15:  64 */
#define SPI_HWINFO_TXFIFODEPTH_MASK      (0x0ff << SPI_HWINFO_TXFIFODEPTH_SHIFT)
#define SPI_HWINFO_RXFIFODEPTH_SHIFT     (0)       /* Bits 0-7:   64 */
#define SPI_HWINFO_RXFIFODEPTH_MASK      (0xff << SPI_HWINFO_RXFIFODEPTH_SHIFT)

/* Slave settings 1 SLV0-2_1, addresses 0x15002024, 0x1500202c, and 0x15002034 */

#define SPI_SLV_1_INTERXFRDLY_SHIFT      (24)      /* Bits 24-31: Delay between slave xfrs (master mode) */
#define SPI_SLV_1_INTERXFRDLY_MASK       (0xff << SPI_SLV_1_INTERXFRDLY_SHIFT)
#define SPI_SLV_1_NWORDS_SHIFT           (16)      /* Bits 16-23: Number words to send in SMS mode (master mode) */
#define SPI_SLV_1_NWORDS_MASK            (0xff << SPI_SLV_1_NWORDS_SHIFT)
#define SPI_SLV_1_CLKDIV2_SHIFT          (8)       /* Bits 8-15: Serial clock divisor 2 */
#define SPI_SLV_1_CLKDIV2_MASK           (0xff << SPI_SLV_1_CLKDIV2_SHIFT)
#define SPI_SLV_1_CLKDIV1_SHIFT          (0)       /* Bits 0-7: Serial clock rate divisor 1 */
#define SPI_SLV_1_CLKDIV1_MASK           (0xff << SPI_SLV_1_CLKDIV1_SHIFT)

/* Slave settings 2 SLV0-2_2, addresses 0x15002028, 0x15002030, and0x15002038 */

#define SPI_SLV_2_DELAY_SHIFT            (9)       /* Bits 9-16: Programmable delay */
#define SPI_SLV_2_DELAY_MASK             (0xff << SPI_SLV_2_DELAY_SHIFT)
#define SPI_SLV_2_CSVAL                  (1 << 8)  /* Bit 8:  Chip select value between transfers */
#define SPI_SLV_2_XFRFMT                 (1 << 7)  /* Bit 7:  Format of transfer */
#define SPI_SLV_2_SPO                    (1 << 6)  /* Bit 6:  Serial clock polarity */
#define SPI_SLV_2_SPH                    (1 << 5)  /* Bit 5:  Serial clock phase */
#define SPI_SLV_2_WDSIZE_SHIFT           (0)       /* Bits 0-4: Word size of transfers slave (minus 1) */
#define SPI_SLV_2_WDSIZE_MASK            (0x1f << SPI_SLV_2_WDSIZE_SHIFT)

/* Interrupt threshold register INTTHR, address 0x15002fd4 */

#define SPI_INTTHR_TX_SHIFT              (8)       /* Bits 8-15: Interrupt when less than this in TX FIFO */
#define SPI_INTTHR_TX_MASK               (0xff << SPI_INTTHR_TX_SHIFT)
#define SPI_INTTHR_RX_SHIFT              (0)       /* Bits 0-7: Interrupt when more than this in RX FIFO */
#define SPI_INTTHR_RX_MASK               (0xff << SPI_INTTHR_TX_SHIFT)

/* Interrupt clear enable register INTCLRENABLE, address 0x15002fd8,
 * Interrupt set enable register   INTSETENABLE, address 0x15002fdc,
 * Interrupt status register       INTSTATUS,    address 0x15002fe0,
 * Interrupt enable register       INTENABLE,    address 0x15002fe4,
 * Interrupt clear status register INTCLRSTATUS, address 0x15002fe8,
 * Interrupt set status register   INTSETSTATUS, address 0x15002fec
 */
 
#define SPI_INT_SMS                      (1 << 4)  /* Bit 4:  Sequential multi-slave mode ready interrupt bit */
#define SPI_INT_TX                       (1 << 3)  /* Bit 3:  Transmit threshold level interrupt bit */
#define SPI_INT_RX                       (1 << 2)  /* Bit 3:  Receive threshold level interrupt bit */
#define SPI_INT_TO                       (1 << 1)  /* Bit 1:  Receive timeout interrupt bit */
#define SPI_INT_OV                       (1 << 0)  /* Bit 0:  Receive overrtun interrrupt bit */
                          
/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_SPI_H */
