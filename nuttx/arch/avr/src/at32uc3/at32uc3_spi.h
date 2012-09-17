/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_spi.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_SPI_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_SPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AVR32_SPI_CR_OFFSET       0x000 /* Control Register */
#define AVR32_SPI_MR_OFFSET       0x004 /* Mode Register */
#define AVR32_SPI_RDR_OFFSET      0x008 /* Receive Data Register */
#define AVR32_SPI_TDR_OFFSET      0x00c /* Transmit Data Register */
#define AVR32_SPI_SR_OFFSET       0x010 /* Status Register */
#define AVR32_SPI_IER_OFFSET      0x014 /* Interrupt Enable Register */
#define AVR32_SPI_IDR_OFFSET      0x018 /* Interrupt Disable Register */
#define AVR32_SPI_IMR_OFFSET      0x01c /* Interrupt Mask Register */
#define AVR32_SPI_CSR0_OFFSET     0x030 /* Chip Select Register 0 */
#define AVR32_SPI_CSR1_OFFSET     0x034 /* Chip Select Register 1 */
#define AVR32_SPI_CSR2_OFFSET     0x038 /* Chip Select Register 2 */
#define AVR32_SPI_CSR3_OFFSET     0x03c /* Chip Select Register 3 */
#define AVR32_SPI_VERSION_OFFSET  0x0fc /* Version Register */

/* Register Addresses ***************************************************************/

#define AVR32_SPI0_CR             (AVR32_SPI0_BASE+AVR32_SPI_CR_OFFSET)
#define AVR32_SPI0_MR             (AVR32_SPI0_BASE+AVR32_SPI_MR_OFFSET)
#define AVR32_SPI0_RDR            (AVR32_SPI0_BASE+AVR32_SPI_RDR_OFFSET)
#define AVR32_SPI0_TDR            (AVR32_SPI0_BASE+AVR32_SPI_TDR_OFFSET)
#define AVR32_SPI0_SR             (AVR32_SPI0_BASE+AVR32_SPI_SR_OFFSET)
#define AVR32_SPI0_IER            (AVR32_SPI0_BASE+AVR32_SPI_IER_OFFSET)
#define AVR32_SPI0_IDR            (AVR32_SPI0_BASE+AVR32_SPI_IDR_OFFSET)
#define AVR32_SPI0_IMR            (AVR32_SPI0_BASE+AVR32_SPI_IMR_OFFSET)
#define AVR32_SPI0_CSR0           (AVR32_SPI0_BASE+AVR32_SPI_CSR0_OFFSET)
#define AVR32_SPI0_CSR1           (AVR32_SPI0_BASE+AVR32_SPI_CSR1_OFFSET)
#define AVR32_SPI0_CSR2           (AVR32_SPI0_BASE+AVR32_SPI_CSR2_OFFSET)
#define AVR32_SPI0_CSR3           (AVR32_SPI0_BASE+AVR32_SPI_CSR3_OFFSET)
#define AVR32_SPI0_VERSION        (AVR32_SPI0_BASE+AVR32_SPI_VERSION_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Control Register */

#define SPI_CR_SPIEN              (1 << 0)  /* Bit 0:  SPI Enable */
#define SPI_CR_SPIDIS             (1 << 1)  /* Bit 1:  SPI Disable */
#define SPI_CR_SWRST              (1 << 7)  /* Bit 7:  SPI Software Reset */
#define SPI_CR_LASTXFER           (1 << 24) /* Bit 24: Last Transfer */

/* Mode Register */


#define SPI_MR_MSTR               (1 << 0)  /* Bit 0:  Master/Slave Mode */
#define SPI_MR_PS                 (1 << 1)  /* Bit 1:  Peripheral Select */
#define SPI_MR_PCSDEC             (1 << 2)  /* Bit 2:  Chip Select Decode */
#define SPI_MR_MODFDIS            (1 << 4)  /* Bit 4:  Mode Fault Detection */
#define SPI_MR_LLB                (1 << 7)  /* Bit 7:  Local Loopback Enable */
#define SPI_MR_PCS_SHIFT          (16)      /* Bits 16-19: Peripheral Chip Select */
#define SPI_MR_PCS_MASK           (15 << SPI_MR_PCS_SHIFT)
#define SPI_MR_DLYBCS_SHIFT       (24)      /* Bits 24-31: Delay Between Chip Selects */
#define SPI_MR_DLYBCS:_MASK       (0xff << SPI_MR_DLYBCS_SHIFT)

/* Receive Data Register */

#define SPI_RDR_MASK              (0xffff)

/* Transmit Data Register */

#define SPI_TDR_MASK              (0xffff)

/* Status Register */
/* Interrupt Enable Register */
/* Interrupt Disable Register */
/* Interrupt Mask Register */

#define SPI_INT_DRF               (1 << 0)  /* Bit 0:  Receive Data Register Full */
#define SPI_INT_TDRE              (1 << 1)  /* Bit 1:  Transmit Data Register Empty */
#define SPI_INT_MODF              (1 << 2)  /* Bit 2:  Mode Fault Error */
#define SPI_INT_OVRES             (1 << 3)  /* Bit 3:  Overrun Error Status */
#define SPI_INT_NSSR              (1 << 8)  /* Bit 8:  NSS Rising */
#define SPI_INT_TXEMPTY           (1 << 9)  /* Bit 9:  Transmission Registers Empty */
#define SPI_SR_SPIENS             (1 << 16) /* Bit 16: SPI Enable Status (SR only) */

/* Chip Select Register 0-3 */

#define SPI_CSR_CPOL              (1 << 0)  /* Bit 0:  Clock Polarity */
#define SPI_CSR_NCPHA             (1 << 1)  /* Bit 1:  Clock Phase */
#define SPI_CSR_CSNAAT            (1 << 2)  /* Bit 2:  Chip Select Not Active After Transfer */
#define SPI_CSR_CSAAT             (1 << 3)  /* Bit 3:  Chip Select Active After Transfer */
#define SPI_CSR_BITS_SHIFT        (4)       /* Bits 4-7: Bits Per Transfer */
#define SPI_CSR_BITS_MASK         (15 << SPI_CSR_BITS_SHIFT)
#  define SPI_CSR_BITS(n)         (((n)-8) << SPI_CSR_BITS_SHIFT)
#  define SPI_CSR_8BITS           (0 << SPI_CSR_BITS_SHIFT)
#  define SPI_CSR_9BITS           (1 << SPI_CSR_BITS_SHIFT)
#  define SPI_CSR_10BITS          (2 << SPI_CSR_BITS_SHIFT)
#  define SPI_CSR_11BITS          (3 << SPI_CSR_BITS_SHIFT)
#  define SPI_CSR_12BITS          (4 << SPI_CSR_BITS_SHIFT)
#  define SPI_CSR_13BITS          (5 << SPI_CSR_BITS_SHIFT)
#  define SPI_CSR_14BITS          (6 << SPI_CSR_BITS_SHIFT)
#  define SPI_CSR_15BITS          (7 << SPI_CSR_BITS_SHIFT)
#  define SPI_CSR_16BITS          (8 << SPI_CSR_BITS_SHIFT)
#define SPI_CSR_SCBR_SHIFT        (8)     /* Bits 8-15: Serial Clock Baud Rate */
#define SPI_CSR_SCBR_MASK         (0xff << SPI_CSR_SCBR_SHIFT)
#define SPI_CSR_DLYBS_SHIFT       (16)     /* Bits 16-23: Delay Before SPCK */
#define SPI_CSR_DLYBS_MASK        (0xff << SPI_CSR_DLYBS_SHIFT)
#define SPI_CSR_DLYBCT_SHIFT      (24)     /* Bits 24-31: Delay Between Consecutive Transfers */
#define SPI_CSR_DLYBCT_MASK       (0xff << SPI_CSR_DLYBCT_SHIFT)

/* Version Register (Values in the Version Register vary with the version of the IP
 * block implementation.)
 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_SPI_H */

