/************************************************************************************
 * arch/hc/src/m9s12/m9s12_spi.h (v3)
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_HC_SRC_M9S12_M9S12_SPI_H
#define __ARCH_ARM_HC_SRC_M9S12_M9S12_SPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define HCS12_SPI_CR1_OFFSET      0x00 /* SPI Control Register 1 */
#define HCS12_SPI_CR2_OFFSET      0x01 /* SPI Control Register 2 */
#define HCS12_SPI_BR_OFFSET       0x02 /* SPI Baud Rate Register */
#define HCS12_SPI_SR_OFFSET       0x03 /* SPI Status Register */       
#define HCS12_SPI_DR_OFFSET       0x05 /* SPI Data Register */

/* Register Addresses ***************************************************************/

#define HCS12_SPI_CR1             (HCS12_REG_BASE+HCS12_SPI_BASE+HCS12_SPI_CR1_OFFSET)
#define HCS12_SPI_CR2             (HCS12_REG_BASE+HCS12_SPI_BASE+HCS12_SPI_CR2_OFFSET)
#define HCS12_SPI_BR              (HCS12_REG_BASE+HCS12_SPI_BASE+HCS12_SPI_BR_OFFSET)
#define HCS12_SPI_SR              (HCS12_REG_BASE+HCS12_SPI_BASE+HCS12_SPI_SR_OFFSET)
#define HCS12_SPI_DR              (HCS12_REG_BASE+HCS12_SPI_BASE+HCS12_SPI_DR_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* SPI Control Register 1 Bit-Field Definitions */

#define SPI_CR1_LSBFE             (1 << 0)  /* Bit 0:  LSB-First Enable */
#define SPI_CR1_SSOE              (1 << 1)  /* Bit 1:  Slave Select Output Enable */
#define SPI_CR1_CPHA              (1 << 2)  /* Bit 2:  SPI Clock Phase */
#define SPI_CR1_CPOL              (1 << 3)  /* Bit 3:  SPI Clock Polarity */
#define SPI_CR1_MSTR              (1 << 4)  /* Bit 4:  SPI Master/Slave Mode Select */
#define SPI_CR1_SPTIE             (1 << 5)  /* Bit 5:  SPI Transmit Interrupt Enable */
#define SPI_CR1_SPE               (1 << 6)  /* Bit 6:  SPI System Enable */
#define SPI_CR1_SPIE              (1 << 7)  /* Bit 7:  SPI Interrupt Enable */

/* SPI Control Register 2 Bit-Field Definitions */

#define SPI_CR2_SPC0              (1 << 0)  /* Bit 0:   Serial Pin Control Bit 0  */
#define SPI_CR2_SPISWAI           (1 << 1)  /* Bit 1:  SPI Stop in Wait Mode */
#define SPI_CR2_BIDIROE           (1 << 3)  /* Bit 3:  Output Enable in the Bidirectional */
#define SPI_CR2_MODFEN            (1 << 4)  /* Bit 4:  Mode Fault Enable */

/* SPI Baud Rate Register Bit-Field Definitions */

#define SPI_BR_SPR_SHIFT          (0)      /* Bits 0-2: SPI Baud Rate Selection */
#define SPI_BR_SPR_MASK           (7 << SPI_BR_SPR_SHIFT)
#define SPI_BR_SPPR_SHIFT         (4)      /* Bits 4-6: SPI Baud Rate Preselection */
#define SPI_BR_SPPR_MASK          (7 << SPI_BR_SPPR_SHIFT)

/* SPI Status Register Bit-Field Definitions */  

#define SPI_SR_MODF               (1 << 4)  /* Bit 4:  Mode Fault */
#define SPI_SR_SPTEF              (1 << 5)  /* Bit 5:  SPI Transmit Empty Interrupt */
#define SPI_SR_SPIF               (1 << 7)  /* Bit 7:  SPIF Interrupt */

/* SPI Data Register Bit-Field Definitions */
/* 8-bit SPI data register */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_M9S12_M9S12_SPI_H */
