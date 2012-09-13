/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_abdac.h
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_ABDAC_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_ABDAC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AVR32_ABDAC_SDR_OFFSET    0x00 /* Sample Data Register */
#define AVR32_ABDAC_CR_OFFSET     0x08 /* Control Register */
#define AVR32_ABDAC_IMR_OFFSET    0x0c /* Interrupt Mask Register */
#define AVR32_ABDAC_IER_OFFSET    0x10 /* Interrupt Enable Register */
#define AVR32_ABDAC_IDR_OFFSET    0x14 /* Interrupt Disable Register */
#define AVR32_ABDAC_ICR_OFFSET    0x18 /* Interrupt Clear Register */
#define AVR32_ABDAC_ISR_OFFSET    0x1c /* Interrupt Status Register */

/* Register Addresses ***************************************************************/

#define AVR32_ABDAC_SDR           (AVR32_ABDAC_BASE+AVR32_ABDAC_SDR_OFFSET)
#define AVR32_ABDAC_CR            (AVR32_ABDAC_BASE+AVR32_ABDAC_CR_OFFSET)
#define AVR32_ABDAC_IMR           (AVR32_ABDAC_BASE+AVR32_ABDAC_IMR_OFFSET)
#define AVR32_ABDAC_IER           (AVR32_ABDAC_BASE+AVR32_ABDAC_IER_OFFSET)
#define AVR32_ABDAC_IDR           (AVR32_ABDAC_BASE+AVR32_ABDAC_IDR_OFFSET)
#define AVR32_ABDAC_ICR           (AVR32_ABDAC_BASE+AVR32_ABDAC_ICR_OFFSET)
#define AVR32_ABDAC_ISR           (AVR32_ABDAC_BASE+AVR32_ABDAC_ISR_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Sample Data Register Bit-field Definitions */
/* This register contains a 32-bit data and, hence, has no bit-fiels */

/* Control Register Bit-field Definitions */

#define ABDAC_CR_SWAP             (1 << 30) /* Bit 30: Swap Channels */
#define ABDAC_CR_EN               (1 << 31) /* Bit 31: Enable Audio Bitstream DAC */

/* Interrupt Mask Register Bit-field Definitions */
/* Interrupt Enable Register Bit-field Definitions */
/* Interrupt Disable Register Bit-field Definitions */
/* Interrupt Clear Register Bit-field Definitions */
/* Interrupt Status Register Bit-field Definitions */

#define ABDAC_INT_UNDERRUN        (1 << 28)  /* Bit 28:  Underrun Interrupt Status */
#define ABDAC_INT_TXREADY         (1 << 29)  /* Bit 29  TX Ready Interrupt Status */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_ABDAC_H */

