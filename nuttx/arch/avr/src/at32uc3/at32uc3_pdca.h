/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_pdca.h
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_PDCA_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_PDCA_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* DMA Channel Offsets **************************************************************/

#define AVR32_PDCA_CHAN_OFFSET(n) ((n) << 6)
#define AVR32_PDCA_CHAN0_OFFSET   0x000
#define AVR32_PDCA_CHAN1_OFFSET   0x040
#define AVR32_PDCA_CHAN2_OFFSET   0x080
#define AVR32_PDCA_CHAN3_OFFSET   0x0c0
#define AVR32_PDCA_CHAN4_OFFSET   0x100
#define AVR32_PDCA_CHAN5_OFFSET   0x140
#define AVR32_PDCA_CHAN6_OFFSET   0x180
#define AVR32_PDCA_CHAN7_OFFSET   0x1c0

/* Channel Register Offsets *********************************************************/

#define AVR32_PDCA_MAR_OFFSET     0x000 /* Memory Address Register */
#define AVR32_PDCA_PSR_OFFSET     0x004 /* Peripheral Select Register */
#define AVR32_PDCA_TCR_OFFSET     0x008 /* Transfer Counter Register */
#define AVR32_PDCA_MARR_OFFSET    0x00C /* Memory Address Reload Register */
#define AVR32_PDCA_TCRR_OFFSET    0x010 /* Transfer Counter Reload Register */
#define AVR32_PDCA_CR_OFFSET      0x014 /* Control Register */
#define AVR32_PDCA_MR_OFFSET      0x018 /* Mode Register */
#define AVR32_PDCA_SR_OFFSET      0x01C /* Status Register */
#define AVR32_PDCA_IER_OFFSET     0x020 /* Interrupt Enable Register */
#define AVR32_PDCA_IDR_OFFSET     0x024 /* Interrupt Disable Register */
#define AVR32_PDCA_IMR_OFFSET     0x028 /* Interrupt Mask Register */
#define AVR32_PDCA_ISR_OFFSET     0x02C /* Interrupt Status Register */

/* DMA Channel Base Addresses *******************************************************/

#define AVR32_PDCA_CHAN_BASE(n)   (AVR32_PDCA_BASE+AVR32_PDCA_CHAN_OFFSET(n))
#define AVR32_PDCA_CHAN0_BASE     (AVR32_PDCA_BASE+AVR32_PDCA_CHAN0_OFFSET)
#define AVR32_PDCA_CHAN1_BASE     (AVR32_PDCA_BASE+AVR32_PDCA_CHAN1_OFFSET)
#define AVR32_PDCA_CHAN2_BASE     (AVR32_PDCA_BASE+AVR32_PDCA_CHAN2_OFFSET)
#define AVR32_PDCA_CHAN3_BASE     (AVR32_PDCA_BASE+AVR32_PDCA_CHAN3_OFFSET)
#define AVR32_PDCA_CHAN4_BASE     (AVR32_PDCA_BASE+AVR32_PDCA_CHAN4_OFFSET)
#define AVR32_PDCA_CHAN5_BASE     (AVR32_PDCA_BASE+AVR32_PDCA_CHAN5_OFFSET)
#define AVR32_PDCA_CHAN6_BASE     (AVR32_PDCA_BASE+AVR32_PDCA_CHAN6_OFFSET)
#define AVR32_PDCA_CHAN7_BASE     (AVR32_PDCA_BASE+AVR32_PDCA_CHAN7_OFFSET)

/* Channel Register Addresses *******************************************************/

#define AVR32_PDCA_MAR(n)         (AVR32_PDCA_CHAN_BASE(n)+AVR32_PDCA_MAR_OFFSET)
#define AVR32_PDCA_PSR(n)         (AVR32_PDCA_CHAN_BASE(n)+AVR32_PDCA_PSR_OFFSET)
#define AVR32_PDCA_TCR(n)         (AVR32_PDCA_CHAN_BASE(n)+AVR32_PDCA_TCR_OFFSET)
#define AVR32_PDCA_MARR(n)        (AVR32_PDCA_CHAN_BASE(n)+AVR32_PDCA_MARR_OFFSET)
#define AVR32_PDCA_TCRR(n)        (AVR32_PDCA_CHAN_BASE(n)+AVR32_PDCA_TCRR_OFFSET)
#define AVR32_PDCA_CR(n)          (AVR32_PDCA_CHAN_BASE(n)+AVR32_PDCA_CR_OFFSET)
#define AVR32_PDCA_MR(n)          (AVR32_PDCA_CHAN_BASE(n)+AVR32_PDCA_MR_OFFSET)
#define AVR32_PDCA_SR(n)          (AVR32_PDCA_CHAN_BASE(n)+AVR32_PDCA_SR_OFFSET)
#define AVR32_PDCA_IER(n)         (AVR32_PDCA_CHAN_BASE(n)+AVR32_PDCA_IER_OFFSET)
#define AVR32_PDCA_IDR(n)         (AVR32_PDCA_CHAN_BASE(n)+AVR32_PDCA_IDR_OFFSET)
#define AVR32_PDCA_IMR(n)         (AVR32_PDCA_CHAN_BASE(n)+AVR32_PDCA_IMR_OFFSET)
#define AVR32_PDCA_ISR(n)         (AVR32_PDCA_CHAN_BASE(n)+AVR32_PDCA_ISR_OFFSET)

#define AVR32_PDCA_CHAN0_MAR      (AVR32_PDCA_CHAN0_BASE+AVR32_PDCA_MAR_OFFSET)
#define AVR32_PDCA_CHAN0_PSR      (AVR32_PDCA_CHAN0_BASE+AVR32_PDCA_PSR_OFFSET)
#define AVR32_PDCA_CHAN0_TCR      (AVR32_PDCA_CHAN0_BASE+AVR32_PDCA_TCR_OFFSET)
#define AVR32_PDCA_CHAN0_MARR     (AVR32_PDCA_CHAN0_BASE+AVR32_PDCA_MARR_OFFSET)
#define AVR32_PDCA_CHAN0_TCRR     (AVR32_PDCA_CHAN0_BASE+AVR32_PDCA_TCRR_OFFSET)
#define AVR32_PDCA_CHAN0_CR       (AVR32_PDCA_CHAN0_BASE+AVR32_PDCA_CR_OFFSET)
#define AVR32_PDCA_CHAN0_MR       (AVR32_PDCA_CHAN0_BASE+AVR32_PDCA_MR_OFFSET)
#define AVR32_PDCA_CHAN0_SR       (AVR32_PDCA_CHAN0_BASE+AVR32_PDCA_SR_OFFSET)
#define AVR32_PDCA_CHAN0_IER      (AVR32_PDCA_CHAN0_BASE+AVR32_PDCA_IER_OFFSET)
#define AVR32_PDCA_CHAN0_IDR      (AVR32_PDCA_CHAN0_BASE+AVR32_PDCA_IDR_OFFSET)
#define AVR32_PDCA_CHAN0_IMR      (AVR32_PDCA_CHAN0_BASE+AVR32_PDCA_IMR_OFFSET)
#define AVR32_PDCA_CHAN0_ISR      (AVR32_PDCA_CHAN0_BASE+AVR32_PDCA_ISR_OFFSET)

#define AVR32_PDCA_CHAN1_MAR      (AVR32_PDCA_CHAN1_BASE+AVR32_PDCA_MAR_OFFSET)
#define AVR32_PDCA_CHAN1_PSR      (AVR32_PDCA_CHAN1_BASE+AVR32_PDCA_PSR_OFFSET)
#define AVR32_PDCA_CHAN1_TCR      (AVR32_PDCA_CHAN1_BASE+AVR32_PDCA_TCR_OFFSET)
#define AVR32_PDCA_CHAN1_MARR     (AVR32_PDCA_CHAN1_BASE+AVR32_PDCA_MARR_OFFSET)
#define AVR32_PDCA_CHAN1_TCRR     (AVR32_PDCA_CHAN1_BASE+AVR32_PDCA_TCRR_OFFSET)
#define AVR32_PDCA_CHAN1_CR       (AVR32_PDCA_CHAN1_BASE+AVR32_PDCA_CR_OFFSET)
#define AVR32_PDCA_CHAN1_MR       (AVR32_PDCA_CHAN1_BASE+AVR32_PDCA_MR_OFFSET)
#define AVR32_PDCA_CHAN1_SR       (AVR32_PDCA_CHAN1_BASE+AVR32_PDCA_SR_OFFSET)
#define AVR32_PDCA_CHAN1_IER      (AVR32_PDCA_CHAN1_BASE+AVR32_PDCA_IER_OFFSET)
#define AVR32_PDCA_CHAN1_IDR      (AVR32_PDCA_CHAN1_BASE+AVR32_PDCA_IDR_OFFSET)
#define AVR32_PDCA_CHAN1_IMR      (AVR32_PDCA_CHAN1_BASE+AVR32_PDCA_IMR_OFFSET)
#define AVR32_PDCA_CHAN1_ISR      (AVR32_PDCA_CHAN1_BASE+AVR32_PDCA_ISR_OFFSET)

#define AVR32_PDCA_CHAN2_MAR      (AVR32_PDCA_CHAN2_BASE+AVR32_PDCA_MAR_OFFSET)
#define AVR32_PDCA_CHAN2_PSR      (AVR32_PDCA_CHAN2_BASE+AVR32_PDCA_PSR_OFFSET)
#define AVR32_PDCA_CHAN2_TCR      (AVR32_PDCA_CHAN2_BASE+AVR32_PDCA_TCR_OFFSET)
#define AVR32_PDCA_CHAN2_MARR     (AVR32_PDCA_CHAN2_BASE+AVR32_PDCA_MARR_OFFSET)
#define AVR32_PDCA_CHAN2_TCRR     (AVR32_PDCA_CHAN2_BASE+AVR32_PDCA_TCRR_OFFSET)
#define AVR32_PDCA_CHAN2_CR       (AVR32_PDCA_CHAN2_BASE+AVR32_PDCA_CR_OFFSET)
#define AVR32_PDCA_CHAN2_MR       (AVR32_PDCA_CHAN2_BASE+AVR32_PDCA_MR_OFFSET)
#define AVR32_PDCA_CHAN2_SR       (AVR32_PDCA_CHAN2_BASE+AVR32_PDCA_SR_OFFSET)
#define AVR32_PDCA_CHAN2_IER      (AVR32_PDCA_CHAN2_BASE+AVR32_PDCA_IER_OFFSET)
#define AVR32_PDCA_CHAN2_IDR      (AVR32_PDCA_CHAN2_BASE+AVR32_PDCA_IDR_OFFSET)
#define AVR32_PDCA_CHAN2_IMR      (AVR32_PDCA_CHAN2_BASE+AVR32_PDCA_IMR_OFFSET)
#define AVR32_PDCA_CHAN2_ISR      (AVR32_PDCA_CHAN2_BASE+AVR32_PDCA_ISR_OFFSET)

#define AVR32_PDCA_CHAN3_MAR      (AVR32_PDCA_CHAN3_BASE+AVR32_PDCA_MAR_OFFSET)
#define AVR32_PDCA_CHAN3_PSR      (AVR32_PDCA_CHAN3_BASE+AVR32_PDCA_PSR_OFFSET)
#define AVR32_PDCA_CHAN3_TCR      (AVR32_PDCA_CHAN3_BASE+AVR32_PDCA_TCR_OFFSET)
#define AVR32_PDCA_CHAN3_MARR     (AVR32_PDCA_CHAN3_BASE+AVR32_PDCA_MARR_OFFSET)
#define AVR32_PDCA_CHAN3_TCRR     (AVR32_PDCA_CHAN3_BASE+AVR32_PDCA_TCRR_OFFSET)
#define AVR32_PDCA_CHAN3_CR       (AVR32_PDCA_CHAN3_BASE+AVR32_PDCA_CR_OFFSET)
#define AVR32_PDCA_CHAN3_MR       (AVR32_PDCA_CHAN3_BASE+AVR32_PDCA_MR_OFFSET)
#define AVR32_PDCA_CHAN3_SR       (AVR32_PDCA_CHAN3_BASE+AVR32_PDCA_SR_OFFSET)
#define AVR32_PDCA_CHAN3_IER      (AVR32_PDCA_CHAN3_BASE+AVR32_PDCA_IER_OFFSET)
#define AVR32_PDCA_CHAN3_IDR      (AVR32_PDCA_CHAN3_BASE+AVR32_PDCA_IDR_OFFSET)
#define AVR32_PDCA_CHAN3_IMR      (AVR32_PDCA_CHAN3_BASE+AVR32_PDCA_IMR_OFFSET)
#define AVR32_PDCA_CHAN3_ISR      (AVR32_PDCA_CHAN3_BASE+AVR32_PDCA_ISR_OFFSET)

#define AVR32_PDCA_CHAN4_MAR      (AVR32_PDCA_CHAN4_BASE+AVR32_PDCA_MAR_OFFSET)
#define AVR32_PDCA_CHAN4_PSR      (AVR32_PDCA_CHAN4_BASE+AVR32_PDCA_PSR_OFFSET)
#define AVR32_PDCA_CHAN4_TCR      (AVR32_PDCA_CHAN4_BASE+AVR32_PDCA_TCR_OFFSET)
#define AVR32_PDCA_CHAN4_MARR     (AVR32_PDCA_CHAN4_BASE+AVR32_PDCA_MARR_OFFSET)
#define AVR32_PDCA_CHAN4_TCRR     (AVR32_PDCA_CHAN4_BASE+AVR32_PDCA_TCRR_OFFSET)
#define AVR32_PDCA_CHAN4_CR       (AVR32_PDCA_CHAN4_BASE+AVR32_PDCA_CR_OFFSET)
#define AVR32_PDCA_CHAN4_MR       (AVR32_PDCA_CHAN4_BASE+AVR32_PDCA_MR_OFFSET)
#define AVR32_PDCA_CHAN4_SR       (AVR32_PDCA_CHAN4_BASE+AVR32_PDCA_SR_OFFSET)
#define AVR32_PDCA_CHAN4_IER      (AVR32_PDCA_CHAN4_BASE+AVR32_PDCA_IER_OFFSET)
#define AVR32_PDCA_CHAN4_IDR      (AVR32_PDCA_CHAN4_BASE+AVR32_PDCA_IDR_OFFSET)
#define AVR32_PDCA_CHAN4_IMR      (AVR32_PDCA_CHAN4_BASE+AVR32_PDCA_IMR_OFFSET)
#define AVR32_PDCA_CHAN4_ISR      (AVR32_PDCA_CHAN4_BASE+AVR32_PDCA_ISR_OFFSET)

#define AVR32_PDCA_CHAN5_MAR      (AVR32_PDCA_CHAN5_BASE+AVR32_PDCA_MAR_OFFSET)
#define AVR32_PDCA_CHAN5_PSR      (AVR32_PDCA_CHAN5_BASE+AVR32_PDCA_PSR_OFFSET)
#define AVR32_PDCA_CHAN5_TCR      (AVR32_PDCA_CHAN5_BASE+AVR32_PDCA_TCR_OFFSET)
#define AVR32_PDCA_CHAN5_MARR     (AVR32_PDCA_CHAN5_BASE+AVR32_PDCA_MARR_OFFSET)
#define AVR32_PDCA_CHAN5_TCRR     (AVR32_PDCA_CHAN5_BASE+AVR32_PDCA_TCRR_OFFSET)
#define AVR32_PDCA_CHAN5_CR       (AVR32_PDCA_CHAN5_BASE+AVR32_PDCA_CR_OFFSET)
#define AVR32_PDCA_CHAN5_MR       (AVR32_PDCA_CHAN5_BASE+AVR32_PDCA_MR_OFFSET)
#define AVR32_PDCA_CHAN5_SR       (AVR32_PDCA_CHAN5_BASE+AVR32_PDCA_SR_OFFSET)
#define AVR32_PDCA_CHAN5_IER      (AVR32_PDCA_CHAN5_BASE+AVR32_PDCA_IER_OFFSET)
#define AVR32_PDCA_CHAN5_IDR      (AVR32_PDCA_CHAN5_BASE+AVR32_PDCA_IDR_OFFSET)
#define AVR32_PDCA_CHAN5_IMR      (AVR32_PDCA_CHAN5_BASE+AVR32_PDCA_IMR_OFFSET)
#define AVR32_PDCA_CHAN5_ISR      (AVR32_PDCA_CHAN5_BASE+AVR32_PDCA_ISR_OFFSET)

#define AVR32_PDCA_CHAN6_MAR      (AVR32_PDCA_CHAN6_BASE+AVR32_PDCA_MAR_OFFSET)
#define AVR32_PDCA_CHAN6_PSR      (AVR32_PDCA_CHAN6_BASE+AVR32_PDCA_PSR_OFFSET)
#define AVR32_PDCA_CHAN6_TCR      (AVR32_PDCA_CHAN6_BASE+AVR32_PDCA_TCR_OFFSET)
#define AVR32_PDCA_CHAN6_MARR     (AVR32_PDCA_CHAN6_BASE+AVR32_PDCA_MARR_OFFSET)
#define AVR32_PDCA_CHAN6_TCRR     (AVR32_PDCA_CHAN6_BASE+AVR32_PDCA_TCRR_OFFSET)
#define AVR32_PDCA_CHAN6_CR       (AVR32_PDCA_CHAN6_BASE+AVR32_PDCA_CR_OFFSET)
#define AVR32_PDCA_CHAN6_MR       (AVR32_PDCA_CHAN6_BASE+AVR32_PDCA_MR_OFFSET)
#define AVR32_PDCA_CHAN6_SR       (AVR32_PDCA_CHAN6_BASE+AVR32_PDCA_SR_OFFSET)
#define AVR32_PDCA_CHAN6_IER      (AVR32_PDCA_CHAN6_BASE+AVR32_PDCA_IER_OFFSET)
#define AVR32_PDCA_CHAN6_IDR      (AVR32_PDCA_CHAN6_BASE+AVR32_PDCA_IDR_OFFSET)
#define AVR32_PDCA_CHAN6_IMR      (AVR32_PDCA_CHAN6_BASE+AVR32_PDCA_IMR_OFFSET)
#define AVR32_PDCA_CHAN6_ISR      (AVR32_PDCA_CHAN6_BASE+AVR32_PDCA_ISR_OFFSET)

#define AVR32_PDCA_CHAN7_MAR      (AVR32_PDCA_CHAN7_BASE+AVR32_PDCA_MAR_OFFSET)
#define AVR32_PDCA_CHAN7_PSR      (AVR32_PDCA_CHAN7_BASE+AVR32_PDCA_PSR_OFFSET)
#define AVR32_PDCA_CHAN7_TCR      (AVR32_PDCA_CHAN7_BASE+AVR32_PDCA_TCR_OFFSET)
#define AVR32_PDCA_CHAN7_MARR     (AVR32_PDCA_CHAN7_BASE+AVR32_PDCA_MARR_OFFSET)
#define AVR32_PDCA_CHAN7_TCRR     (AVR32_PDCA_CHAN7_BASE+AVR32_PDCA_TCRR_OFFSET)
#define AVR32_PDCA_CHAN7_CR       (AVR32_PDCA_CHAN7_BASE+AVR32_PDCA_CR_OFFSET)
#define AVR32_PDCA_CHAN7_MR       (AVR32_PDCA_CHAN7_BASE+AVR32_PDCA_MR_OFFSET)
#define AVR32_PDCA_CHAN7_SR       (AVR32_PDCA_CHAN7_BASE+AVR32_PDCA_SR_OFFSET)
#define AVR32_PDCA_CHAN7_IER      (AVR32_PDCA_CHAN7_BASE+AVR32_PDCA_IER_OFFSET)
#define AVR32_PDCA_CHAN7_IDR      (AVR32_PDCA_CHAN7_BASE+AVR32_PDCA_IDR_OFFSET)
#define AVR32_PDCA_CHAN7_IMR      (AVR32_PDCA_CHAN7_BASE+AVR32_PDCA_IMR_OFFSET)
#define AVR32_PDCA_CHAN7_ISR      (AVR32_PDCA_CHAN7_BASE+AVR32_PDCA_ISR_OFFSET)

/* Channel Register Bit-field Definitions *******************************************/

/* Memory Address Register Bit-field Definitions */
/* Memory Address Reload Register Bit-field Definitions */
/* These registers hold a 32-bit address and contain no bit-fields */

/* Peripheral Select Register Bit-field Definitions */

#define PDCA_PSR_PID_SHIFT        (0)     /* Bits 0-7: Peripheral Identifier */
#define PDCA_PSR_PID_MASK         (0xff << PDCA_PSR_PID_SHIFT)
#  define PDCA_PSR_PID_ADC        (0 << PDCA_PSR_PID_SHIFT)  /* ADC */
#  define PDCA_PSR_PID_SSCRX      (1 << PDCA_PSR_PID_SHIFT)  /* SSC - RX */
#  define PDCA_PSR_PID_USART0RX   (2 << PDCA_PSR_PID_SHIFT)  /* USART0 - RX */
#  define PDCA_PSR_PID_USART1RX   (3 << PDCA_PSR_PID_SHIFT)  /* USART1 - RX */
#  define PDCA_PSR_PID_USART2RX   (4 << PDCA_PSR_PID_SHIFT)  /* USART2 - RX */
#  define PDCA_PSR_PID_TWIRX      (5 << PDCA_PSR_PID_SHIFT)  /* TWI - RX */
#  define PDCA_PSR_PID_SPI0RX     (6 << PDCA_PSR_PID_SHIFT)  /* SPI0 - RX */
#  define PDCA_PSR_PID_SSCTX      (7 << PDCA_PSR_PID_SHIFT)  /* SSC - TX */
#  define PDCA_PSR_PID_USART0TX   (8 << PDCA_PSR_PID_SHIFT)  /* USART0 - TX */
#  define PDCA_PSR_PID_USART1TX   (9 << PDCA_PSR_PID_SHIFT)  /* USART1 - TX */
#  define PDCA_PSR_PID_USART2TX   (10 << PDCA_PSR_PID_SHIFT) /* USART2 - TX */
#  define PDCA_PSR_PID_TWITX      (11 << PDCA_PSR_PID_SHIFT) /* TWI - TX */
#  define PDCA_PSR_PID_SPI0TX     (12 << PDCA_PSR_PID_SHIFT) /* SPI0 - TX */
#  define PDCA_PSR_PID_ABDACTX    (13 << PDCA_PSR_PID_SHIFT) /* ABDAC - TX */

/* Transfer Counter Register Bit-field Definitions */
/* Transfer Counter Reload Register Bit-field Definitions */

#define PDCA_TCV_SHIFT            (0)     /* Bits 0-15: Transfer Counter Value */
#define PDCA_TCV_MASK             (0xffff << PDCA_TCV_SHIFT)

/* Control Register Bit-field Definitions */

#define PDCA_CR_TEN               (1 << 0) /* Bit 0: Transfer Enable */
#define PDCA_CR_TDIS              (1 << 1) /* Bit 1: Transfer Disable */
#define PDCA_CR_ECLR              (1 << 8) /* Bit 8: Transfer Error Clear */

/* Mode Register Bit-field Definitions */

#define PDCA_MR_SIZE_SHIFT        (0)     /* Bits 0-1: Prescale Select */
#define PDCA_MR_SIZE_MASK         (3 << PDCA_MR_SIZE_SHIFT)
#  define PDCA_MR_SIZE_ BYTE      (0 << PDCA_MR_SIZE_SHIFT) /* Byte */
#  define PDCA_MR_SIZE_ HWORD     (1 << PDCA_MR_SIZE_SHIFT) /* Halfword */
#  define PDCA_MR_SIZE_ WORD      (2 << PDCA_MR_SIZE_SHIFT) /* Word */

/* Status Register Bit-field Definitions */

#define PDCA_SR_TEN               (1 << 0) /* Bit 0: Transfer Enabled */

/* Interrupt Enable Register Bit-field Definitions */
/* Interrupt Disable Register Bit-field Definitions */
/* Interrupt Mask Register Bit-field Definitions */
/* Interrupt Status Register Bit-field Definitions */

#define PDCA_INT_RCZ              (1 << 0) /* Bit 0: Reload Counter Zero */
#define PDCA_INT_TRC              (1 << 1) /* Bit 1: Transfer Complete */
#define PDCA_INT_TERR             (1 << 2) /* Bit 2: Transfer Error */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_PDCA_H */

