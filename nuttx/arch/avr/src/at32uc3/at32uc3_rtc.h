/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_rtc.h
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_RTC_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_RTC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AVR32_RTC_CTRL_OFFSET       0x00 /* Control Register */
#define AVR32_RTC_VAL_OFFSET        0x04 /* Value Register */
#define AVR32_RTC_TOP_OFFSET        0x08 /* Top Register */
#define AVR32_RTC_IER_OFFSET        0x10 /* Interrupt Enable Register */
#define AVR32_RTC_IDR_OFFSET        0x14 /* Interrupt Disable Register */
#define AVR32_RTC_IMR_OFFSET        0x18 /* Interrupt Mask Register */
#define AVR32_RTC_ISR_OFFSET        0x1c /* Interrupt Status Register */
#define AVR32_RTC_ICR_OFFSET        0x20 /* Interrupt Clear Register */

/* Register Addresses ***************************************************************/

#define AVR32_RTC_CTRL              (AVR32_RTC_BASE+AVR32_RTC_CTRL_OFFSET)
#define AVR32_RTC_VAL               (AVR32_RTC_BASE+AVR32_RTC_VAL_OFFSET)
#define AVR32_RTC_TOP               (AVR32_RTC_BASE+AVR32_RTC_TOP_OFFSET)
#define AVR32_RTC_IER               (AVR32_RTC_BASE+AVR32_RTC_IER_OFFSET)
#define AVR32_RTC_IDR               (AVR32_RTC_BASE+AVR32_RTC_IDR_OFFSET)
#define AVR32_RTC_IMR               (AVR32_RTC_BASE+AVR32_RTC_IMR_OFFSET)
#define AVR32_RTC_ISR               (AVR32_RTC_BASE+AVR32_RTC_ISR_OFFSET)
#define AVR32_RTC_ICR               (AVR32_RTC_BASE+AVR32_RTC_ICR_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Control Register Bit-field Definitions */

#define RTC_CTRL_EN                 (1 << 0)  /* Bit 0:  Enable */
#define RTC_CTRL_PCLR               (1 << 1)  /* Bit 1:  Prescaler Clear */
#define RTC_CTRL_WAKEN              (1 << 2)  /* Bit 2:  Wakeup Enable */
#define RTC_CTRL_CLK32              (1 << 3)  /* Bit 3:  32 KHz Oscillator Select */
#define RTC_CTRL_BUSY               (1 << 4)  /* Bit 4:  RTC Busy */
#define RTC_CTRL_PSEL_SHIFT         (8)       /* Bits 8-11: Prescale Select */
#define RTC_CTRL_PSEL_MASK          (15 << RTC_CTRL_PSEL_SHIFT)
#define RTC_CTRL_CLKEN              (1 << 16) /* Bit 16: Clock Enable */

/* Value Register Bit-field Definitions */
/* This is a 32-bit data register and, hence, has no bit field */

/* Top Register Bit-field Definitions */
/* This is a 32-bit data register and, hence, has no bit field */

/* Interrupt Enable Register Bit-field Definitions
 * Interrupt Disable Register Bit-field Definitions
 * Interrupt Mask Register Bit-field Definitions
 * Interrupt Status Register Bit-field Definitions
 * Interrupt Clear Register Bit-field Definitions
 */

#define RTC_INT_TOPI                (1 << 0)  /* Bit 0:  Top interrupt */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_RTC_H */

