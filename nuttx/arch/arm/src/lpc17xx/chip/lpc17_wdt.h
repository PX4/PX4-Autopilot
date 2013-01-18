/************************************************************************************
 * arch/arm/src/lpc17xx/chip/lpc17_wdt.h
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC17_WDT_H
#define __ARCH_ARM_SRC_LPC17XX_LPC17_WDT_H

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

#define LPC17_WDT_WDMOD_OFFSET     0x0000  /* Watchdog mode register */
#define LPC17_WDT_WDTC_OFFSET      0x0004  /* Watchdog timer constant register */
#define LPC17_WDT_WDFEED_OFFSET    0x0008  /* Watchdog feed sequence register */
#define LPC17_WDT_WDTV_OFFSET      0x000c  /* Watchdog timer value register */
#define LPC17_WDT_WDCLKSEL_OFFSET  0x0010  /* Watchdog clock source selection register */

/* Register addresses ***************************************************************/

#define LPC17_WDT_WDMOD            (LPC17_WDT_BASE+LPC17_WDT_WDMOD_OFFSET)
#define LPC17_WDT_WDTC             (LPC17_WDT_BASE+LPC17_WDT_WDTC_OFFSET)
#define LPC17_WDT_WDFEED           (LPC17_WDT_BASE+LPC17_WDT_WDFEED_OFFSET)
#define LPC17_WDT_WDTV             (LPC17_WDT_BASE+LPC17_WDT_WDTV_OFFSET)
#define LPC17_WDT_WDCLKSEL         (LPC17_WDT_BASE+LPC17_WDT_WDCLKSEL_OFFSET)

/* Register bit definitions *********************************************************/

/* Watchdog mode register */

#define WDT_WDMOD_WDEN             (1 << 0)  /* Bit 0: Watchdog enable */
#define WDT_WDMOD_WDRESET          (1 << 1)  /* Bit 1: Watchdog reset enable */
#define WDT_WDMOD_WDTOF            (1 << 2)  /* Bit 2: Watchdog time-out */
#define WDT_WDMOD_WDINT            (1 << 3)  /* Bit 3: Watchdog interrupt */
                                             /* Bits 14-31: Reserved */

/* Watchdog timer constant register (Bits 0-31: Watchdog time-out interval) */

/* Watchdog feed sequence register  */

#define WDT_WDFEED_MASK            (0xff)    /* Bits 0-7: Feed value should be 0xaa followed by 0x55 */
                                             /* Bits 14-31: Reserved */
/* Watchdog timer value register (Bits 0-31: Counter timer value) */

/* Watchdog clock source selection register */

#define WDT_WDCLKSEL_WDSEL_SHIFT   (0)       /* Bits 0-1: Clock source for the Watchdog timer */
#define WDT_WDCLKSEL_WDSEL_MASK    (3 << WDT_WDCLKSEL_WDSEL_SHIFT)
#  define WDT_WDCLKSEL_WDSEL_INTRC (0 << WDT_WDCLKSEL_WDSEL_SHIFT) /* Internal RC osc */
#  define WDT_WDCLKSEL_WDSEL_APB   (1 << WDT_WDCLKSEL_WDSEL_SHIFT) /* APB peripheral clock (watchdog pclk) */
#  define WDT_WDCLKSEL_WDSEL_RTC   (2 << WDT_WDCLKSEL_WDSEL_SHIFT) /* RTC oscillator (rtc_clk) */
                                             /* Bits 2-30: Reserved */
#define WDT_WDCLKSEL_WDLOCK        (1 << 31) /* Bit 31: Lock WDT register bits if set */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC17_WDT_H */
