/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_wdt.h
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_WDT_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_WDT_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AVR32_WDT_CTRL_OFFSET 0x00 /* Control Register */
#define AVR32_WDT_CLR_OFFSET  0x04 /* Clear Register */

/* Register Addresses ***************************************************************/

#define AVR32_WDT_CTRL        (AVR32_WDT_BASE+AVR32_WDT_CTRL_OFFSET)
#define AVR32_WDT_CLR         (AVR32_WDT_BASE+AVR32_WDT_CLR_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Control Register Bit-field Definitions */

#define WDT_CTRL_EN          (1 << 0) /* Bit 0: WDT Enable */
#define WDT_CTRL_PSEL_SHIFT  (18)     /* Bits 8-12: Prescale Select */
#define WDT_CTRL_PSEL_MASK   (0x1f << WDT_CTRL_PSEL_SHIFT)
#define WDT_CTRL_KEY_SHIFT   (24)     /* Bits 24-31: Write protection key */
#define WDT_CTRL_KEY_MASK    (0xff << WDT_CTRL_KEY_SHIFT)

/* Clear Register Bit-field Definitions.  These registers have no bit fields: "Writing
 * periodically any value to this field when the WDT is enabled, within the watchdog
 * time-out period, will prevent a watchdog reset."
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

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_WDT_H */

