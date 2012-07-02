/************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_atimer.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_ATIMER_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_ATIMER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define LPC43_ATIMER_COUNT_OFFSET    0x0000 /* Downcounter register */
#define LPC43_ATIMER_PRESET_OFFSET   0x0004 /* Preset value register */
#define LPC43_ATIMER_CLREN_OFFSET    0x0fd8 /* Interrupt clear enable register */
#define LPC43_ATIMER_SETEN_OFFSET    0x0fdc /* Interrupt set enable register */
#define LPC43_ATIMER_STATUS_OFFSET   0x0fe0 /* Status register */
#define LPC43_ATIMER_ENABLE_OFFSET   0x0fe4 /* Enable register */
#define LPC43_ATIMER_CLRSTAT_OFFSET  0x0fe8 /* Clear register */
#define LPC43_ATIMER_SETSTAT_OFFSET  0x0fec /* Set register */

/* Register Addresses ***************************************************************/

#define LPC43_ATIMER_COUNT           (LPC43_ATIMER_BASE+LPC43_ATIMER_COUNT_OFFSET)
#define LPC43_ATIMER_PRESET          (LPC43_ATIMER_BASE+LPC43_ATIMER_PRESET_OFFSET)
#define LPC43_ATIMER_CLREN           (LPC43_ATIMER_BASE+LPC43_ATIMER_CLREN_OFFSET)
#define LPC43_ATIMER_SETEN           (LPC43_ATIMER_BASE+LPC43_ATIMER_SETEN_OFFSET)
#define LPC43_ATIMER_STATUS          (LPC43_ATIMER_BASE+LPC43_ATIMER_STATUS_OFFSET)
#define LPC43_ATIMER_ENABLE          (LPC43_ATIMER_BASE+LPC43_ATIMER_ENABLE_OFFSET)
#define LPC43_ATIMER_CLRSTAT         (LPC43_ATIMER_BASE+LPC43_ATIMER_CLRSTAT_OFFSET)
#define LPC43_ATIMER_SETSTAT         (LPC43_ATIMER_BASE+LPC43_ATIMER_SETSTAT_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Downcounter register */

#define ATIMER_COUNT_MASK            0xffff    /* Bits 0-15: Down counter */
                                               /* Bits 16-31: Reserved */
/* Preset value register */

#define ATIMER_PRESET_MASK           0xffff    /* Bits 0-15: Counter reload value */
                                               /* Bits 16-31: Reserved */
/* Interrupt clear enable register */

#define ATIMER_CLREN                 (1 << 0)  /* Bit 0:  Clear interrupt enable */
                                               /* Bits 1-31: Reserved */
/* Interrupt set enable register */

#define ATIMER_SETEN                 (1 << 0)  /* Bit 0:  Set interrupt enable */
                                               /* Bits 1-31: Reserved */
/* Status register */

#define ATIMER_STATUS                (1 << 0)  /* Bit 0:  Interrupt status */
                                               /* Bits 1-31: Reserved */
/* Enable register */

#define ATIMER_ENABLE                (1 << 0)  /* Bit 0:  Interrupt enable status */
                                               /* Bits 1-31: Reserved */
/* Clear register */

#define ATIMER_CLRSTAT               (1 << 0)  /* Bit 0:  Clear interrupt status */
                                               /* Bits 1-31: Reserved */
/* Set register */

#define ATIMER_SETSTAT               (1 << 0)  /* Bit 0:  Set interrupt status */
                                               /* Bits 1-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_ATIMER_H */
