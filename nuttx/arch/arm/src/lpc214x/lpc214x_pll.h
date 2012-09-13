/****************************************************************************************************
 * arch/arm/src/lpc214x/lpc214x_pll.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef _ARCH_ARM_SRC_LPC214X_PLL_H
#define _ARCH_ARM_SRC_LPC214X_PLL_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <chip.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* PLL bass addresses *******************************************************************************/

/* There are two PLLs:  PLL0 generates CCLK and PLL1 is configured to provide the 48MHx USB clock */

#define LPC214X_PLL0_BASE               (LPC214X_PLL_BASE)
#define LPC214X_PLL1_BASE               (LPC214X_PLL_BASE + 0x00000020)

/* PLL registers ************************************************************************************/

#define LPC214x_PLL0_CON                (LPC214X_PLL0_BASE+LPC214X_PLL_CON_OFFSET)
#define LPC214x_PLL0_CFG                (LPC214X_PLL0_BASE+LPC214X_PLL_CFG_OFFSET)
#define LPC214x_PLL0_STAT               (LPC214X_PLL0_BASE+LPC214X_PLL_STAT_OFFSET)
#define LPC214x_PLL0_FEED               (LPC214X_PLL0_BASE+LPC214X_PLL_FEED_OFFSET)

#define LPC214x_PLL1_CON                (LPC214X_PLL1_BASE+LPC214X_PLL_CON_OFFSET)
#define LPC214x_PLL1_CFG                (LPC214X_PLL1_BASE+LPC214X_PLL_CFG_OFFSET)
#define LPC214x_PLL1_STAT               (LPC214X_PLL1_BASE+LPC214X_PLL_STAT_OFFSET)
#define LPC214x_PLL1_FEED               (LPC214X_PLL1_BASE+LPC214X_PLL_FEED_OFFSET)

/* Register bit settings ****************************************************************************/

/* PLL Control Register Bit Settings */

#define LPC214X_PLL_CON_PLLE            (1 << 0) /* PLL Enable */
#define LPC214X_PLL_CON_PLLC            (1 << 1) /* PLL Connect */

/* PLL Configuration Register Bit Settings */

#define LPC214X_PLL_CFG_MSEL            (0x1f << 0) /* PLL Multiplier (minus 1) */
#define LPC214X_PLL_CFG_PSEL            (0x03 << 5) /* PLL Divider (encoded) */
#define LPC214X_PLL_CFG_PSEL1           (0x00 << 5)
#define LPC214X_PLL_CFG_PSEL2           (0x01 << 5)
#define LPC214X_PLL_CFG_PSEL4           (0x02 << 5)
#define LPC214X_PLL_CFG_PSEL8           (0x03 << 5)

/* PLL Status Register Bit Settings */

#define LPC214X_PLL_STAT_MSEL           (0x1f << 0) /* PLL Multiplier Readback */
#define LPC214X_PLL_STAT_PSEL           (0x03 << 5) /* PLL Divider Readback */
#define LPC214X_PLL_STAT_PLLE           (1 << 8)    /* PLL Enable Readback */
#define LPC214X_PLL_STAT_PLLC           (1 << 9)    /* PLL Connect Readback */
#define LPC214X_PLL_STAT_PLOCK          (1 << 10)   /* PLL Lock Status */

/* PLL Feed Register values */

#define LPC214X_PLL_FEED1               0xaa
#define LPC214X_PLL_FEED2               0x55

/****************************************************************************************************
 * Inline Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Global Function Prototypes
 ****************************************************************************************************/

#endif  /* _ARCH_ARM_SRC_LPC214X_PLL_H */
