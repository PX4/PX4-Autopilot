/************************************************************************************
 * configs/mx1ads/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clock settings -- All clock values are precalculated */

#define IMX_SYS_CLK_FREQ    16780000  /* Crystal frequency */

/* MPCTL0 -- Controls the MCU clock:
 *
 *                                                MFI + MFN / (MFD+1)
 *  IMX_MCUPLL_CLK_FREQ = 2 * IMX_SYS_CLK_FREQ * --------------------
 *                                                    PD + 1
 */

#if 0 /* 150 MHz */
#  define IMX_MPCTL0_MFN   16
#  define IMX_MPCTL0_MFI   9
#  define IMX_MPCTL0_MFD   99
#  define IMX_MPCTL0_PD    1
#else /* 180 MHz */
#  define IMX_MPCTL0_MFN   441
#  define IMX_MPCTL0_MFI   4
#  define IMX_MPCTL0_MFD   938
#  define IMX_MPCTL0_PD    0
#endif

#define IMX_MPCTL0_VALUE \
  ((IMX_MPCTL0_MFN << PLL_MPCTL0_MFN_SHIFT) |\
   (IMX_MPCTL0_MFI << PLL_MPCTL0_MFI_SHIFT) |\
   (IMX_MPCTL0_MFD << PLL_MPCTL0_MFD_SHIFT) |\
   (IMX_MPCTL0_PD << PLL_MPCTL0_PD_SHIFT))

/* This yields: */

#if 0 /* 150 MHz */
#  define IMX_MCUPLL_CLK_FREQ 153704800
#else /* 180 MHz */
#  define IMX_MCUPLL_CLK_FREQ 183561405
#endif

/* SPCTL0 -- Controls the system PLL:
 *
 *                                                MFI + MFN / (MFD+1)
 *  IMX_SYSPLL_CLK_FREQ = 2 * IMX_SYS_CLK_FREQ * --------------------
 *                                                    PD + 1
 */

#define IMX_SPCTL0_MFN   678
#define IMX_SPCTL0_MFI   5
#define IMX_SPCTL0_MFD   938
#define IMX_SPCTL0_PD    1

#define IMX_SPCTL0_VALUE \
  ((IMX_SPCTL0_MFN << PLL_SPCTL0_MFN_SHIFT) |\
   (IMX_SPCTL0_MFI << PLL_SPCTL0_MFI_SHIFT) |\
   (IMX_SPCTL0_MFD << PLL_SPCTL0_MFD_SHIFT) |\
   (IMX_SPCTL0_PD << PLL_SPCTL0_PD_SHIFT))

/* This yields: */

#define IMX_SYSPLL_CLK_FREQ 96015910

/* PDCR -- Controls peripheral clocks */

#define IMX_PCLKDIV1 0
#define IMX_PCLKDIV2 0
#define IMX_PCLKDIV3 0

#define IMX_PCDR_VALUE \
  ((IMX_PCLKDIV1 << PLL_PCDR_PCLKDIV1_SHIFT) |\
   (IMX_PCLKDIV2 << PLL_PCDR_PCLKDIV2_SHIFT) |\
   (IMX_PCLKDIV3 << PLL_PCDR_PCLKDIV3_SHIFT))

/* PERCLK1: UART, Timers, PWM */

#define IMX_PERCLK1_FREQ  (IMX_SYSPLL_CLK_FREQ/(IMX_PCLKDIV1+1))

/* PERCLK2: CSPI, LCD, SD */

#define IMX_PERCLK2_FREQ  (IMX_SYSPLL_CLK_FREQ/(IMX_PCLKDIV2+1))

/* PERCLK3: SSI */

#define IMX_PERCLK3_FREQ  (IMX_SYSPLL_CLK_FREQ/(IMX_PCLKDIV3+1))

/* CSCR settings -- Controls HCLK and BCLK and USB clock.
 * HCLK: SDRAM, CSI, Memory Stick, I2C, DMA
 */

#define IMX_CSCR_BCLKDIV 1
#define IMX_CSCR_USBDIV  6

/* LED definitions ******************************************************************/

/* The MX1ADS has only one usable LED: Port A, bit 2 */

                                /* ON   OFF */
#define LED_STARTED       0     /* OFF  OFF */
#define LED_HEAPALLOCATE  1     /* OFF  OFF */
#define LED_IRQSENABLED   2     /* OFF  OFF */
#define LED_STACKCREATED  3     /* OFF  OFF */
#define LED_INIRQ         4     /* ON   OFF */
#define LED_SIGNAL        5     /* ON   OFF */
#define LED_ASSERTION     6     /* ON   OFF */
#define LED_PANIC         7     /* ON   OFF */

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__

/* All i.MX architectures must provide the following entry point.  This entry point
 * is called early in the intitialization -- after all memory has been configured
 * and mapped but before any devices have been initialized.
 */

extern void imx_boardinitialize(void);

#endif

#endif  /* __ARCH_BOARD_BOARD_H */
