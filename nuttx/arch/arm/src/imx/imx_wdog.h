/************************************************************************************
 * arch/arm/src/imx/imx_wdog.h
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

#ifndef __ARCH_ARM_IMX_WDOG_H
#define __ARCH_ARM_IMX_WDOG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
 
/************************************************************************************
 * Definitions
 ************************************************************************************/

/* WDOG Register Offsets ************************************************************/

#define WDOG_WCR_OFFSET     0x0000 /* Watchdog Control Register */
#define WDOG_WSR_OFFSET     0x0004 /* Watchdog Service Register */
#define WDOG_WSTR_OFFSET    0x0008 /* Watchdog Status Register */

/* WDOG Register Addresses **********************************************************/

#define IMX_WDOG_WCR        (IMX_WDOG_VBASE + WDOG_WCR_OFFSET)
#define IMX_WDOG_WSR        (IMX_WDOG_VBASE + WDOG_WSR_OFFSET)
#define IMX_WDOG_WSTRT      (IMX_WDOG_VBASE + WDOG_WSTR_OFFSET)

/* WDOG Register Bit Definitions ****************************************************/

/* Watchdog Control Register */

#define WDOG_WCR_WDE        (1 << 0)  /* Bit 0: Watchdog Enable */
#define WDOG_WCR_WDEC       (1 << 1)  /* Bit 1: Watchdog Enable Control */
#define WDOG_WCR_SWR        (1 << 2)  /* Bit 2: Software Reset Enable */
#define WDOG_WCR_TMD        (1 << 3)  /* Bit 3: Test Mode Enable */
#define WDOG_WCR_WIE        (1 << 4)  /* Bit 4: Watchdog Interrupt Enable */
#define WDOG_WCR_WT_SHIFT   8 /* Bit 8-14: Watchdog Timeout */
#define WDOG_WCR_WT_MASK    (0x7f << WDOG_WCR_WT_SHIFT)
#define WDOG_WCR_WHALT      (1 << 15) /* Bit 15: Watchdog Halt */

/* Watchdog Service Register */

#define WDOG_WSR_SHIFT      0 /* Bit 0-15: Watchdog Service Register */
#define WDOG_WT_MASK        (0xffff << WDOG_WSR_SHIFT)

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_IMX_WDOG_H */
