/************************************************************************************
 * dm320/dm320_clkc.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_DM320_DM320_CLKC_H
#define __ARCH_ARM_SRC_DM320_DM320_CLKC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clock Controller Register Map (CLKC) *********************************************/

#define DM320_CLKC_PLLA      (DM320_CLKC_REGISTER_BASE+0x0000) /* PLLA Configuration */
#define DM320_CLKC_PLLB      (DM320_CLKC_REGISTER_BASE+0x0002) /* PLLB Configuration */
#define DM320_CLKC_SEL0      (DM320_CLKC_REGISTER_BASE+0x0004) /* Input Clock Source Selection #0 */
#define DM320_CLKC_SEL1      (DM320_CLKC_REGISTER_BASE+0x0006) /* Input Slock Source Selection #1 */
#define DM320_CLKC_SEL2      (DM320_CLKC_REGISTER_BASE+0x0008) /* Input Clock Source Selection #2 */
#define DM320_CLKC_DIV0      (DM320_CLKC_REGISTER_BASE+0x000a) /* Clock Divisor Settings #0 */
#define DM320_CLKC_DIV1      (DM320_CLKC_REGISTER_BASE+0x000c) /* Clock Divisor Settings #1 */
#define DM320_CLKC_DIV2      (DM320_CLKC_REGISTER_BASE+0x000e) /* Clock Divisor Settings #2 */
#define DM320_CLKC_DIV3      (DM320_CLKC_REGISTER_BASE+0x0010) /* Clock Divisor Settings #3 */
#define DM320_CLKC_DIV4      (DM320_CLKC_REGISTER_BASE+0x0012) /* Clock Divisor Settings #4 */
#define DM320_CLKC_BYP       (DM320_CLKC_REGISTER_BASE+0x0014) /* Bypass Control */
#define DM320_CLKC_INV       (DM320_CLKC_REGISTER_BASE+0x0016) /* Inverse Control */
#define DM320_CLKC_MOD0      (DM320_CLKC_REGISTER_BASE+0x0018) /* Module Clock Enables #0 */
#define DM320_CLKC_MOD1      (DM320_CLKC_REGISTER_BASE+0x001a) /* Module ClockEnables #1 */
#define DM320_CLKC_MOD2      (DM320_CLKC_REGISTER_BASE+0x001c) /* Module ClockEnables #1 */
#define DM320_CLKC_LPCTL0    (DM320_CLKC_REGISTER_BASE+0x001e) /* Low Power Control #0 */
#define DM320_CLKC_LPCTL1    (DM320_CLKC_REGISTER_BASE+0x0020) /* Low Power Control #1 */
#define DM320_CLKC_OSEL      (DM320_CLKC_REGISTER_BASE+0x0022) /* Output Clock Selector */
#define DM320_CLKC_O0DIV     (DM320_CLKC_REGISTER_BASE+0x0024) /* Output Clock #0 Divider */
#define DM320_CLKC_O1DIV     (DM320_CLKC_REGISTER_BASE+0x0026) /* Output Clock #1 Divider */
#define DM320_CLKC_O2DIV     (DM320_CLKC_REGISTER_BASE+0x0028) /* Output Clock #2 Divider */
#define DM320_CLKC_PWM0C     (DM320_CLKC_REGISTER_BASE+0x002a) /* PWM #0 Cycle Count */
#define DM320_CLKC_PWM0H     (DM320_CLKC_REGISTER_BASE+0x002c) /* PWM #0 High Period */
#define DM320_CLKC_PWM1C     (DM320_CLKC_REGISTER_BASE+0x002e) /* PWM #1 Cycle Count */
#define DM320_CLKC_PWM1H     (DM320_CLKC_REGISTER_BASE+0x0030) /* PWM #1 Cycle Period */
#define DM320_CLKC_TEST0     (DM320_CLKC_REGISTER_BASE+0x08FE) /* Test #0 */
#define DM320_CLKC_TEST1     (DM320_CLKC_REGISTER_BASE+0x08FE) /* Test #1 */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_SRC_DM320_DM320_CLKC_H */
