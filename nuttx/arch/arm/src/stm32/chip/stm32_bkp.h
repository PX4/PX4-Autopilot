/************************************************************************************
 * arch/arm/src/stm32/chip/stm32_bkp.h
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_BKP_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_BKP_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if defined(CONFIG_STM32_HIGHDENSITY) || defined(CONFIG_STM32_CONNECTIVITYLINE)
#  define CONFIG_STM32_NBKP_BYTES 84
#  define CONFIG_STM32_NBKP_REGS  42
#else
#  define CONFIG_STM32_NBKP_BYTES 20
#  define CONFIG_STM32_NBKP_REGS  10
#endif

/* Register Offsets *****************************************************************/

#if defined(CONFIG_STM32_HIGHDENSITY) || defined(CONFIG_STM32_CONNECTIVITYLINE)
#  define STM32_BKP_DR_OFFSET(n) ((n) > 10 ? 0x0040+4*((n)-10) : 0x0004+4*(n))
#else
#  define STM32_BKP_DR_OFFSET(n)  (0x0004+4*(n))
#endif

#define STM32_BKP_DR1_OFFSET     0x0004  /* Backup data register 1 */
#define STM32_BKP_DR2_OFFSET     0x0008  /* Backup data register 2 */
#define STM32_BKP_DR3_OFFSET     0x000c  /* Backup data register 3 */
#define STM32_BKP_DR4_OFFSET     0x0010  /* Backup data register 4 */
#define STM32_BKP_DR5_OFFSET     0x0014  /* Backup data register 5 */
#define STM32_BKP_DR6_OFFSET     0x0018  /* Backup data register 6 */
#define STM32_BKP_DR7_OFFSET     0x001c  /* Backup data register 7 */
#define STM32_BKP_DR8_OFFSET     0x0020  /* Backup data register 8 */
#define STM32_BKP_DR9_OFFSET     0x0024  /* Backup data register 9 */
#define STM32_BKP_DR10_OFFSET    0x0028  /* Backup data register 10 */

#define STM32_BKP_RTCCR_OFFSET   0x002c  /* RTC clock calibration register */
#define STM32_BKP_CR_OFFSET      0x0030  /* Backup control register */
#define STM32_BKP_CSR_OFFSET     0x0034  /* Backup control/status register */

#if defined(CONFIG_STM32_HIGHDENSITY) || defined(CONFIG_STM32_CONNECTIVITYLINE)
#  define STM32_BKP_DR11_OFFSET  0x0040  /* Backup data register 11 */
#  define STM32_BKP_DR12_OFFSET  0x0044  /* Backup data register 12 */
#  define STM32_BKP_DR13_OFFSET  0x0048  /* Backup data register 13 */
#  define STM32_BKP_DR14_OFFSET  0x004c  /* Backup data register 14 */
#  define STM32_BKP_DR15_OFFSET  0x0050  /* Backup data register 15 */
#  define STM32_BKP_DR16_OFFSET  0x0054  /* Backup data register 16 */
#  define STM32_BKP_DR17_OFFSET  0x0058  /* Backup data register 17 */
#  define STM32_BKP_DR18_OFFSET  0x005c  /* Backup data register 18 */
#  define STM32_BKP_DR19_OFFSET  0x0060  /* Backup data register 19 */
#  define STM32_BKP_DR20_OFFSET  0x0064  /* Backup data register 20 */
#  define STM32_BKP_DR21_OFFSET  0x0068  /* Backup data register 21 */
#  define STM32_BKP_DR22_OFFSET  0x006c  /* Backup data register 22 */
#  define STM32_BKP_DR23_OFFSET  0x0070  /* Backup data register 23 */
#  define STM32_BKP_DR24_OFFSET  0x0074  /* Backup data register 24 */
#  define STM32_BKP_DR25_OFFSET  0x0078  /* Backup data register 25 */
#  define STM32_BKP_DR26_OFFSET  0x007c  /* Backup data register 26 */
#  define STM32_BKP_DR27_OFFSET  0x0080  /* Backup data register 27 */
#  define STM32_BKP_DR28_OFFSET  0x0084  /* Backup data register 28 */
#  define STM32_BKP_DR29_OFFSET  0x0088  /* Backup data register 29 */
#  define STM32_BKP_DR30_OFFSET  0x008c  /* Backup data register 30 */
#  define STM32_BKP_DR31_OFFSET  0x0090  /* Backup data register 31 */
#  define STM32_BKP_DR32_OFFSET  0x0094  /* Backup data register 32 */
#  define STM32_BKP_DR33_OFFSET  0x0098  /* Backup data register 33 */
#  define STM32_BKP_DR34_OFFSET  0x009c  /* Backup data register 34 */
#  define STM32_BKP_DR35_OFFSET  0x00a0  /* Backup data register 35 */
#  define STM32_BKP_DR36_OFFSET  0x00a4  /* Backup data register 36 */
#  define STM32_BKP_DR37_OFFSET  0x00a8  /* Backup data register 37 */
#  define STM32_BKP_DR38_OFFSET  0x00ac  /* Backup data register 38 */
#  define STM32_BKP_DR39_OFFSET  0x00b0  /* Backup data register 39 */
#  define STM32_BKP_DR40_OFFSET  0x00b4  /* Backup data register 40 */
#  define STM32_BKP_DR41_OFFSET  0x00b8  /* Backup data register 41 */
#  define STM32_BKP_DR42_OFFSET  0x00bc  /* Backup data register 42 */
#endif

/* Register Addresses ***************************************************************/

#define STM32_BKP_RTCCR          (STM32_BKP_BASE+STM32_BKP_RTCCR_OFFSET)
#define STM32_BKP_CR             (STM32_BKP_BASE+STM32_BKP_CR_OFFSET)
#define STM32_BKP_CSR            (STM32_BKP_BASE+STM32_BKP_CSR_OFFSET)

#define STM32_BKP_DR(n)          (STM32_BKP_BASE+STM32_BKP_DR_OFFSET(n))
#define STM32_BKP_DR1            (STM32_BKP_BASE+STM32_BKP_DR1_OFFSET)
#define STM32_BKP_DR2            (STM32_BKP_BASE+STM32_BKP_DR2_OFFSET)
#define STM32_BKP_DR3            (STM32_BKP_BASE+STM32_BKP_DR3_OFFSET)
#define STM32_BKP_DR4            (STM32_BKP_BASE+STM32_BKP_DR4_OFFSET)
#define STM32_BKP_DR5            (STM32_BKP_BASE+STM32_BKP_DR5_OFFSET)
#define STM32_BKP_DR6            (STM32_BKP_BASE+STM32_BKP_DR6_OFFSET)
#define STM32_BKP_DR7            (STM32_BKP_BASE+STM32_BKP_DR7_OFFSET)
#define STM32_BKP_DR8            (STM32_BKP_BASE+STM32_BKP_DR8_OFFSET)
#define STM32_BKP_DR9            (STM32_BKP_BASE+STM32_BKP_DR9_OFFSET)
#define STM32_BKP_DR10           (STM32_BKP_BASE+STM32_BKP_DR10_OFFSET)

#if defined(CONFIG_STM32_HIGHDENSITY) || defined(CONFIG_STM32_CONNECTIVITYLINE)
#  define STM32_BKP_DR11         (STM32_BKP_BASE+STM32_BKP_DR11_OFFSET)
#  define STM32_BKP_DR12         (STM32_BKP_BASE+STM32_BKP_DR12_OFFSET)
#  define STM32_BKP_DR13         (STM32_BKP_BASE+STM32_BKP_DR13_OFFSET)
#  define STM32_BKP_DR14         (STM32_BKP_BASE+STM32_BKP_DR14_OFFSET)
#  define STM32_BKP_DR15         (STM32_BKP_BASE+STM32_BKP_DR15_OFFSET)
#  define STM32_BKP_DR16         (STM32_BKP_BASE+STM32_BKP_DR16_OFFSET)
#  define STM32_BKP_DR17         (STM32_BKP_BASE+STM32_BKP_DR17_OFFSET)
#  define STM32_BKP_DR18         (STM32_BKP_BASE+STM32_BKP_DR18_OFFSET)
#  define STM32_BKP_DR19         (STM32_BKP_BASE+STM32_BKP_DR19_OFFSET)
#  define STM32_BKP_DR20         (STM32_BKP_BASE+STM32_BKP_DR20_OFFSET)
#  define STM32_BKP_DR21         (STM32_BKP_BASE+STM32_BKP_DR21_OFFSET)
#  define STM32_BKP_DR22         (STM32_BKP_BASE+STM32_BKP_DR22_OFFSET)
#  define STM32_BKP_DR23         (STM32_BKP_BASE+STM32_BKP_DR23_OFFSET)
#  define STM32_BKP_DR24         (STM32_BKP_BASE+STM32_BKP_DR24_OFFSET)
#  define STM32_BKP_DR25         (STM32_BKP_BASE+STM32_BKP_DR25_OFFSET)
#  define STM32_BKP_DR26         (STM32_BKP_BASE+STM32_BKP_DR26_OFFSET)
#  define STM32_BKP_DR27         (STM32_BKP_BASE+STM32_BKP_DR27_OFFSET)
#  define STM32_BKP_DR28         (STM32_BKP_BASE+STM32_BKP_DR28_OFFSET)
#  define STM32_BKP_DR29         (STM32_BKP_BASE+STM32_BKP_DR29_OFFSET)
#  define STM32_BKP_DR30         (STM32_BKP_BASE+STM32_BKP_DR30_OFFSET)
#  define STM32_BKP_DR31         (STM32_BKP_BASE+STM32_BKP_DR31_OFFSET)
#  define STM32_BKP_DR32         (STM32_BKP_BASE+STM32_BKP_DR32_OFFSET)
#  define STM32_BKP_DR33         (STM32_BKP_BASE+STM32_BKP_DR33_OFFSET)
#  define STM32_BKP_DR34         (STM32_BKP_BASE+STM32_BKP_DR34_OFFSET)
#  define STM32_BKP_DR35         (STM32_BKP_BASE+STM32_BKP_DR35_OFFSET)
#  define STM32_BKP_DR36         (STM32_BKP_BASE+STM32_BKP_DR36_OFFSET)
#  define STM32_BKP_DR37         (STM32_BKP_BASE+STM32_BKP_DR37_OFFSET)
#  define STM32_BKP_DR38         (STM32_BKP_BASE+STM32_BKP_DR38_OFFSET)
#  define STM32_BKP_DR39         (STM32_BKP_BASE+STM32_BKP_DR39_OFFSET)
#  define STM32_BKP_DR40         (STM32_BKP_BASE+STM32_BKP_DR40_OFFSET)
#  define STM32_BKP_DR41         (STM32_BKP_BASE+STM32_BKP_DR41_OFFSET)
#  define STM32_BKP_DR42         (STM32_BKP_BASE+STM32_BKP_DR42_OFFSET)
#endif

/* Register Bitfield Definitions ****************************************************/

/* RTC clock calibration register */

#define BKP_RTCCR_CAL_SHIFT      (0)       /* Bits 6-0: Calibration value */
#define BKP_RTCCR_CAL_MASK       (0x7f << BKP_RTCCR_CAL_SHIFT)
#define BKP_RTCCR_CCO            (1 << 7)  /* Bit 7: Calibration Clock Output */
#define BKP_RTCCR_ASOE           (1 << 8)  /* Bit 8: Alarm or Second Output Enable */
#define BKP_RTCCR_ASOS           (1 << 9)  /* Bit 9: Alarm or Second Output Selection */

/* Backup control register */

#define BKP_CR_TPE               (1 << 0)  /* Bit 0: TAMPER pin enable */
#define BKP_CR_TPAL              (1 << 1)  /* Bit 1: TAMPER pin active level */

/* Backup control/status register */

#define BKP_CSR_CTE              (1 << 0)  /* Bit 0: Clear Tamper event */
#define BKP_CSR_CTI              (1 << 1)  /* Bit 1: Clear Tamper Interrupt */
#define BKP_CSR_TPIE             (1 << 2)  /* Bit 2: TAMPER Pin interrupt enable */
#define BKP_CSR_TEF              (1 << 8)  /* Bit 8: Tamper Event Flag */
#define BKP_CSR_TIF              (1 << 9)  /* Bit 9: Tamper Interrupt Flag */

/* Backup data register */

#define BKP_DR_SHIFT             (0)       /* Bits 1510: Backup data */
#define BKP_DR_MASK              (0xffff << BKP_DR_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_BKP_H */
